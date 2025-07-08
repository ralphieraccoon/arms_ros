#include "assembler/Part.hpp"

#include "assembler/ARMSConfig.hpp"

#include "assembler/Config.hpp"

#include <cmath>

/*  Input: ptr to another part

    Output: collision between parts yes/no

    First check the bounding boxes of the two parts to see if they intersect. If they do, use an OCC function
    to check the closest distance between them
*/
bool Part::collide(std::shared_ptr<Part> otherPart)
{
    float collisionThreshold = 0.01;                                
    Bnd_Box this_bb = ShapeBoundingBox(*shape_);
    Bnd_Box other_bb = ShapeBoundingBox(*(otherPart->getShape()));

    if (this_bb.IsOut(other_bb))
        return false;

    BRepExtrema_DistShapeShape distCalc(*shape_, *(otherPart->getShape()));
    
    if (!distCalc.IsDone()) 
    {
        std::cerr << "Collision can't be calculated" << std::endl;

        return true;  // Distance can't be computed - assume collision
    }

    if (distCalc.Value() < collisionThreshold)
        return true;
    
    else
        return false;
}

/*  Input: occupancy matrix for the parts bays

    Creates a substrate of appropriate size and positions it in the parts bay. Positions the part within the substrate and
    creates a negative. Saves a .stl copy of the negative.
*/
void Part::createNegativeAndPositionPart(std::vector<std::vector<bool>>& occupancy)
{
    double substrate_depth = 6;   
    int bay_size_index = 0;
    int bay_index = -1;                                 //The bay index for a given size
    Standard_Real x_size = ShapeAxisSize(*shape_, 0);
    Standard_Real y_size = ShapeAxisSize(*shape_, 1);

    //Select correct bay size depending on size of part
    
    if (x_size > 38 || y_size > 38)
    {
        bay_size_index = 1;
    }

    //Create the substrate and add a notch to the corner to indicate orientation

    TopoDS_Shape substrate = BRepPrimAPI_MakeBox(BAY_SIZES[bay_size_index], BAY_SIZES[bay_size_index], substrate_depth).Shape();    //TODO: box shape depends on part

    TopoDS_Shape notch = BRepPrimAPI_MakeCylinder(3, 2).Shape();

    notch = ShapeSetCentroid(notch, gp_Pnt(0, 0, ShapeHighestPoint(substrate)));

    substrate = BRepAlgoAPI_Cut(substrate, notch);

    //Find a free, appropriately sized bay (TODO ideally need to do this before creating the substrate so that you can use a larger bay if available)

    for (int i = 0; i < occupancy[bay_size_index].size(); i ++)
    {
        if (occupancy[bay_size_index][i] == false)
        {
            bay_index = i;
            break;
        }
    }

    if (bay_index == -1)
    {
        std::cerr << "No free bay" << std::endl;
        return;
    }

    occupancy[bay_size_index][bay_index] = true;

    //Position the substrate in the parts bay

    substrate = ShapeSetCentroid(substrate, gp_Pnt(PARTS_BAY_POSITIONS[bay_size_index][bay_index].X(), PARTS_BAY_POSITIONS[bay_size_index][bay_index].Y(), (substrate_depth / 2)));

    gp_Pnt substrate_centroid = ShapeCentroid(substrate);
    gp_Pnt shape_centroid = ShapeCentroid(*shape_);
    Standard_Real shape_min_z = ShapeLowestPoint(*shape_);
    Standard_Real substrate_min_z = ShapeLowestPoint(substrate);
    Standard_Real step_size = 1;

    //Position the shape so that the X and Y centers align and the base is 2mm above the substrate base

    gp_Vec shape_move(substrate_centroid.X() - shape_centroid.X(),
                      substrate_centroid.Y() - shape_centroid.Y(),
                      substrate_min_z - shape_min_z + 2);

    *shape_ = TranslateShape(*shape_, shape_move);

    shape_centroid = ShapeCentroid(*shape_);
    shape_min_z = ShapeLowestPoint(*shape_);

    //Create two boxes that are slightly larger than the shape
    TopoDS_Shape top_bound = BRepPrimAPI_MakeBox(ShapeAxisSize(*shape_, 0) + 1, ShapeAxisSize(*shape_, 1) + 1, ShapeAxisSize(*shape_, 2) + 1).Shape();
    TopoDS_Shape bottom_bound = BRepPrimAPI_MakeBox(ShapeAxisSize(*shape_, 0) + 1, ShapeAxisSize(*shape_, 1) + 1, ShapeAxisSize(*shape_, 2) + 1).Shape();

    //Set one box bottom to bottom of part and XY centroid pos to part XY centroid pos
    gp_Pnt top_centroid = ShapeCentroid(top_bound);
    Standard_Real top_min_z = ShapeLowestPoint(top_bound);

    gp_Vec top_move(shape_centroid.X() - top_centroid.X(), 
                    shape_centroid.Y() - top_centroid.Y(), 
                    shape_min_z - top_min_z);
    
    top_bound = TranslateShape(top_bound, top_move);

    //Set other box top to bottom of part minus step size, and XY centroid pos to part XY centroid pos
    gp_Pnt bottom_centroid = ShapeCentroid(bottom_bound);
    Standard_Real bottom_max_z = ShapeHighestPoint(bottom_bound);

    gp_Vec bottom_move(shape_centroid.X() - bottom_centroid.X(), 
                    shape_centroid.Y() - bottom_centroid.Y(), 
                    shape_min_z - bottom_max_z - step_size);
    
    bottom_bound = TranslateShape(bottom_bound, bottom_move);

    std::vector<TopoDS_Shape> slivers;

    //Step the boxes through the step size, cut the part with both of them
    //Find the bounding box of the sliver, scale it
    //Make sure the the x and y sizes are at least as big as the previous sliver
    //Add to the list of slivers
    for (int i = 0; i < 5; i ++)
    {
        std::cout << "making cuts" << std::endl;

        top_bound = TranslateShape(top_bound, gp_Vec(0, 0, step_size));
        bottom_bound = TranslateShape(bottom_bound, gp_Vec(0, 0, step_size));


        bool perform_first_cut = true;
        bool perform_second_cut = true;
        

        TopoDS_Shape first_intersect = ShapeIntersection(*shape_, bottom_bound);

        if (first_intersect.IsNull())
        {
            perform_first_cut = false;

            std::cout << "No first cut" << std::endl;
        }

        TopoDS_Shape second_intersect = ShapeIntersection(*shape_, top_bound);

        if (second_intersect.IsNull())
        {
            perform_second_cut = false;

            std::cout << "No second cut" << std::endl;
        }

        if (!perform_first_cut)
            break;

        // Perform the first subtraction: A - B
        
        std::cout << "subtraction 1" << std::endl;

        TopoDS_Shape first_cut = SubtractShapeBFromA(*shape_, bottom_bound);

        TopoDS_Shape both_cuts;

        if (perform_second_cut)
        {
            std::cout << "substraction 2" << std::endl;

            //Perform the second subtraction
            both_cuts = SubtractShapeBFromA(first_cut, top_bound);
        }

        else
        {
            both_cuts = first_cut;
        }

        if (ShapeBoundingBox(both_cuts).IsVoid())
            break;

        // std::stringstream ss;

        // ss << "sliver" << i << ".stl";

        // std::cout << "saving sliver" << std::endl;

        // SaveShapeAsSTL(both_cuts, ss.str());

        TopoDS_Shape both_cuts_bbox_shape = ShapeHighBoundingBoxShape(both_cuts, 5);

        // std::stringstream ss2;

        // ss2 << "sliver_bbox" << i << ".stl";

        // std::cout << "saving sliver bbox" << std::endl;

        // SaveShapeAsSTL(both_cuts_bbox_shape, ss2.str());

        Standard_Real x_size = ShapeAxisSize(both_cuts_bbox_shape, 0);

        Standard_Real y_size = ShapeAxisSize(both_cuts_bbox_shape, 1);

        Standard_Real scaling_factor;

        if (x_size < y_size)
        {
            scaling_factor = (x_size + 0.8) / x_size;
        }

        else
        {
            scaling_factor = (y_size + 0.8) / y_size;
        }

        TopoDS_Shape scaled_sliver = UniformScaleShape(both_cuts_bbox_shape, scaling_factor);

        //Now extrude upwards

        slivers.push_back(scaled_sliver);
    }

    int s = 0;

    for (TopoDS_Shape sliver : slivers)
    {
        //std::cout << "subtracting sliver" << std::endl;

        substrate = SubtractShapeBFromA(substrate, sliver);

        // std::cout << "Sliver width: " << ShapeAxisSize(sliver, 0) << std::endl;

        // std::cout << "Sliver height: " << ShapeAxisSize(sliver, 1) << std::endl;

        s++;
    }

    BRepMesh_IncrementalMesh(substrate, 0.1);  // Mesh with a 0.1 tolerance

    std::stringstream sub_ss;

    sub_ss << OUTPUT_DIR << name_ << "_sliced_cradle_size_index_" << bay_size_index << "_bay_index_" << bay_index << ".stl";

    // Export the result to STL
    StlAPI_Writer substrate_writer;
    substrate_writer.Write(substrate, sub_ss.str().c_str());
}

/*  
    Creates a simulated vacuum nozzle and searches for locations on the top face of a part where the nozzle has a strong overlap
*/
void Part::generateVacuumGraspPosition()
{
    TopoDS_Shape nozzle = BRepPrimAPI_MakeCylinder(4, 1);
    double nozzle_full_volume = ShapeVolume(nozzle);
    gp_Pnt part_com = ShapeCenterOfMass(*shape_);
    Standard_Real part_highest_point = ShapeHighestPoint(*shape_);
    double largest_shape_axis = std::max(ShapeAxisSize(*shape_, 0), ShapeAxisSize(*shape_, 1));
    double best_fit = 0;
    int best_r = 0;
    int best_th = 0;

    //Iterature over radius
    for (int r = 0; r < largest_shape_axis; r ++)
    {
        //Iterate over angle
        for (int th = 0; th < 360; th += 90)
        {
            gp_Pnt new_nozzle_pos(part_com.X() + r * cos(th * 3.14159 / 180),
                                part_com.Y() + r * sin(th * 3.14159 / 180),
                                part_highest_point - 0.51);

            TopoDS_Shape moved_nozzle = ShapeSetCentroid(nozzle, new_nozzle_pos);

            //Intersect the nozzle and the part
            TopoDS_Shape intersection = ShapeIntersection(moved_nozzle, *shape_);

            if (intersection.IsNull())
            {
                continue;
            }

            //Check the volume of the intersection
            double intersection_volume = ShapeVolume(intersection);

            double intersection_ratio = intersection_volume / nozzle_full_volume;

            if (intersection_ratio > best_fit + 0.01)
            {
                best_fit = intersection_ratio;
                best_r = r;
                best_th = th;
            }           
        }
    }

    //std::cout << "Intersection ratio: " << best_fit << std::endl;

    TopoDS_Shape visual_nozzle = BRepPrimAPI_MakeCylinder(4, 8);

    gp_Pnt visual_nozzle_pos(part_com.X() + best_r * cos(best_th * 3.14159 / 180),
                            part_com.Y() + best_r * sin(best_th * 3.14159 / 180),
                            part_highest_point - 0.51);

    visual_nozzle = ShapeSetCentroid(visual_nozzle, visual_nozzle_pos);

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    builder.Add(compound, *shape_);

    builder.Add(compound, visual_nozzle);

    BRepMesh_IncrementalMesh mesher(compound, 0.1);

    StlAPI_Writer writer;

    std::stringstream ss;

    ss << WORKING_DIR << name_ << "_vacuum_grasp.stl";

    writer.Write(compound, ss.str().c_str());

    vacuum_grasp_position_ = gp_Pnt(best_r * cos(best_th * 3.14159 / 180), best_r * sin(best_th * 3.14159 / 180), part_highest_point - part_com.Z());

    std::cout << "Finished grasp checking" << std::endl;
}


/*  
    TODO
*/
void Part::generatePPGGraspPosition()
{
    std::cout << "Generating PPG grasp" << std::endl;

    //Iterate through the faces of the part and get their normals

    //Disregard normals that don't have close to zero value in z
    //Check all remaining normals against one another to find pairs that align, that are close to the CoM, and that don't cause collisions

    std::vector<gp_Pnt> centers;
    std::vector<gp_Dir> normals;

    TopExp_Explorer faceExplorer(*shape_, TopAbs_FACE);

    for (; faceExplorer.More(); faceExplorer.Next()) {
        const TopoDS_Face& face = TopoDS::Face(faceExplorer.Current());

        // Get surface from face
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

        // Get parametric bounds of the face
        Standard_Real u1, u2, v1, v2;
        BRepTools::UVBounds(face, u1, u2, v1, v2);

        // Evaluate properties at the center of the parametric domain
        Standard_Real uMid = (u1 + u2) / 2.0;
        Standard_Real vMid = (v1 + v2) / 2.0;

        // Use SLProps to get surface properties including normal
        GeomLProp_SLProps props(surface, uMid, vMid, 1, Precision::Confusion());

        if (props.IsNormalDefined()) {
            gp_Pnt center = props.Value();
            gp_Dir normal = props.Normal();

            if (normal.Z() > 0.01 || normal.Z() < -0.01)
                continue;

            centers.push_back(center);
            normals.push_back(normal);

            //std::cout << "Face center: " << center.X() << ", " << center.Y() << ", " << center.Z() << std::endl;
            //std::cout << "Face normal: " << normal.X() << ", " << normal.Y() << ", " << normal.Z() << std::endl;
        }
    }

    bool grasp_found = false;

    for (int a = 0; a < normals.size(); a++)
    {
        for (int b = 0; b < normals.size(); b++)
        {
            Standard_Real normal_normal_angle = normals[a].Angle(normals[b]);

            //Check the the two normals are anti-aligned
            if (normal_normal_angle < 0.65 * 3.14159)
                continue;

            gp_Vec delta(centers[a], centers[b]);

            if (delta.Magnitude() < 1)
                continue;

            Standard_Real delta_normal_angle = gp_Vec(normals[b]).Angle(delta);

            //Check if the normal and the delta are misaligned
            if (delta_normal_angle > 0.35 * 3.14159)
                continue;

            //Check x y distance from center of delta to CoM
            gp_Pnt delta_center = centers[a].Translated(0.5 * delta);

            gp_Vec com_xy_distance(delta_center.X() - getCoM().X(), delta_center.Y() - getCoM().Y(), 0);

            if (com_xy_distance.Magnitude() > 5)
                continue;

            //Create padle models and check for collisions
            TopoDS_Shape paddle_1 = BRepPrimAPI_MakeBox(10, 2, 10).Shape();
            TopoDS_Shape paddle_2 = BRepPrimAPI_MakeBox(10, 2, 10).Shape();

            //1st rotation
            gp_Dir source_normal_1(0, 1, 0);
            gp_Dir target_normal_1 = normals[a];

            gp_Pnt source_point_1(5, 0, 5);
            gp_Pnt target_point_1 = centers[a].Translated(3 * gp_Vec(normals[a]));

            gp_Vec rotation_axis_1;

            if ((gp_Vec(source_normal_1) ^ gp_Vec(target_normal_1)).Z() > 0)
                rotation_axis_1 = gp_Vec(0, 0, 1);

            else
                rotation_axis_1 = gp_Vec(0, 0, -1);

            Standard_Real angle_1 = source_normal_1.Angle(target_normal_1); 

            gp_Trsf rotation_1;

            gp_Ax1 axis_1(source_point_1, gp_Dir(rotation_axis_1));  // rotate around axis passing through source point
            rotation_1.SetRotation(axis_1, angle_1);

            BRepBuilderAPI_Transform rotTransformer_1(rotation_1);
            rotTransformer_1.Perform(paddle_1);
            paddle_1 = rotTransformer_1.Shape();

            gp_Pnt rotated_source_point_1 = source_point_1.Transformed(rotation_1);  // new location of P1 after rotation TODO requierd?

            gp_Vec translation_vector_1(rotated_source_point_1, target_point_1);

            gp_Trsf translation_1;
            translation_1.SetTranslation(translation_vector_1);

            BRepBuilderAPI_Transform transTransformer_1(translation_1);
            transTransformer_1.Perform(paddle_1);
            paddle_1 = transTransformer_1.Shape();

            //Second rotation
            gp_Dir source_normal_2(0, 1, 0);
            gp_Dir target_normal_2 = normals[b];

            gp_Pnt source_point_2(5, 0, 5);
            gp_Pnt target_point_2 = centers[b].Translated(3 * gp_Vec(normals[b]));

            gp_Vec rotation_axis_2;

            if ((gp_Vec(source_normal_2) ^ gp_Vec(target_normal_2)).Z() > 0)
                rotation_axis_2 = gp_Vec(0, 0, 1);

            else
                rotation_axis_2 = gp_Vec(0, 0, -1);

            Standard_Real angle_2 = source_normal_2.Angle(target_normal_2); 

            gp_Trsf rotation_2;

            gp_Ax1 axis_2(source_point_2, gp_Dir(rotation_axis_2));  // rotate around axis passing through source point
            rotation_2.SetRotation(axis_2, angle_2);

            BRepBuilderAPI_Transform rotTransformer_2(rotation_2);
            rotTransformer_2.Perform(paddle_2);
            paddle_2 = rotTransformer_2.Shape();

            gp_Pnt rotated_source_point_2 = source_point_2.Transformed(rotation_2);  // new location of P2 after rotation TODO requierd?

            gp_Vec translation_vector_2(rotated_source_point_2, target_point_2);

            gp_Trsf translation_2;
            translation_2.SetTranslation(translation_vector_2);

            BRepBuilderAPI_Transform transTransformer_2(translation_2);
            transTransformer_2.Perform(paddle_2);
            paddle_2 = transTransformer_2.Shape();

            //Intersect the paddles and the part
            TopoDS_Shape intersection_1 = ShapeIntersection(paddle_1, *shape_);

            TopoDS_Shape intersection_2 = ShapeIntersection(paddle_2, *shape_);

            if ((!intersection_1.IsNull() && ShapeVolume(intersection_1) > 0.01) || (!intersection_2.IsNull() && ShapeVolume(intersection_2) > 0.01))
            {   
                //Collision between paddles and part, continue

                continue;
            }

            TopoDS_Compound compound;
            BRep_Builder builder;
            builder.MakeCompound(compound);

            builder.Add(compound, *shape_);

            builder.Add(compound, paddle_1);

            builder.Add(compound, paddle_2);

            BRepMesh_IncrementalMesh mesher(compound, 0.1);

            StlAPI_Writer writer;

            std::stringstream ss;

            ss << name_ << "_ppg_grasp_" << a << "_" << b << ".stl";

            writer.Write(compound, ss.str().c_str());



            std::cout << std::endl << "GRASP: " << std::endl;
            std::cout << "Face A center: " << centers[a].X() << ", " << centers[a].Y() << ", " << centers[a].Z() << std::endl;
            std::cout << "Face A normal: " << normals[a].X() << ", " << normals[a].Y() << ", " << normals[a].Z() << std::endl;
            std::cout << "Face B center: " << centers[b].X() << ", " << centers[b].Y() << ", " << centers[b].Z() << std::endl;
            std::cout << "Face B normal: " << normals[b].X() << ", " << normals[b].Y() << ", " << normals[b].Z() << std::endl << std::endl;;


        }
    }

    //TODO need to check collisions and 'size' of grasp
}