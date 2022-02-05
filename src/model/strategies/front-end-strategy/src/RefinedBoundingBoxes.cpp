//
// Copyright Clemens Linnhoff, M. Sc.
// Institute of Automotive Engineering
// of Technical University of Darmstadt, 2021.
// Licensed under the EUPL-1.2-or-later
//
// This work covered by the EUPL can be used/merged and distributed
// in other works covered by GPL-2.0, GPL-3.0, LGPL, AGPL, CeCILL,
// OSL, EPL, MPL and other licences listed as compatible in the EUPL
// Appendix. This applies to the other (combined) work, while the
// original project stays covered by the EUPL without re-licensing.
//
// Alternatively, the contents of this file may be used under the
// terms of the Mozilla Public License, v. 2.0. If a copy of the MPL
// was not distributed with this file, you can obtain one at
// http://mozilla.org/MPL/2.0/.
//

#include "frontendstrategy/RefinedBoundingBoxes.hpp"

void set_refined_bounding_boxes(std::vector<GroundTruthObject> &ground_truth_object_list, TF::EgoData &ego_data, const Profile &profile, const Log &log, const Alert &alert) {
    for(auto &current_object : ground_truth_object_list) {
        std::vector<std::vector<float>> bounding_box_definition;
        std::vector<std::vector<int>> surfaces_definition;
        get_refined_bounding_box_definition(bounding_box_definition, surfaces_definition, current_object.osi_gt_object.type(), current_object.osi_gt_object.vehicle_classification());
        create_refined_bounding_box(bounding_box_definition, current_object, ego_data, profile, alert);
        current_object.bounding_box_surfaces = surfaces_definition;
    }
}

void get_refined_bounding_box_definition(std::vector<std::vector<float>> &bounding_box_definition, std::vector<std::vector<int>> &surfaces_definition, MovingObject::Type type, const MovingObject::VehicleClassification &classification) {
    //TODO: consider ground clearance
    if(type == osi3::MovingObject_Type_TYPE_VEHICLE) {
        bounding_box_definition = {{0.5, 0.5, -0.5},    //0
                                   {0.5, -0.5, -0.5},   //1
                                   {-0.5, -0.5, -0.5},  //2
                                   {-0.5, 0.5, -0.5},   //3
                                   {0.5, 0.5, 0.0},     //4
                                   {0.5, -0.5, 0.0},    //5
                                   {0.22, 0.5, 0.15},     //6
                                   {0.22, -0.5, 0.15},    //7
                                   {0.1, 0.5, 0.5},    //8
                                   {0.1, -0.5, 0.5},   //9
                                   {-0.5, -0.5, 0.5},   //10
                                   {-0.5, 0.5, 0.5},    //11
                                   };
        surfaces_definition = {{2, 1, 5, 7, 9, 10},
                               {1, 0, 4, 5},
                               {3, 0, 4, 6, 8, 11},
                               {3, 2, 10, 11},
                               {10, 9, 8, 11},
                               {9, 7, 6, 8},
                               {7, 5, 4, 6},
                               {2, 1, 0, 3}};
    } else {
        bounding_box_definition = {{0.5, 0.5, 0.5},
                                   {0.5, 0.5, -0.5},
                                   {0.5, -0.5, 0.5},
                                   {0.5, -0.5, -0.5},
                                   {-0.5, 0.5, 0.5},
                                   {-0.5, 0.5, -0.5},
                                   {-0.5, -0.5, 0.5},
                                   {-0.5, -0.5, -0.5}};
        surfaces_definition = {{7, 3, 2, 6},
                               {3, 1, 0, 2},
                               {2, 1, 0, 4},
                               {7, 5, 4, 6},
                               {6, 2, 0, 4},
                               {7, 3, 1, 5}};
    }
}

void create_refined_bounding_box(std::vector<std::vector<float>> &bounding_box_definition, GroundTruthObject &current_object, TF::EgoData &ego_data, const Profile &profile, const Alert &alert) {
    for(auto &current_vertex_definition : bounding_box_definition) {
        osi3::Vector3d current_vertex;
        current_vertex.set_x(current_vertex_definition.at(0) * current_object.osi_gt_object.base().dimension().length());
        current_vertex.set_y(current_vertex_definition.at(1) * current_object.osi_gt_object.base().dimension().width());
        current_vertex.set_z(current_vertex_definition.at(2) * current_object.osi_gt_object.base().dimension().height());
        current_object.bounding_box_vertices.emplace_back(current_vertex);

        osi3::Vector3d current_vertex_world_coord = TF::transform_from_local_coordinates(current_vertex, current_object.osi_gt_object.base().orientation(), current_object.osi_gt_object.base().position());
        osi3::Vector3d current_vertex_ego_coord = TF::transform_position_from_world_to_ego_coordinates(current_vertex_world_coord, ego_data);
        osi3::MountingPosition mounting_pose;
        if(!profile.sensor_view_configuration.radar_sensor_view_configuration().empty()) { // radar
            mounting_pose.CopyFrom(profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position());
        } else if(!profile.sensor_view_configuration.lidar_sensor_view_configuration().empty()) { // lidar
            mounting_pose.CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position());
        } else {
            alert("No lidar or radar sensor view in profile!");
        }
        osi3::Vector3d current_vertex_sensor_coord = TF::transform_position_from_world_to_sensor_coordinates(current_vertex_world_coord, ego_data, mounting_pose);
        current_object.bounding_box_vertices_sensor_coord.emplace_back(current_vertex_sensor_coord);
    }
}

