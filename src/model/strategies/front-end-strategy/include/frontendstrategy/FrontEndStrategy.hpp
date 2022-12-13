//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef FRONT_END_STRATEGY_HPP
#define FRONT_END_STRATEGY_HPP

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <model/include/strategy.hpp>
#include <Dense>
#include <iostream>
#include <deque>
#include <stack>
#include <cstdlib>
#include <list>
#include <vector>
#include <algorithm>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

#include "osi_sensordata.pb.h"
#include "../../transformation-functions/TransformationFunctions.hpp"


using namespace model;

struct GroundTruthObject {
    osi3::MovingObject osi_gt_object;
    osi3::BaseMoving base_ego_coord;
    osi3::BaseMoving base_sensor_coord;
    osi3::Spherical3d position_spherical_sensor_coord;
    std::vector<osi3::Vector3d> bounding_box_vertices;  //vertices of the class dependent refined bounding box with respect to length, width and height
    std::vector<std::vector<int>> bounding_box_surfaces;    //indices of bounding box vertices forming a planar surface area on refined bounding box
    std::vector<osi3::Vector3d> bounding_box_vertices_sensor_coord;
    std::vector<osi3::Spherical3d> visible_bounding_box_vertices_sensor_coord;

    bool operator() (const GroundTruthObject& i, const GroundTruthObject& j) { return (i.position_spherical_sensor_coord.distance() < j.position_spherical_sensor_coord.distance());}
};

class FrontEndStrategy : public Strategy {
public:
    using Strategy::Strategy;
    void apply(osi3::SensorData &sensor_data) override;

private:
    static void check_sensor_data_input(const osi3::SensorData &sensor_data, const Alert &alert);
    static void set_sensor_data_timestamp(osi3::SensorData &sensor_data, const osi3::SensorView &input_sensor_view, const Alert &alert);
    static bool simulate_sensor_failure(osi3::SensorData &sensor_data, const Profile &profile, const Log &log);
    static std::vector<GroundTruthObject> bring_ground_truth_objects_to_unified_format(const osi3::SensorView &input_sensor_view, const TF::EgoData& ego_data, const Profile &profile, const Alert &alert);
    static GroundTruthObject get_stationary_object_from_ground_truth(const osi3::StationaryObject &input_object, const TF::EgoData &ego_data, const MountingPosition &mounting_pose);
    static GroundTruthObject get_moving_object_from_ground_truth(const osi3::MovingObject &input_object, const TF::EgoData &ego_data, const osi3::MountingPosition &mounting_pose);
    static void apply_noise_to_visible_vertices(std::vector<GroundTruthObject>& ground_truth_object_list, const Profile& profile);
    static void write_vertices_to_logical_detections(osi3::SensorData &sensor_data, std::vector<GroundTruthObject> &ground_truth_object_list, const TF::EgoData& ego_data);
    static void write_visible_vertices_to_detections(osi3::SensorData &sensor_data, std::vector<GroundTruthObject> &ground_truth_object_list, const TF::EgoData& ego_data);
};
#endif //FRONT_END_STRATEGY_HPP
