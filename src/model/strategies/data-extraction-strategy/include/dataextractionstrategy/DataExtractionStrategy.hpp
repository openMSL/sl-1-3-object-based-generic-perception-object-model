//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef DATA_EXTRACTION_STRATEGY_HPP
#define DATA_EXTRACTION_STRATEGY_HPP

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

class DataExtractionStrategy : public Strategy {

public:
    using Strategy::Strategy;
    void apply(osi3::SensorData &sensor_data) override;

private:

    static bool check_sensor_data_input(const osi3::SensorData &sensor_data, const Log &log, const Alert &alert);
    void process_vertices_from_one_object(const std::vector<Vector2d> &proj_vertices_current_obj, const Spherical3d& mean_vertex_position_of_current_object,
                                          const LidarDetectionData &detection_data_of_current_object, osi3::SensorData &sensor_data,
                                          const TF::EgoData &ego_data) const;
    static double get_rcs_in_sm(osi3::SensorData &sensor_data, uint64_t current_object_id);
    static double calc_visible_area(const std::vector<Vector2d> &proj_vertices_current_obj, double mean_vertex_distance_of_current_object);
    double calculate_irradiation_gain(double azimuth_angle_rad, double elevation_angle_rad) const;
    void transform_detections_to_logical_detections(osi3::SensorData &sensor_data, const osi3::LidarDetectionData &detection_data, const TF::EgoData &ego_data) const;
    };
#endif //DATA_EXTRACTION_STRATEGY_HPP
