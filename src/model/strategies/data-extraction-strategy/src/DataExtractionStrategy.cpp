//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef Speed_of_Light
#define Speed_of_Light 299792458
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "dataextractionstrategy/DataExtractionStrategy.hpp"
#include <random>

using namespace model;

void DataExtractionStrategy::apply(osi3::SensorData &sensor_data) {
    log("Starting Data Extraction");

    const osi3::SensorView &input_sensor_view = sensor_data.sensor_view(0);
    if (!check_sensor_data_input(sensor_data, log, alert)) {
        return;
    }
    if(!sensor_data.has_feature_data()){
        return;
    }
    if(sensor_data.feature_data().lidar_sensor(0).detection_size() == 0){
        return;
    }
    TF::EgoData ego_data;
    TF::get_ego_info(ego_data, input_sensor_view);

    //iterate over detections (vertices)
    auto& sensor = sensor_data.feature_data().lidar_sensor(0);
    uint64_t current_object_id = sensor.detection(0).object_id().value();
    std::vector<Vector2d> proj_vertices_current_obj;
    LidarDetectionData detection_data_of_current_object;
    Spherical3d mean_vertex_position;
    for(int detection_idx = 0; detection_idx < sensor.detection().size(); detection_idx++) {
        if(sensor.detection(detection_idx).object_id().value() != current_object_id) {
            mean_vertex_position.set_distance(mean_vertex_position.distance()/((double)proj_vertices_current_obj.size()));
            mean_vertex_position.set_azimuth(mean_vertex_position.azimuth()/((double)proj_vertices_current_obj.size()));
            mean_vertex_position.set_elevation(mean_vertex_position.elevation()/((double)proj_vertices_current_obj.size()));
            process_vertices_from_one_object(proj_vertices_current_obj, mean_vertex_position, detection_data_of_current_object, sensor_data, ego_data);

            current_object_id = sensor.detection(detection_idx).object_id().value();
            mean_vertex_position.clear_azimuth();
            mean_vertex_position.clear_elevation();
            mean_vertex_position.clear_distance();
            proj_vertices_current_obj.clear();
            detection_data_of_current_object.clear_detection();
        }
        Vector2d current_detection_position_on_plane;
        TF::projection_onto_unit_distance_cylinder(sensor.detection(detection_idx).position(), current_detection_position_on_plane);
        proj_vertices_current_obj.emplace_back(current_detection_position_on_plane);
        auto current_detection = detection_data_of_current_object.add_detection();
        current_detection->CopyFrom(sensor.detection(detection_idx));
        mean_vertex_position.set_distance(mean_vertex_position.distance()+sensor.detection(detection_idx).position().distance());
        mean_vertex_position.set_azimuth(mean_vertex_position.azimuth()+sensor.detection(detection_idx).position().azimuth());
        mean_vertex_position.set_elevation(mean_vertex_position.elevation()+sensor.detection(detection_idx).position().elevation());
    }
    mean_vertex_position.set_distance(mean_vertex_position.distance()/((double)proj_vertices_current_obj.size()));
    mean_vertex_position.set_azimuth(mean_vertex_position.azimuth()/((double)proj_vertices_current_obj.size()));
    mean_vertex_position.set_elevation(mean_vertex_position.elevation()/((double)proj_vertices_current_obj.size()));
    process_vertices_from_one_object(proj_vertices_current_obj, mean_vertex_position, detection_data_of_current_object, sensor_data, ego_data);

} // void apply()

bool DataExtractionStrategy::check_sensor_data_input(const osi3::SensorData &sensor_data, const Log &log, const Alert &alert) {
    bool check_passed = true;
    if (sensor_data.sensor_view_size() == 0) {
        alert("no sensor view");
        check_passed = false;
    }
    if (sensor_data.sensor_view(0).global_ground_truth().moving_object_size() == 0) {
        log("moving objects empty");
        check_passed = false;
    }
    return check_passed;
}

void DataExtractionStrategy::process_vertices_from_one_object(const std::vector<Vector2d> &proj_vertices_current_obj, const Spherical3d& mean_vertex_position_of_current_object,
                                                              const LidarDetectionData &detection_data_of_current_object, osi3::SensorData &sensor_data,
                                                              const TF::EgoData &ego_data) const {
    double wave_length;
    double power_equivalent_area;
    float equivalent_reflecting_area_threshold;
    double detection_threshold_dB;
    if(!profile.sensor_view_configuration.radar_sensor_view_configuration().empty()) { // radar
        wave_length = Speed_of_Light / profile.sensor_view_configuration.radar_sensor_view_configuration(0).emitter_frequency();
        float rcs_dbsm_threshold = 10;
        equivalent_reflecting_area_threshold = (float)pow(10.0, rcs_dbsm_threshold / 10);
        detection_threshold_dB = float(10.0 * log10(pow(wave_length, 2) * equivalent_reflecting_area_threshold / pow(profile.data_extraction_parameters.reference_range_in_m, 4)));
        power_equivalent_area = get_rcs_in_sm(sensor_data, detection_data_of_current_object.detection(0).object_id().value());

    } else { // lidar
        wave_length = Speed_of_Light / profile.sensor_view_configuration.lidar_sensor_view_configuration(0).emitter_frequency();
        equivalent_reflecting_area_threshold = 2.5;
        detection_threshold_dB = float(10.0 * log10(pow(wave_length, 2) * equivalent_reflecting_area_threshold / pow(profile.data_extraction_parameters.reference_range_in_m, 4)));
        power_equivalent_area = calc_visible_area(proj_vertices_current_obj, mean_vertex_position_of_current_object.distance());
    }

    double irradiation_gain = calculate_irradiation_gain(mean_vertex_position_of_current_object.azimuth(), mean_vertex_position_of_current_object.elevation());

    double distance = mean_vertex_position_of_current_object.distance();

    auto power_equivalent_value_db = double(10.0 * log10(std::pow(wave_length, 2) * irradiation_gain * power_equivalent_area / std::pow(distance, 4)));
    if (profile.data_extraction_parameters.detection_threshold_dB_stdv > 0.0) {
		std::normal_distribution<double> distribution(0.0, profile.data_extraction_parameters.detection_threshold_dB_stdv); // dist(mean, stddev), normal distribution with stddev in dB
		std::knuth_b generator(std::rand()); // rand used for Windows compatibility
        detection_threshold_dB += distribution(generator);
	}
    bool is_detected = mean_vertex_position_of_current_object.distance() < profile.data_extraction_parameters.max_range_in_m &&
                       power_equivalent_value_db > detection_threshold_dB;

    if(is_detected) {
        transform_detections_to_logical_detections(sensor_data, detection_data_of_current_object, ego_data);
    }
}

double DataExtractionStrategy::get_rcs_in_sm(osi3::SensorData &sensor_data, uint64_t current_object_id) {
    double rcs_dbsm = 0;

    for(auto& current_object : sensor_data.sensor_view(0).global_ground_truth().moving_object()) {
        if(current_object.id().value() == current_object_id) {
            if (current_object.type() == osi3::MovingObject_Type_TYPE_VEHICLE) {
                if (current_object.vehicle_classification().type() <= osi3::MovingObject_VehicleClassification::TYPE_LUXURY_CAR) {
                    rcs_dbsm = 10;
                } else if (current_object.vehicle_classification().type() <= osi3::MovingObject_VehicleClassification::TYPE_TRAILER) {
                    rcs_dbsm = 45;
                } else if (current_object.vehicle_classification().type() == osi3::MovingObject_VehicleClassification::TYPE_MOTORBIKE) {
                    rcs_dbsm = 0;
                } else if (current_object.vehicle_classification().type() == osi3::MovingObject_VehicleClassification::TYPE_BICYCLE) {
                    rcs_dbsm = -5;
                } else {
                    rcs_dbsm = 40;
                }
            } else if (current_object.type() == osi3::MovingObject_Type_TYPE_PEDESTRIAN) {
                rcs_dbsm = -10;
            }
            break;
        }
    }
    return pow(10.0, rcs_dbsm/10);
}

double DataExtractionStrategy::calc_visible_area(const std::vector<Vector2d> &proj_vertices_current_obj, double mean_vertex_distance_of_current_object) {
    double visible_area;
    if(proj_vertices_current_obj.size() > 2) {
        double projected_area_on_cylinder = proj_vertices_current_obj.at(0).x() * (proj_vertices_current_obj.at(1).y()-proj_vertices_current_obj.back().y());
        for(int vertex_idx = 1; vertex_idx < (proj_vertices_current_obj.size()-1); vertex_idx++){
            projected_area_on_cylinder += proj_vertices_current_obj.at(vertex_idx).x() * (proj_vertices_current_obj.at(vertex_idx+1).y()-proj_vertices_current_obj.at(vertex_idx-1).y());
        }
        projected_area_on_cylinder += proj_vertices_current_obj.back().x() * (proj_vertices_current_obj.at(0).y()-proj_vertices_current_obj.back().y());
        projected_area_on_cylinder = std::abs(projected_area_on_cylinder/2);
        visible_area = projected_area_on_cylinder * (std::pow(mean_vertex_distance_of_current_object, 2));
    } else {
        visible_area = 0;
    }
    return visible_area;
}

int closest_high(const std::vector<float> &vec, double value) {
    auto const it = std::upper_bound(vec.begin(), vec.end(), value);
    if(it == vec.end()) {
        return (int)std::distance(vec.begin(), it)-1;
    }
    return std::distance(vec.begin(), it);
}

float get_interpolation_factor(double input_angle, float lower_bound, float upper_bound) {
    return ((float)input_angle - lower_bound)/(upper_bound-lower_bound);
}

double interpolate_gain(float interpolation_factor, double lower_bound, double upper_bound) {
    return lower_bound + interpolation_factor*(upper_bound-lower_bound);
}

double DataExtractionStrategy::calculate_irradiation_gain(double azimuth_angle_rad, double elevation_angle_rad) const {
    double irradiation_gain;
    double elevation_angle_deg = elevation_angle_rad*180.0/M_PI;
    double azimuth_angle_deg = azimuth_angle_rad*180.0/M_PI;
    std::vector<std::vector<double>> elevation_azimuth_gain = profile.data_extraction_parameters.irradiation_pattern.elevation_azimuth_power_values;
    std::vector<float> elevation_pattern = profile.data_extraction_parameters.irradiation_pattern.elevation;
    std::vector<float> azimuth_pattern = profile.data_extraction_parameters.irradiation_pattern.azimuth;

    int upper_bound_elevation_index = closest_high(elevation_pattern, elevation_angle_deg);
    int upper_bound_azimuth_index = closest_high(azimuth_pattern, azimuth_angle_deg);
    if(upper_bound_azimuth_index > 0 && upper_bound_azimuth_index < azimuth_pattern.size() && upper_bound_elevation_index > 0 && upper_bound_elevation_index < elevation_pattern.size()) {
        float elevation_interpolation = get_interpolation_factor(elevation_angle_deg, elevation_pattern.at(upper_bound_elevation_index-1), elevation_pattern.at(upper_bound_elevation_index));
        float azimuth_interpolation = get_interpolation_factor(azimuth_angle_deg, azimuth_pattern.at(upper_bound_azimuth_index-1), azimuth_pattern.at(upper_bound_azimuth_index));

        double gain_low_elevation_low_azimuth = elevation_azimuth_gain.at(upper_bound_elevation_index-1).at(upper_bound_azimuth_index-1);
        double gain_low_elevation_high_azimuth = elevation_azimuth_gain.at(upper_bound_elevation_index-1).at(upper_bound_azimuth_index);
        double gain_low_elevation = interpolate_gain(azimuth_interpolation, gain_low_elevation_low_azimuth, gain_low_elevation_high_azimuth);

        double gain_high_elevation_low_azimuth = elevation_azimuth_gain.at(upper_bound_elevation_index).at(upper_bound_azimuth_index-1);
        double gain_high_elevation_high_azimuth = elevation_azimuth_gain.at(upper_bound_elevation_index).at(upper_bound_azimuth_index);
        double gain_high_elevation = interpolate_gain(azimuth_interpolation, gain_high_elevation_low_azimuth, gain_high_elevation_high_azimuth);

        irradiation_gain = interpolate_gain(elevation_interpolation, gain_low_elevation, gain_high_elevation);
    } else {
        irradiation_gain = 0;
    }
    return irradiation_gain;
}


void DataExtractionStrategy::transform_detections_to_logical_detections(osi3::SensorData &sensor_data, const osi3::LidarDetectionData &detection_data, const TF::EgoData &ego_data) const {
    osi3::Vector3d mounting_position;
    osi3::Orientation3d mounting_orientation;
    if(!profile.sensor_view_configuration.radar_sensor_view_configuration().empty()) { // radar
        mounting_position.CopyFrom(profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position().position());
        mounting_orientation.CopyFrom(profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position().orientation());
    } else { // lidar
        mounting_position.CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position().position());
        mounting_orientation.CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position().orientation());
    }

    for(auto& current_detection : detection_data.detection()) {
        double elevation = current_detection.position().elevation();
        double azimuth   = current_detection.position().azimuth();
        double distance  = current_detection.position().distance();

        osi3::Vector3d point_cartesian_sensor;
        point_cartesian_sensor.set_x(distance * cos(elevation) * cos(azimuth));
        point_cartesian_sensor.set_y(distance * cos(elevation) * sin(azimuth));
        point_cartesian_sensor.set_z(distance * sin(elevation));
        osi3::Vector3d point_cartesian_vehicle = TF::transform_from_local_coordinates(point_cartesian_sensor, mounting_orientation, mounting_position);

        auto current_logical_detection = sensor_data.mutable_logical_detection_data()->add_logical_detection();
        current_logical_detection->mutable_position()->set_x(point_cartesian_vehicle.x());
        current_logical_detection->mutable_position()->set_y(point_cartesian_vehicle.y());
        current_logical_detection->mutable_position()->set_z(point_cartesian_vehicle.z());
        current_logical_detection->set_intensity(current_detection.intensity());
        current_logical_detection->mutable_object_id()->set_value(current_detection.object_id().value());
    }
}