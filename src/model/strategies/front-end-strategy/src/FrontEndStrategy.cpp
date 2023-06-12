//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "frontendstrategy/FrontEndStrategy.hpp"

#include <random>

#include "frontendstrategy/Occlusion.h"
#include "frontendstrategy/RefinedBoundingBoxes.hpp"

using namespace model;

void FrontEndStrategy::apply(osi3::SensorData& sensor_data)
{
    log("Starting Front-End");
    std::cout << "Starting Front-End" << std::endl;

    check_sensor_data_input(sensor_data, alert);
    const osi3::SensorView& input_sensor_view = sensor_data.sensor_view(0);

    // get ego information
    TF::EgoData ego_data;
    TF::get_ego_info(ego_data, input_sensor_view);
    set_sensor_data_timestamp(sensor_data, input_sensor_view, alert);

    if (!simulate_sensor_failure(sensor_data, profile, log))
    {
        std::vector<GroundTruthObject> ground_truth_object_list = FrontEndStrategy::bring_ground_truth_objects_to_unified_format(input_sensor_view, ego_data, profile, alert);

        set_refined_bounding_boxes(ground_truth_object_list, ego_data, profile, alert);
        get_visible_vertices(ground_truth_object_list, profile, log, alert);
        apply_noise_to_visible_vertices(ground_truth_object_list, profile);
        write_visible_vertices_to_detections(sensor_data, ground_truth_object_list);

        // write_vertices_to_logical_detections(sensor_data, ground_truth_object_list, ego_data); //DEBUG
    }
}

void FrontEndStrategy::check_sensor_data_input(osi3::SensorData& sensor_data, const Alert& alert)
{
    if (sensor_data.sensor_view().empty())
    {
        alert("no sensor view");
    }
    if (sensor_data.sensor_view(0).global_ground_truth().moving_object().empty())
    {
        alert("GT moving objects empty");
    }
    if (!sensor_data.sensor_view(0).has_mounting_position())
    {
        alert("Input sensor_view does not contain a mounting position. Mounting position replaced with values from profile");
        sensor_data.mutable_sensor_view(0)->mutable_mounting_position()->mutable_position()->set_x(profile.sensor_view_configuration.mounting_position().position().x());
        sensor_data.mutable_sensor_view(0)->mutable_mounting_position()->mutable_position()->set_y(profile.sensor_view_configuration.mounting_position().position().y());
        sensor_data.mutable_sensor_view(0)->mutable_mounting_position()->mutable_position()->set_z(profile.sensor_view_configuration.mounting_position().position().z());
        sensor_data.mutable_sensor_view(0)->mutable_mounting_position()->mutable_orientation()->set_yaw(profile.sensor_view_configuration.mounting_position().orientation().yaw());
        sensor_data.mutable_sensor_view(0)->mutable_mounting_position()->mutable_orientation()->set_pitch(profile.sensor_view_configuration.mounting_position().orientation().pitch());
        sensor_data.mutable_sensor_view(0)->mutable_mounting_position()->mutable_orientation()->set_roll(profile.sensor_view_configuration.mounting_position().orientation().roll());
    }
    if (profile.sensor_view_configuration.lidar_sensor_view_configuration_size() > 0)
    {
        bool replace_lidar_mounting = false;
        if (sensor_data.sensor_view(0).lidar_sensor_view_size() > 0)
        {
            auto lidar_sensor_view_config = sensor_data.sensor_view(0).lidar_sensor_view(0).view_configuration();
            if (lidar_sensor_view_config.has_mounting_position())
            {
                const auto& mounting_position_in = lidar_sensor_view_config.mounting_position();
                const auto& mounting_position_profile = profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position();
                if (mounting_position_in.position().x() != mounting_position_profile.position().x() ||
                    mounting_position_in.position().y() != mounting_position_profile.position().y() ||
                    mounting_position_in.position().z() != mounting_position_profile.position().z() ||
                    mounting_position_in.orientation().yaw() != mounting_position_profile.orientation().yaw() ||
                    mounting_position_in.orientation().pitch() != mounting_position_profile.orientation().pitch() ||
                    mounting_position_in.orientation().roll() != mounting_position_profile.orientation().roll())
                {
                    alert("Input lidar_sensor_view_config mounting position different from profile. Input mounting position will be used.");
                }
            }
            else
            {
                alert("Input lidar_sensor_view_config does not contain a mounting position. Mounting position replaced with values from profile");
                replace_lidar_mounting = true;
            }
        }
        else
        {
            alert("Input lidar_sensor_view does not contain a view_configuration. Mounting position replaced with values from profile.");

            replace_lidar_mounting = true;
            sensor_data.mutable_sensor_view(0)->add_lidar_sensor_view();
        }
        if (replace_lidar_mounting)
        {
            auto *mounting_position_out = sensor_data.mutable_sensor_view(0)->mutable_lidar_sensor_view(0)->mutable_view_configuration()->mutable_mounting_position();
            const auto& mounting_position_profile = profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position();
            mounting_position_out->mutable_position()->set_x(mounting_position_profile.position().x());
            mounting_position_out->mutable_position()->set_y(mounting_position_profile.position().y());
            mounting_position_out->mutable_position()->set_z(mounting_position_profile.position().z());
            mounting_position_out->mutable_orientation()->set_yaw(mounting_position_profile.orientation().yaw());
            mounting_position_out->mutable_orientation()->set_pitch(mounting_position_profile.orientation().pitch());
            mounting_position_out->mutable_orientation()->set_roll(mounting_position_profile.orientation().roll());
        }
    }
    if (profile.sensor_view_configuration.radar_sensor_view_configuration_size() > 0)
    {
        bool replace_radar_mounting = false;
        if (sensor_data.sensor_view(0).radar_sensor_view_size() > 0)
        {
            auto radar_sensor_view_config = sensor_data.sensor_view(0).radar_sensor_view(0).view_configuration();
            if (radar_sensor_view_config.has_mounting_position())
            {
                const auto& mounting_position_in = radar_sensor_view_config.mounting_position();
                const auto& mounting_position_profile = profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position();
                if (mounting_position_in.position().x() != mounting_position_profile.position().x() ||
                    mounting_position_in.position().y() != mounting_position_profile.position().y() ||
                    mounting_position_in.position().z() != mounting_position_profile.position().z() ||
                    mounting_position_in.orientation().yaw() != mounting_position_profile.orientation().yaw() ||
                    mounting_position_in.orientation().pitch() != mounting_position_profile.orientation().pitch() ||
                    mounting_position_in.orientation().roll() != mounting_position_profile.orientation().roll())
                {
                    alert("Input radar_sensor_view_config mounting position different from profile. Input mounting position will be used.");
                }
            }
            else
            {
                alert("Input radar_sensor_view_config does not contain a mounting position. Mounting position replaced with values from profile");
                replace_radar_mounting = true;
            }
        }
        else
        {
            alert("Input radar_sensor_view does not contain a view_configuration. Mounting position replaced with values from profile.");

            replace_radar_mounting = true;
            sensor_data.mutable_sensor_view(0)->add_radar_sensor_view();
        }
        if (replace_radar_mounting)
        {
            auto *mounting_position_out = sensor_data.mutable_sensor_view(0)->mutable_radar_sensor_view(0)->mutable_view_configuration()->mutable_mounting_position();
            const auto& mounting_position_profile = profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position();
            mounting_position_out->mutable_position()->set_x(mounting_position_profile.position().x());
            mounting_position_out->mutable_position()->set_y(mounting_position_profile.position().y());
            mounting_position_out->mutable_position()->set_z(mounting_position_profile.position().z());
            mounting_position_out->mutable_orientation()->set_yaw(mounting_position_profile.orientation().yaw());
            mounting_position_out->mutable_orientation()->set_pitch(mounting_position_profile.orientation().pitch());
            mounting_position_out->mutable_orientation()->set_roll(mounting_position_profile.orientation().roll());
        }
    }
}

void FrontEndStrategy::set_sensor_data_timestamp(osi3::SensorData& sensor_data, const osi3::SensorView& input_sensor_view, const Alert& alert)
{
    if (input_sensor_view.global_ground_truth().has_timestamp())
    {
        sensor_data.mutable_timestamp()->CopyFrom(input_sensor_view.global_ground_truth().timestamp());
    }
    else
    {
        alert("global_ground_truth has no timestamp.");
    }
}

bool FrontEndStrategy::simulate_sensor_failure(osi3::SensorData& sensor_data, const Profile& profile, const Log& log)
{
    if (profile.sensor_parameters.simulate_sensor_failure)
    {
        if (sensor_data.timestamp().seconds() >= profile.sensor_parameters.stop_detection_time)
        {
            sensor_data.mutable_stationary_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_NOT_AVAILABLE);
            sensor_data.mutable_moving_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_NOT_AVAILABLE);
            log("Data Qualifier: not available.");
            return true;
        }
    }
    sensor_data.mutable_stationary_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);
    sensor_data.mutable_moving_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);
    return false;
}

std::vector<GroundTruthObject> FrontEndStrategy::bring_ground_truth_objects_to_unified_format(const osi3::SensorView& input_sensor_view,
                                                                                              const TF::EgoData& ego_data,
                                                                                              const Profile& profile,
                                                                                              const Alert& alert)
{
    std::vector<GroundTruthObject> ground_truth_object_list;

    MountingPosition mounting_pose;
    if (!profile.sensor_view_configuration.radar_sensor_view_configuration().empty())
    {  // radar
        mounting_pose.CopyFrom(profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position());
    }
    else if (!profile.sensor_view_configuration.lidar_sensor_view_configuration().empty())
    {  // lidar
        mounting_pose.CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position());
    }
    else
    {
        alert("No lidar or radar sensor view in profile!");
    }

    for (const auto& current_object : input_sensor_view.global_ground_truth().stationary_object())
    {
        GroundTruthObject gt_object = FrontEndStrategy::get_stationary_object_from_ground_truth(current_object, ego_data, mounting_pose);
        ground_truth_object_list.push_back(gt_object);
    }

    for (const auto& current_object : input_sensor_view.global_ground_truth().moving_object())
    {
        if (current_object.id().value() != input_sensor_view.global_ground_truth().host_vehicle_id().value())
        {
            GroundTruthObject gt_object = FrontEndStrategy::get_moving_object_from_ground_truth(current_object, ego_data, mounting_pose);
            ground_truth_object_list.push_back(gt_object);
        }
    }

    for (const auto& current_object : input_sensor_view.global_ground_truth().traffic_sign())
    {
        // TODO
    }

    for (const auto& current_object : input_sensor_view.global_ground_truth().traffic_light())
    {
        // TODO
    }

    return ground_truth_object_list;
}

GroundTruthObject FrontEndStrategy::get_stationary_object_from_ground_truth(const osi3::StationaryObject& input_object,
                                                                            const TF::EgoData& ego_data,
                                                                            const MountingPosition& mounting_pose)
{
    GroundTruthObject gt_object;
    gt_object.osi_gt_object.mutable_base()->mutable_position()->CopyFrom(input_object.base().position());
    gt_object.osi_gt_object.mutable_base()->mutable_orientation()->CopyFrom(input_object.base().orientation());
    gt_object.osi_gt_object.mutable_base()->mutable_dimension()->CopyFrom(input_object.base().dimension());
    gt_object.osi_gt_object.set_type(osi3::MovingObject_Type_TYPE_OTHER);
    gt_object.osi_gt_object.mutable_id()->set_value(input_object.id().value());
    gt_object.osi_gt_object.mutable_vehicle_attributes()->set_ground_clearance(0.0);

    // ego coordinates
    gt_object.base_ego_coord.mutable_position()->CopyFrom(
        TF::transform_to_local_coordinates(input_object.base().position(), ego_data.ego_base.orientation(), ego_data.ego_base.position()));
    gt_object.base_ego_coord.mutable_orientation()->CopyFrom(TF::calc_relative_orientation_to_local(input_object.base().orientation(), ego_data.ego_base.orientation()));

    // sensor coordinates
    osi3::Vector3d position_sensor_coord = TF::transform_position_from_world_to_sensor_coordinates(input_object.base().position(), ego_data, mounting_pose);
    gt_object.base_sensor_coord.mutable_position()->CopyFrom(position_sensor_coord);
    gt_object.position_spherical_sensor_coord = TF::transform_cartesian_to_spherical(position_sensor_coord);

    osi3::Orientation3d orientation_sensor_coord = TF::transform_orientation_from_world_to_sensor_coordinates(input_object.base().orientation(), ego_data, mounting_pose);
    gt_object.base_sensor_coord.mutable_orientation()->CopyFrom(orientation_sensor_coord);

    return gt_object;
}

GroundTruthObject FrontEndStrategy::get_moving_object_from_ground_truth(const osi3::MovingObject& input_object,
                                                                        const TF::EgoData& ego_data,
                                                                        const osi3::MountingPosition& mounting_pose)
{

    GroundTruthObject gt_object;
    // ego coordinates
    gt_object.osi_gt_object = input_object;
    gt_object.base_ego_coord.mutable_position()->CopyFrom(
        TF::transform_to_local_coordinates(input_object.base().position(), ego_data.ego_base.orientation(), ego_data.ego_base.position()));
    gt_object.base_ego_coord.mutable_orientation()->CopyFrom(TF::calc_relative_orientation_to_local(input_object.base().orientation(), ego_data.ego_base.orientation()));

    // sensor coordinates
    osi3::Vector3d position_sensor_coord = TF::transform_position_from_world_to_sensor_coordinates(input_object.base().position(), ego_data, mounting_pose);
    gt_object.base_sensor_coord.mutable_position()->CopyFrom(position_sensor_coord);
    gt_object.position_spherical_sensor_coord = TF::transform_cartesian_to_spherical(position_sensor_coord);

    osi3::Orientation3d orientation_sensor_coord = TF::transform_orientation_from_world_to_sensor_coordinates(input_object.base().orientation(), ego_data, mounting_pose);
    gt_object.base_sensor_coord.mutable_orientation()->CopyFrom(orientation_sensor_coord);

    return gt_object;
}

void FrontEndStrategy::apply_noise_to_visible_vertices(std::vector<GroundTruthObject>& ground_truth_object_list, const Profile& profile)
{
    for (auto& current_object : ground_truth_object_list)
    {
        for (auto& current_vertex : current_object.visible_bounding_box_vertices_sensor_coord)
        {
            double angle_noise = 0.0;
            double distance_noise = 0.0;

            if (profile.sensor_parameters.vertex_angle_stddev > 0.0)
            {
                std::normal_distribution<double> distribution_angle(0.0, profile.sensor_parameters.vertex_angle_stddev);  // dist(mean, stddev)
                std::knuth_b generator_angle(std::rand());                                                                // rand used for Windows compatibility
                angle_noise = distribution_angle(generator_angle);
            }
            if (profile.sensor_parameters.vertex_distance_stddev > 0.0)
            {
                std::normal_distribution<double> distribution_distance(0.0, profile.sensor_parameters.vertex_distance_stddev);  // dist(mean, stddev)
                std::knuth_b generator_dist(std::rand());                                                                       // rand used for Windows compatibility
                distance_noise = distribution_distance(generator_dist);
            }
            osi3::Spherical3d current_vertex_with_noise;
            current_vertex.set_azimuth(current_vertex.azimuth() + angle_noise);
            current_vertex.set_elevation(current_vertex.elevation() + angle_noise);
            current_vertex.set_distance(current_vertex.distance() + distance_noise);
        }
    }
}

void FrontEndStrategy::write_vertices_to_logical_detections(osi3::SensorData& sensor_data, std::vector<GroundTruthObject>& ground_truth_object_list, const TF::EgoData& ego_data)
{
    for (auto& current_object : ground_truth_object_list)
    {
        for (auto& current_vertex : current_object.bounding_box_vertices)
        {
            osi3::Vector3d current_vertex_word_coord =
                TF::transform_from_local_coordinates(current_vertex, current_object.osi_gt_object.base().orientation(), current_object.osi_gt_object.base().position());
            osi3::Vector3d current_vertex_ego_coord = TF::transform_position_from_world_to_ego_coordinates(current_vertex_word_coord, ego_data);
            auto* current_logical_detection = sensor_data.mutable_logical_detection_data()->add_logical_detection();
            current_logical_detection->mutable_position()->CopyFrom(current_vertex_ego_coord);
        }
    }
}

void FrontEndStrategy::write_visible_vertices_to_detections(osi3::SensorData& sensor_data, std::vector<GroundTruthObject>& ground_truth_object_list)
{
    auto* current_lidar = sensor_data.mutable_feature_data()->add_lidar_sensor();
    MountingPosition mounting_pose;
    if (!profile.sensor_view_configuration.radar_sensor_view_configuration().empty())
    {  // radar
        mounting_pose.CopyFrom(profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position());
    }
    else if (!profile.sensor_view_configuration.lidar_sensor_view_configuration().empty())
    {  // lidar
        mounting_pose.CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position());
    }
    current_lidar->mutable_header()->mutable_mounting_position()->mutable_position()->set_x(mounting_pose.position().x());
    current_lidar->mutable_header()->mutable_mounting_position()->mutable_position()->set_y(mounting_pose.position().y());
    current_lidar->mutable_header()->mutable_mounting_position()->mutable_position()->set_z(mounting_pose.position().z());
    current_lidar->mutable_header()->mutable_mounting_position()->mutable_orientation()->set_yaw(mounting_pose.orientation().yaw());
    current_lidar->mutable_header()->mutable_mounting_position()->mutable_orientation()->set_pitch(mounting_pose.orientation().pitch());
    current_lidar->mutable_header()->mutable_mounting_position()->mutable_orientation()->set_roll(mounting_pose.orientation().roll());
    for (auto& current_object : ground_truth_object_list)
    {
        for (auto& current_vertex : current_object.visible_bounding_box_vertices_sensor_coord)
        {
            auto* current_detection = current_lidar->add_detection();
            current_detection->mutable_position()->CopyFrom(current_vertex);
            current_detection->mutable_object_id()->set_value(current_object.osi_gt_object.id().value());
        }
    }
}
