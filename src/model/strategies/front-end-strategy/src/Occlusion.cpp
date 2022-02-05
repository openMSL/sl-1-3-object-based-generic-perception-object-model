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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

//#include <fstream> // DEBUG
#include <numeric>
#include "frontendstrategy/FrontEndStrategy.hpp"
#include "frontendstrategy/Occlusion.h"
#include "frontendstrategy/concaveman.h"
#include "frontendstrategy/clipper.hpp"

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

//bool written_to_csv = false; // DEBUG

void get_visible_vertices(std::vector<GroundTruthObject> &ground_truth_object_list, const Profile &profile, const Log &log, const Alert &alert) {
    //sort gt objects by distance
    GroundTruthObject gt_object_sort;
    std::sort(ground_truth_object_list.begin(), ground_truth_object_list.end(), gt_object_sort);
    //iterate over sorted objects
    std::vector<std::vector<Vertex>> polygons_in_occlusion_process;
    std::vector<std::vector<Vertex>> csv_output; //DEBUG
    for(auto& current_gt_object : ground_truth_object_list) {
        std::vector<osi3::Spherical3d> all_vertices_current_object_spherical;
        for(auto& current_vertex : current_gt_object.bounding_box_vertices_sensor_coord) {
            Spherical3d all_vertices = TF::transform_cartesian_to_spherical(current_vertex);
            all_vertices.set_elevation(round(all_vertices.elevation() * 1000000) / 1000000);
            all_vertices.set_azimuth(round(all_vertices.azimuth() * 1000000) / 1000000);
            all_vertices.set_distance(round(all_vertices.distance() * 1000000) / 1000000);
            all_vertices_current_object_spherical.emplace_back(all_vertices);
        }
        std::vector<Vertex> vertices_on_hull;
        concave_hull_of_vertices_projection(vertices_on_hull, current_gt_object, all_vertices_current_object_spherical);

        std::vector<Vertex> visible_vertices;
        double ground_clearance;
        if(current_gt_object.osi_gt_object.has_vehicle_attributes() && current_gt_object.osi_gt_object.vehicle_attributes().has_ground_clearance()) {
            ground_clearance = current_gt_object.osi_gt_object.vehicle_attributes().ground_clearance();
        } else {
            ground_clearance = 0;
            log("!! Field 'ground_clearance' missing. Occlusion potentially falsely simulated. !!");
        }
        calculate_occlusion(polygons_in_occlusion_process, visible_vertices, vertices_on_hull, ground_clearance, profile, log);

        std::vector<Vertex> visible_vertices_in_fov;
        clip_fov(visible_vertices_in_fov, visible_vertices, vertices_on_hull, profile, current_gt_object, log, alert);

        for (auto &vertex : visible_vertices_in_fov) {
            if(profile.sensor_view_configuration.radar_sensor_view_configuration().empty() || vertex.vertex3d.distance() < profile.data_extraction_parameters.max_range_in_m)   //hard range cut for radar
            current_gt_object.visible_bounding_box_vertices_sensor_coord.push_back(vertex.vertex3d);
        }
        //csv_output.emplace_back(visible_vertices_in_fov); //DEBUG
    }
    //write_vertices_to_csv(csv_output, "/tmp/solution.csv", log); //DEBUG
    //written_to_csv = true; //DEBUG
}


void concave_hull_of_vertices_projection(std::vector<Vertex> &vertices_on_hull, GroundTruthObject &gt_object, const std::vector<osi3::Spherical3d> &all_vertices_spherical) {

    std::vector<Vertex> all_vertices_on_cylinder = project_object_vertices_to_cylinder(all_vertices_spherical);
    graham_scan(vertices_on_hull, all_vertices_on_cylinder);

    //TODO: concave convex_hull in separate function

    double test_distance_factor = gt_object.position_spherical_sensor_coord.distance();

    std::vector<point_type> points;
    std::vector<int> convex_hull ;
    for(int vertex_idx = 0; vertex_idx < all_vertices_on_cylinder.size(); vertex_idx++) {
        Vertex current_vertex = all_vertices_on_cylinder.at(vertex_idx);
        point_type point{current_vertex.vertex_on_plane.x()*test_distance_factor, current_vertex.vertex_on_plane.y()*test_distance_factor};
        points.emplace_back(point);
        for (auto &current_vertex_on_convex_hull : vertices_on_hull) {
            if (std::abs(current_vertex.vertex_on_plane.x() - current_vertex_on_convex_hull.vertex_on_plane.x()) < 0.00001 &&
                    std::abs(current_vertex.vertex_on_plane.y() - current_vertex_on_convex_hull.vertex_on_plane.y()) < 0.00001) {
                convex_hull.emplace_back(vertex_idx);
            }
        }
    }

    auto concave = concaveman<T, 16>(points, convex_hull, 1, 1);//0.5/gt_object.position_spherical_sensor_coord.distance());

    ////sort concave by angle
    point_type center_of_shape{0,0};
    for (auto& current_point : concave) {
        center_of_shape[0] = center_of_shape[0]+current_point[0];
        center_of_shape[1] = center_of_shape[1]+current_point[1];
    }
    point_type point_min_left{center_of_shape[0]/concave.size(),center_of_shape[1]/concave.size()};
    point_type point_min_right{center_of_shape[0]/concave.size(),center_of_shape[1]/concave.size()};
    for (auto& current_point : concave) {
        if(current_point[0] < point_min_left[0] && current_point[1] < point_min_left[1]) {
            point_min_left = current_point;
        } else if(current_point[0] > point_min_right[0] && current_point[1] < point_min_right[1]) {
            point_min_right = current_point;
        }
    }
    point_type point_min_center;
    for(int axis_idx = 0; axis_idx < 2; axis_idx++) {
        point_min_center[axis_idx] = point_min_left[axis_idx] + (point_min_right[axis_idx]-point_min_left[axis_idx])/2;
    }
    point_type base_axis = get_normalized_vector(point_min_center, point_min_right);

    //calc angle
    std::vector<double> angles;
    for (auto& point : concave) {
        point_type vector_direction = get_normalized_vector(point_min_center, point);
        double vector_product = round((base_axis[0]*vector_direction[0]+base_axis[1]*vector_direction[1])*1000000)/1000000;
        double angle = acos(vector_product);
        angles.emplace_back(angle);
    }
    //sort by angle
    std::vector<int> V;
    for(int idx = 0; idx < concave.size(); idx++) {
        V.emplace_back(idx);
    }
    int x=0;
    std::iota(V.begin(),V.end(),x++); //Initializing
    sort( V.begin(),V.end(), [&](int i,int j){return angles[i]<angles[j];} );
    std::vector<point_type> sorted_concave_hull;
    for(auto& index : V) {
        sorted_concave_hull.emplace_back(concave.at(index));
    }
   //write sorted concave hull to output
    vertices_on_hull.clear();
    for (auto& point : sorted_concave_hull) {
        for(auto& current_vertex_on_convex_hull : all_vertices_on_cylinder) {
            if(std::abs(point[0]/test_distance_factor - current_vertex_on_convex_hull.vertex_on_plane.x()) < 0.000001 &&
                    std::abs(point[1]/test_distance_factor - current_vertex_on_convex_hull.vertex_on_plane.y()) < 0.000001) {
                vertices_on_hull.emplace_back(current_vertex_on_convex_hull);
                break;
            }
        }
    }
}

point_type get_normalized_vector(point_type origin, point_type point) {
    point_type vector_direction{point[0]-origin[0],point[1]-origin[1]};
    double vector_length = std::sqrt(std::pow(vector_direction[0],2)+std::pow(vector_direction[1],2));
    vector_direction[0] = vector_direction[0]/vector_length;
    vector_direction[1] = vector_direction[1]/vector_length;
    return vector_direction;
}

std::vector<Vertex> project_object_vertices_to_cylinder(const std::vector<osi3::Spherical3d> &all_vertices_spherical) {
    std::vector<Vertex> all_vertices_on_cylinder;
    for (auto &current_vertex_3d : all_vertices_spherical) {
        Vertex vertex_struct;
        vertex_struct.vertex3d.CopyFrom(current_vertex_3d);
        TF::projection_onto_unit_distance_cylinder(current_vertex_3d, vertex_struct.vertex_on_plane);
        all_vertices_on_cylinder.push_back(vertex_struct);
    }
    return all_vertices_on_cylinder;
}

void calculate_occlusion(std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, std::vector<Vertex> &visible_vertices, std::vector<Vertex> &vertices_on_hull, const double &ground_clearance,
                         const Profile &profile, const Log &log) {
    using namespace ClipperLib;
    float float_to_int_factor = 1000000000000.0;
    bool consider_in_occlusion_process = false;
    if(profile.sensor_view_configuration.radar_sensor_view_configuration().empty() // lidar
    || (!profile.sensor_view_configuration.radar_sensor_view_configuration().empty() && ground_clearance < profile.sensor_parameters.radar_multipath_min_ground_clearance)) { // radar, but ground_clearance to low
        consider_in_occlusion_process = true;
    }

    if(polygons_in_occlusion_process.empty()) {
        if(consider_in_occlusion_process) {
            polygons_in_occlusion_process.push_back(vertices_on_hull);
        }
        visible_vertices = vertices_on_hull;
    } else {
        ClipperLib::Paths clipped_polygons = calculate_clipped_polygons(vertices_on_hull, polygons_in_occlusion_process, float_to_int_factor);
        append_polygons_in_occlusion_process(visible_vertices, polygons_in_occlusion_process, clipped_polygons, vertices_on_hull, float_to_int_factor, consider_in_occlusion_process);
    }
}


ClipperLib::Paths calculate_clipped_polygons(std::vector<Vertex> &vertices_on_hull, std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, float float_to_int_factor) {
    using namespace ClipperLib;
    Paths previous_solutions(1);
    for(auto& current_vertex : vertices_on_hull) {
        IntPoint current_point(current_vertex.vertex_on_plane.x()*float_to_int_factor,current_vertex.vertex_on_plane.y()*float_to_int_factor);
        previous_solutions[0] << current_point;
    }
    for(auto& current_polygon : polygons_in_occlusion_process) {
        Paths clipper(1), current_solutions;
        for(auto& current_vertex : current_polygon) {
            IntPoint current_point(current_vertex.vertex_on_plane.x()*float_to_int_factor,current_vertex.vertex_on_plane.y()*float_to_int_factor);
            clipper[0] << current_point;
        }
        for(auto& current_path : previous_solutions) {
            Paths solution = clip_polygons(current_path, clipper, ctDifference);
            for(auto& current_solution : solution) {
                current_solutions.emplace_back(current_solution);
            }
        }
        previous_solutions = current_solutions;
    }
    return previous_solutions;
}


ClipperLib::Paths clip_polygons(std::vector<ClipperLib::IntPoint> &subject, const ClipperLib::Paths &clipper, const ClipperLib::ClipType clip_type) {
    using namespace ClipperLib;
    Paths subj(1), solution;
    subj[0] = subject;
    Clipper c;
    c.AddPaths(subj, ptSubject, true);
    c.AddPaths(clipper, ptClip, true);
    c.Execute(clip_type, solution, pftNonZero, pftNonZero);
    return solution;
}

void
clip_fov(std::vector<Vertex> &visible_vertices_in_fov, const std::vector<Vertex> &visible_vertices, std::vector<Vertex> &vertices_on_hull, const Profile &profile, const GroundTruthObject &gt_object,
         const Log &log, const Alert &alert) {
    using namespace ClipperLib;
    float float_to_int_factor = 1000000.0;
    Paths subject(1);
    for(auto& current_vertex : visible_vertices) {
        IntPoint current_point(current_vertex.vertex_on_plane.x()*float_to_int_factor,current_vertex.vertex_on_plane.y()*float_to_int_factor);
        subject[0] << current_point;
    }

    std::vector<Spherical3d> fov_vertices = create_fov_vertices(profile, alert);

    Paths clipper(1), solutions;
    for(auto& current_fov_vertex : fov_vertices) {
        Vertex current_vertex_plane;
        TF::projection_onto_unit_distance_cylinder(current_fov_vertex, current_vertex_plane.vertex_on_plane);
        IntPoint current_point(current_vertex_plane.vertex_on_plane.x()*float_to_int_factor,current_vertex_plane.vertex_on_plane.y()*float_to_int_factor);
        clipper[0] << current_point;
    }
    for(auto& current_path : subject) {
        Paths solution = clip_polygons(current_path, clipper, ctIntersection);
        for(auto& current_solution : solution) {
            solutions.emplace_back(current_solution);
        }
    }
    if(!solutions.empty()) {
        for(auto& current_point : solutions.at(0)) {
            Vertex current_vertex;
            current_vertex.vertex_on_plane.set_x(current_point.X / float_to_int_factor);
            current_vertex.vertex_on_plane.set_y(current_point.Y / float_to_int_factor);
            current_vertex.vertex3d.set_azimuth(current_vertex.vertex_on_plane.x());
            current_vertex.vertex3d.set_elevation(atan(current_vertex.vertex_on_plane.y()));
            reconstruct_3d_distance_from_2d_points(current_vertex, vertices_on_hull, gt_object);
            visible_vertices_in_fov.emplace_back(current_vertex);
        }
    }
}


std::vector<Spherical3d> create_fov_vertices(const Profile &profile, const Alert &alert) {
    std::vector<Spherical3d> fov_vertices;
    osi3::Spherical3d vertex_1;
    vertex_1.set_distance(1);
    osi3::Spherical3d vertex_2;
    vertex_2.set_distance(1);
    osi3::Spherical3d vertex_3;
    vertex_3.set_distance(1);
    osi3::Spherical3d vertex_4;
    vertex_4.set_distance(1);
    if(!profile.sensor_view_configuration.radar_sensor_view_configuration().empty()) { // radar
        vertex_1.set_elevation(- profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_1.set_azimuth(  - profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
        vertex_2.set_elevation(- profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_2.set_azimuth(    profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
        vertex_3.set_elevation(  profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_3.set_azimuth(    profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
        vertex_4.set_elevation(  profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_4.set_azimuth(  - profile.sensor_view_configuration.radar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
    } else if(!profile.sensor_view_configuration.lidar_sensor_view_configuration().empty()) { // lidar
        vertex_1.set_elevation(- profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_1.set_azimuth(  - profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
        vertex_2.set_elevation(- profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_2.set_azimuth(    profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
        vertex_3.set_elevation(  profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_3.set_azimuth(    profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
        vertex_4.set_elevation(  profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_vertical() / 2);
        vertex_4.set_azimuth(  - profile.sensor_view_configuration.lidar_sensor_view_configuration(0).field_of_view_horizontal() / 2);
    } else {
        alert("No lidar or radar sensor view in profile!");
    }

    fov_vertices.emplace_back(vertex_1);
    fov_vertices.emplace_back(vertex_4);
    fov_vertices.emplace_back(vertex_3);
    fov_vertices.emplace_back(vertex_2);

    return fov_vertices;
}


void append_polygons_in_occlusion_process(std::vector<Vertex> &visible_vertices, std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, const ClipperLib::Paths &clipped_polygons,
                                          std::vector<Vertex> &vertices_on_hull, float float_to_int_factor, const bool &consider_in_occlusion_process) {
    for(auto& current_polygon : clipped_polygons) {
        std::vector<Vertex> output_polygon;
        for(auto& current_point : current_polygon) {
            Vertex current_vertex;
            current_vertex.vertex_on_plane.set_x(current_point.X/float_to_int_factor);
            current_vertex.vertex_on_plane.set_y(current_point.Y/float_to_int_factor);

            current_vertex.vertex3d.set_elevation(atan(current_vertex.vertex_on_plane.y()));
            current_vertex.vertex3d.set_azimuth(current_vertex.vertex_on_plane.x());

            output_polygon.emplace_back(current_vertex);
            visible_vertices.emplace_back(current_vertex);
        }
        if(consider_in_occlusion_process) {
            polygons_in_occlusion_process.emplace_back(output_polygon);
        }
    }
}


void reconstruct_3d_distance_from_2d_points(Vertex &current_vertex, std::vector<Vertex> &vertices_on_hull, const GroundTruthObject &current_gt_object) {
    //current vertex is a non-occluded original bounding box vertex
    for(auto& current_vertex_of_hull : vertices_on_hull) {
        if(std::abs(current_vertex_of_hull.vertex3d.azimuth() - current_vertex.vertex3d.azimuth()) < 0.00001 &&
                std::abs(current_vertex_of_hull.vertex3d.elevation() - current_vertex.vertex3d.elevation()) < 0.00001) {
            current_vertex.vertex3d.set_distance(current_vertex_of_hull.vertex3d.distance());
        }
    }

    //current vertex is not an original bb vertex, but located on an original edge
    if(!current_vertex.vertex3d.has_distance()) {
        calc_new_vertex_on_edge(current_vertex, vertices_on_hull);
    }

    //current vertex is not original nor on an edge, but on the bb surface
    if(current_vertex.vertex3d.distance() == 0) {
        //TODO:calc new vertex on surface -> to separate function
        osi3::Vector3d origin;
        origin.set_x(0);
        origin.set_y(0);
        origin.set_z(0);
        osi3::Vector3d direction_cartesian;     //l
        current_vertex.vertex3d.set_distance(1);
        TF::transform_spherical_to_cartesian(current_vertex.vertex3d, direction_cartesian);
        current_vertex.vertex3d.set_distance(2000000);

        ///iterate over all surfaces of current object
        for(auto& current_surface : current_gt_object.bounding_box_surfaces) {
            ///get surface vertices
            std::vector<Vector3d> surface_vertices_sensor_coord;
            for(auto& vertex_idx : current_surface) {
                surface_vertices_sensor_coord.emplace_back(current_gt_object.bounding_box_vertices_sensor_coord.at(vertex_idx));
            }

            ///define plane from surface -> normal N = AB x AC
            Vector3d surface_point_A = surface_vertices_sensor_coord.at(0);     //p_0
            Vector3d vector_AB = TF::vector_translation(surface_vertices_sensor_coord.at(1), surface_point_A, -1);
            Vector3d vector_AC = TF::vector_translation(surface_vertices_sensor_coord.at(2), surface_point_A, -1);
            Vector3d plane_normal = TF::cross_product(vector_AB, vector_AC);   //N

            ///calculate intersection point of ray and surface -> distance d = ((p0 - l0) dot N)/(l dot N)
            Vector3d intersection_point;
            double l_dot_N = TF::dot_product(direction_cartesian, plane_normal);
            if(l_dot_N != 0) {
                double distance = TF::dot_product(surface_point_A, plane_normal) / l_dot_N;
                intersection_point.set_x(direction_cartesian.x()*distance);
                intersection_point.set_y(direction_cartesian.y()*distance);
                intersection_point.set_z(direction_cartesian.z()*distance);

                ///check if intersection is inside BB (work around)
                //TODO: check if intersection point is inside surface boundaries
                Vector3d intersection_point_object_coord =  TF::transform_to_local_coordinates(intersection_point, current_gt_object.base_sensor_coord.orientation(), current_gt_object.base_sensor_coord.position());
                double bounding_box_margin = 0.01;
                double object_half_width = current_gt_object.osi_gt_object.base().dimension().width()/2+bounding_box_margin;
                double object_half_length = current_gt_object.osi_gt_object.base().dimension().length()/2+bounding_box_margin;
                double object_half_height = current_gt_object.osi_gt_object.base().dimension().height()/2+bounding_box_margin;
                if(intersection_point_object_coord.x() >= - object_half_length && intersection_point_object_coord.x() <= object_half_length &&
                        intersection_point_object_coord.y() >= - object_half_width && intersection_point_object_coord.y() <= object_half_width &&
                        intersection_point_object_coord.z() >= - object_half_height && intersection_point_object_coord.z() <= object_half_height) {
                    ///check if shortest distance
                    if(distance > 0 && distance < current_vertex.vertex3d.distance()) {
                        current_vertex.vertex3d.set_distance(distance);
                    }
                }
            }
        }
    }
}


double calc_d_mnop(const osi3::Vector3d& m, const osi3::Vector3d& n, const osi3::Vector3d& o, const osi3::Vector3d& p) {
    double d_mnop = (m.x()-n.x())*(o.x()-p.x()) + (m.y()-n.y())*(o.y()-p.y()) + (m.z()-n.z())*(o.z()-p.z());
    return d_mnop;
}


void calc_new_vertex_on_edge(Vertex& current_vertex, const std::vector<Vertex> &vertices_on_hull) {
    //mathematical foundation: http://paulbourke.net/geometry/pointlineplane/
    //line equation: P_a = P_1 + mu_a(P2-P1)
    osi3::Vector3d origin;
    origin.set_x(0);
    origin.set_y(0);
    origin.set_z(0);
    osi3::Vector3d direction_cartesian;
    current_vertex.vertex3d.set_distance(1);
    TF::transform_spherical_to_cartesian(current_vertex.vertex3d, direction_cartesian);
    current_vertex.vertex3d.set_distance(0);

    //calculate distance between lines
    double min_distance = 0.01;
    for(auto& current_vertex_of_hull_1 : vertices_on_hull) {
        for(auto& current_vertex_of_hull_2 : vertices_on_hull) {
            osi3::Vector3d vertex_1_cartesian;
            TF::transform_spherical_to_cartesian(current_vertex_of_hull_1.vertex3d, vertex_1_cartesian);
            osi3::Vector3d vertex_2_cartesian;
            TF::transform_spherical_to_cartesian(current_vertex_of_hull_2.vertex3d, vertex_2_cartesian);

            //calculate vector product of distance_between_lines vectors
            double d_1343 = calc_d_mnop(origin, vertex_1_cartesian, vertex_2_cartesian, vertex_1_cartesian);
            double d_4321 = calc_d_mnop(vertex_2_cartesian, vertex_1_cartesian, direction_cartesian, origin);
            double d_1321 = calc_d_mnop(origin, vertex_1_cartesian, direction_cartesian, origin);
            double d_4343 = calc_d_mnop(vertex_2_cartesian, vertex_1_cartesian, vertex_2_cartesian, vertex_1_cartesian);
            double d_2121 = calc_d_mnop(direction_cartesian, origin, direction_cartesian, origin);

            //distances
            double mu_a = (d_1343 * d_4321 - d_1321 * d_4343 ) / ( d_2121 * d_4343 - d_4321 * d_4321 );
            double mu_b = (d_1343 + mu_a * d_4321) / d_4343;

            //intersection points
            osi3::Vector3d vertex_direction = TF::vector_translation(vertex_2_cartesian, vertex_1_cartesian, -1);
            osi3::Vector3d intersection_point_1 = TF::vector_translation(origin, direction_cartesian, mu_a);
            osi3::Vector3d intersection_point_2 = TF::vector_translation(vertex_1_cartesian, vertex_direction, mu_b);

            osi3::Vector3d distance_vector = TF::vector_translation(intersection_point_2, intersection_point_1, -1);
            double distance_between_lines = std::sqrt(std::pow(distance_vector.x(), 2) + std::pow(distance_vector.x(), 2) + std::pow(distance_vector.x(), 2));

            osi3::Spherical3d intersection_point_2_spherical = TF::transform_cartesian_to_spherical(intersection_point_2);
            if(mu_b <= 1 && mu_b >= 0 && distance_between_lines < min_distance && intersection_point_2_spherical.distance() < current_vertex.vertex3d.distance()) {
                min_distance = distance_between_lines;
                current_vertex.vertex3d.set_distance(intersection_point_2_spherical.distance());
            }
        }
    }
}


inline double vertex_relative_to_line(const Vertex &edge_begin, const Vertex &edge_end, const Vertex &point) {
    return (edge_end.vertex_on_plane.x()- edge_begin.vertex_on_plane.x()) * (point.vertex_on_plane.y() - edge_end.vertex_on_plane.y()) - (edge_end.vertex_on_plane.y() - edge_begin.vertex_on_plane.y()) * (point.vertex_on_plane.x()- edge_end.vertex_on_plane.x());
    // 0: collinear
    // positive: left
    // negative: right
}


inline double distance_comparison(const Vertex &edge_begin, const Vertex &edge_end, const Vertex &point) {
    return (edge_end.vertex_on_plane.x()- edge_begin.vertex_on_plane.x()) * (edge_end.vertex_on_plane.x()- edge_begin.vertex_on_plane.x()) + (edge_end.vertex_on_plane.y() - edge_begin.vertex_on_plane.y()) * (edge_end.vertex_on_plane.y() - edge_begin.vertex_on_plane.y()) -
           (point.vertex_on_plane.x()- edge_begin.vertex_on_plane.x()) * (point.vertex_on_plane.x()- edge_begin.vertex_on_plane.x()) - (point.vertex_on_plane.y() - edge_begin.vertex_on_plane.y()) * (point.vertex_on_plane.y() - edge_begin.vertex_on_plane.y());
    // positive: point is nearer
    // negative: edge_end is nearer
}


void graham_scan(std::vector<Vertex> &vertices_on_hull, const std::vector<Vertex> &all_vertices) {
    std::vector<osi3::Vector3d>::size_type size_all_vertices = all_vertices.size();
    if (size_all_vertices < 3) return;
    double z_min = all_vertices.at(0).vertex_on_plane.y();
    auto start_itr = all_vertices.begin();
    for (auto vertex_itr = all_vertices.begin(); vertex_itr != all_vertices.end(); ++vertex_itr) { // element at 0 is copied into ptr_subject_head at initialization
        if (vertex_itr->vertex_on_plane.y() < z_min) {
            z_min = vertex_itr->vertex_on_plane.y();
            start_itr = vertex_itr;
        } else if ((vertex_itr->vertex_on_plane.y() == z_min) && (vertex_itr->vertex_on_plane.x() < vertex_itr->vertex_on_plane.x()))
            start_itr = vertex_itr;
    }
    std::vector<Vertex> temp_container;
    temp_container.push_back(*start_itr);
    for (const auto &vertex_processing : all_vertices) {
        if (std::abs(vertex_processing.vertex_on_plane.x() - start_itr->vertex_on_plane.x()) < 0.00001 && std::abs(vertex_processing.vertex_on_plane.y() - start_itr->vertex_on_plane.y()) < 0.00001) {
            continue;
        }
        if (temp_container.size() == 1) {
            temp_container.push_back(vertex_processing);
        } else {
            auto itr_processing = ++(temp_container.begin());
            do {
                double position_flag = vertex_relative_to_line(temp_container.front(), *itr_processing, vertex_processing);
                double distance_flag = distance_comparison(temp_container.front(), *itr_processing, vertex_processing);
                if (position_flag < 0 || (position_flag == 0 && distance_flag > 0)) {
                    temp_container.insert(itr_processing, vertex_processing);
                    break;
                } else {
                    ++itr_processing;
                    if (itr_processing == temp_container.end()) {
                        temp_container.push_back(vertex_processing);
                        break;
                    }
                }
            } while (true);
        }
    }
    for (auto &vertex_push_back : temp_container) {
        do {
            if (vertices_on_hull.size() == 1 || vertices_on_hull.empty()) {
                vertices_on_hull.push_back(vertex_push_back);
                break;
            }
            double position_flag = vertex_relative_to_line(vertices_on_hull.end()[-2], vertices_on_hull.end()[-1],
                                                           vertex_push_back);
            if (position_flag < 0) {
                vertices_on_hull.pop_back();
                continue;
            } else if (position_flag == 0) {
                double distance_flag = distance_comparison(vertices_on_hull.end()[-2], vertices_on_hull.end()[-1],
                                                           vertex_push_back);
                if (distance_flag > 0) {
                    vertices_on_hull.pop_back();
                    continue;
                }
                vertices_on_hull.push_back(vertex_push_back);
                break;
            } else {
                vertices_on_hull.push_back(vertex_push_back);
                break;
            }
        } while (true);
    }
}

/*void write_vertices_to_csv(std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, const std::string &file_name, const Log &log) {
    if(!written_to_csv){
        std::ofstream my_file(file_name);
        log("Write csv to " + file_name);
        for (auto& current_polygon : polygons_in_occlusion_process) {
            for(auto& current_vertex : current_polygon) {
                my_file << current_vertex.vertex_on_plane.x() << "," << current_vertex.vertex_on_plane.y() << std::endl;
            }
            my_file << 0 << "," << 0 << std::endl;
        }
    }
}*/
