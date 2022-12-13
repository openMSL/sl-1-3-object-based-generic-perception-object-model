//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef OCCLUSION_H
#define OCCLUSION_H

#include "string"
#include "frontendstrategy/clipper.hpp"

struct Vertex {
    osi3::Vector2d vertex_on_plane;
    osi3::Spherical3d vertex3d;
};

typedef double T;
typedef std::array<T, 2> point_type;

void get_visible_vertices(std::vector<GroundTruthObject> &ground_truth_object_list, const Profile &profile, const Log &log, const Alert &alert);

void graham_scan(std::vector<Vertex> &vertices_on_hull, const std::vector<Vertex> &all_vertices);
void concave_hull_of_vertices_projection(std::vector<Vertex> &vertices_on_hull, GroundTruthObject &gt_object, const std::vector<osi3::Spherical3d> &all_vertices_spherical);
void calculate_occlusion(std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, std::vector<Vertex> &visible_vertices, std::vector<Vertex> &vertices_on_hull, const double &ground_clearance,
                         const Profile &profile, const Log &log);
std::vector<Vertex> project_object_vertices_to_cylinder(const std::vector<osi3::Spherical3d> &all_vertices_spherical);
ClipperLib::Paths clip_polygons(std::vector<ClipperLib::IntPoint> &subject, const ClipperLib::Paths &clipper, ClipperLib::ClipType clip_type);
//void write_vertices_to_csv(std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, const std::string &file_name, const Log &log);
ClipperLib::Paths calculate_clipped_polygons(std::vector<Vertex> &vertices_on_hull, std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, float float_to_int_factor);
void
clip_fov(std::vector<Vertex> &visible_vertices_in_fov, const std::vector<Vertex> &visible_vertices, std::vector<Vertex> &vertices_on_hull, const Profile &profile, const GroundTruthObject &gt_object,
         const Log &log, const Alert &alert);
std::vector<Spherical3d> create_fov_vertices(const Profile &profile, const Alert &alert);
void append_polygons_in_occlusion_process(std::vector<Vertex> &visible_vertices, std::vector<std::vector<Vertex>> &polygons_in_occlusion_process, const ClipperLib::Paths &clipped_polygons,
                                          std::vector<Vertex> &vertices_on_hull, float float_to_int_factor, const bool &consider_in_occlusion_process);
void reconstruct_3d_distance_from_2d_points(Vertex &current_vertex, std::vector<Vertex> &vertices_on_hull, const GroundTruthObject &current_gt_object);
void calc_new_vertex_on_edge(Vertex& current_vertex, const std::vector<Vertex> &vertices_on_hull);

point_type get_normalized_vector(point_type origin, point_type point);

#endif //OCCLUSION_H
