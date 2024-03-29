//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef REFINED_BOUNDING_BOXES_HPP
#define REFINED_BOUNDING_BOXES_HPP

#include <model/include/strategy.hpp>

#include "../../transformation-functions/TransformationFunctions.hpp"
#include "frontendstrategy/FrontEndStrategy.hpp"
#include "osi_sensordata.pb.h"

using namespace model;
using namespace osi3;

void set_refined_bounding_boxes(std::vector<GroundTruthObject>& ground_truth_object_list, TF::EgoData& ego_data, const Profile& profile, const Alert& alert);
void create_refined_bounding_box(std::vector<std::vector<float>>& bounding_box_definition,
                                 GroundTruthObject& current_object,
                                 TF::EgoData& ego_data,
                                 const Profile& profile,
                                 const Alert& alert);
void get_refined_bounding_box_definition(std::vector<std::vector<float>>& bounding_box_definition, std::vector<std::vector<int>>& surfaces_definition, MovingObject::Type type);

#endif  // REFINED_BOUNDING_BOXES_HPP
