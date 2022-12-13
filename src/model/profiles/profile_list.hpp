//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef OSMPSENSORFRAMEWORK_PROFILE_LIST_HPP
#define OSMPSENSORFRAMEWORK_PROFILE_LIST_HPP

#include <model/profiles/profile.hpp>
#include <string>

/* TODO add further profiles here */
#include <model/profiles/profile_SCALA_1.hpp>
#include <model/profiles/profile_Ibeo_LUX_2010.hpp>
#include <model/profiles/profile_LongRange_Radar.hpp>
#include <model/profiles/profile_MidRange_Radar.hpp>

bool CFrameworkPackaging::try_load_profile(const std::string &name) {
    /* TODO add further profile generators here */
    if (name == "SCALA_1") {
        profile = model::profile::SCALA_1::generate();
        return true;
    }
    if (name == "Ibeo_LUX_2010") {
        profile = model::profile::Ibeo_LUX_2010::generate();
        return true;
    }
    if (name == "LongRange_Radar") {
        profile = model::profile::LongRange_Radar::generate();
        return true;
    }
    if (name == "MidRange_Radar") {
        profile = model::profile::MidRange_Radar::generate();
        return true;
    }

    return false;
}

#endif //OSMPSENSORFRAMEWORK_PROFILE_LIST_HPP
