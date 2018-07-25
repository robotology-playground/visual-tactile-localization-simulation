/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file handKinematics.h
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

#ifndef HAND_KINEMATICS_H
#define HAND_KINEMATICS_H

// yarp
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/sig/Matrix.h>

// std
#include <string>
#include <vector>
#include <unordered_map>

//
#include <iKinFwdExtended.h>

typedef std::unordered_map<std::string, std::vector<yarp::sig::Matrix> > fingersLinks;

class handKinematics
{
protected:
    // driver and view for encoders
    yarp::dev::PolyDriver drv_arm;
    yarp::dev::PolyDriver drv_torso;
    yarp::dev::PolyDriver drv_analog;
    yarp::dev::IEncoders *ienc_arm;
    yarp::dev::IEncoders *ienc_torso;
    yarp::dev::IAnalogSensor *ianalog;
    yarp::dev::IControlLimits *ilim;

    // hand name
    std::string hand_name;

    // forward kinematics of arm
    iCub::iKin::iCubArm arm_kin;

    // forward kinematics of fingers
    std::unordered_map<std::string, iCub::iKin::iCubFingerExt> fingers_kin;

    // fingers names
    std::vector<std::string> fingers_names;

    // matrix of analog bounds
    // for encoders of proximal/distal joints
    yarp::sig::Matrix analog_bounds;

    bool getEncoders(yarp::sig::Vector &arm_encs,
		     yarp::sig::Vector &torso_encs,
		     yarp::sig::Vector &fingers_analogs);
    void getArmDofs(const yarp::sig::Vector &arm_encs,
		    const yarp::sig::Vector &torso_encs,
		    yarp::sig::Vector &arm_dofs);

    void setupAnalogBounds();

    // whether to use analog encoders of proximal/distal joints
    bool use_analog;
    // whether to use analog custom bounds
    bool use_analog_bounds;

public:
    bool configure(const std::string &robot_name,
	           const std::string &hand_name,
		   const bool &use_analog = false,
                   const bool &use_analog_bounds = false,
		   const std::string &port_prefix = "");
    bool getFingersLinks(fingersLinks &links);
    void close();
};

#endif
