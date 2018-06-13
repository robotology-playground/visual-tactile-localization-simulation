/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef HEAD_KINEMATICS_H
#define HEAD_KINEMATICS_H

// yarp
// #include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
// #include <yarp/sig/Matrix.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>

// std
// #include <string>
// #include <vector>
// #include <unordered_map>

class headKinematics
{
protected:
    // driver and view for encoders
    yarp::dev::PolyDriver drv_torso;    
    yarp::dev::PolyDriver drv_head;
    yarp::dev::IEncoders *ienc_torso;
    yarp::dev::IEncoders *ienc_head;

    // forward kinematics of the eyes
    iCub::iKin::iCubEye left_eye_kin;
    iCub::iKin::iCubEye right_eye_kin;

    bool getEncoders(yarp::sig::Vector &torso_encs,
		     yarp::sig::Vector &head_encs);
    void getEyesDofs(const yarp::sig::Vector &torso_encs,
		     const yarp::sig::Vector &head_encs,
		     yarp::sig::Vector &left_eye_dofs,
		     yarp::sig::Vector &right_eye_dofs);

public:
    bool configure(const std::string &robot_name,
		   const std::string &port_suffix = "");
    bool getEyesPose(yarp::sig::Vector &left_eye_pose,
		     yarp::sig::Vector &right_eye_pose);
    void close();
};

#endif
