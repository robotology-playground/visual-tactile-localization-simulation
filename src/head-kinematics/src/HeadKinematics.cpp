/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

//
#include <cmath>

#include <HeadKinematics.h>

using namespace yarp::math;

bool headKinematics::configure(const std::string &robot_name,
			       const std::string &port_prefix)
{
    // encoders configuration
    yarp::os::Property prop_encoders;
    prop_encoders.put("device", "remote_controlboard");
    prop_encoders.put("remote", "/" + robot_name + "/torso");
    prop_encoders.put("local", port_prefix + "/head_kinematics/torso");

    // try to open driver
    bool ok_drv = drv_torso.open(prop_encoders);
    if (!ok_drv)
    {
	yError() << "headKinematics::configure error:"
		 << "unable to open the Remote Control Board driver for the torso";
	return false;
    }
    
    prop_encoders.put("remote", "/" + robot_name + "/head");
    prop_encoders.put("local", port_prefix + "/head_kinematics");
    ok_drv = drv_head.open(prop_encoders);
    if (!ok_drv)
    {
    	yError() << "headKinematics::configure error:"
    		 << "unable to open the Remote Control Board driver for the head";
    	return false;
    }

    // try to retrieve the views
    bool ok_view = drv_torso.view(ienc_torso);
    if (!ok_view || ienc_torso == 0)
    {
    	yError() << "headKinematics::configure error:"
    		 << "unable to retrieve the Encoders view for the torso";
    	return false;
    }

    ok_view = drv_head.view(ienc_head);
    if (!ok_view || ienc_head == 0)
    {
    	yError() << "headKinematics::configure error:"
    		 << "unable to retrieve the Encoders view for the head";
    	return false;
    }

    // instantiate forward kinematics for eyes
    left_eye_kin = iCub::iKin::iCubEye("left");
    right_eye_kin = iCub::iKin::iCubEye("right");

    // limits update is not required to evaluate the forward kinematics
    // using angles from the encoders
    left_eye_kin.setAllConstraints(false);
    right_eye_kin.setAllConstraints(false);    
    // torso can be moved in general so its links have to be released
    left_eye_kin.releaseLink(0);
    left_eye_kin.releaseLink(1);
    left_eye_kin.releaseLink(2);
    right_eye_kin.releaseLink(0);
    right_eye_kin.releaseLink(1);
    right_eye_kin.releaseLink(2);
    
    return true;
}

bool headKinematics::getEncoders(yarp::sig::Vector &torso_encs,
				 yarp::sig::Vector &head_encs)
{
    // resize vectors
    torso_encs.resize(3);
    head_encs.resize(6);

    bool ok = ienc_torso->getEncoders(torso_encs.data());
    if(!ok)
    	return false;
    ok = ienc_head->getEncoders(head_encs.data());
    if(!ok)
    	return false;

    return true;
}

void headKinematics::getEyesDofs(const yarp::sig::Vector &torso_encs,
				 const yarp::sig::Vector &head_encs,
				 yarp::sig::Vector &left_eye_dofs,
				 yarp::sig::Vector &right_eye_dofs)
{
    // these are the dofs as expected by the class
    // iCub::iKin::iCubEye
    left_eye_dofs.resize(left_eye_kin.getDOF());
    right_eye_dofs.resize(right_eye_kin.getDOF());

    // torso in reversed order
    left_eye_dofs[0] = torso_encs[2];
    left_eye_dofs[1] = torso_encs[1];
    left_eye_dofs[2] = torso_encs[0];

    // neck
    left_eye_dofs.setSubvector(3, head_encs.subVector(0, 2));

    // eyes tilt
    left_eye_dofs[6] = head_encs[3];

    // copy to the right eye dofs
    right_eye_dofs = left_eye_dofs;

    // eye angles from vergence and version
    double vs = head_encs[4];
    double vg = head_encs[5];
    double left_angle = vs + vg / 2.0;
    double right_angle = vs - vg / 2.0;
    left_eye_dofs[7] = left_angle;
    right_eye_dofs[7] = right_angle;
}

bool headKinematics::getEyesPose(yarp::sig::Vector &left_eye_pose,
				 yarp::sig::Vector &right_eye_pose)
{
    bool ok;
    
    // get current value of encoders
    yarp::sig::Vector torso_encs;
    yarp::sig::Vector head_encs;
    ok = getEncoders(torso_encs, head_encs);
    if (!ok)
    	return false;

    // update the eyes chains
    yarp::sig::Vector left_eye_dofs;
    yarp::sig::Vector right_eye_dofs;
    getEyesDofs(torso_encs, head_encs,
    		left_eye_dofs, right_eye_dofs);
    left_eye_kin.setAng((M_PI/180.0) * left_eye_dofs);
    right_eye_kin.setAng((M_PI/180.0) * right_eye_dofs);    

    // get the current eyes poses
    left_eye_pose = left_eye_kin.EndEffPose();
    right_eye_pose = right_eye_kin.EndEffPose();

    return true;
}

void headKinematics::close()
{
    // close remote control board driver
    drv_torso.close();
    drv_head.close();
}
