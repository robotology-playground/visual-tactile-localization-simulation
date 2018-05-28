/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include "FingerController.h"

using namespace yarp::math;

bool FingerController::init(const std::string &hand_name,
                            const std::string &finger_name,
                            yarp::dev::IControlMode2 *imod,
                            yarp::dev::IControlLimits2 *ilim,
                            yarp::dev::IPositionControl2 *ipos,
                            yarp::dev::IVelocityControl2 *ivel)
{
    bool ok;

    // reset current control mode
    control_mode = -1;

    // store pointer to ControlMode2 instance
    this->imod = imod;

    // store pointer to ControlLimits2 instance
    this->ilim = ilim;

    // store pointer to PositionControl2 instance
    this->ipos = ipos;

    // store pointer to VelocityControl2 instance
    this->ivel = ivel;

    // store name of the finger
    this->finger_name = finger_name;

    // store name of the hand
    this->hand_name = hand_name;

    // initialize the finger
    finger = iCub::iKin::iCubFingerExt(hand_name + "_" + finger_name);

    // set the controlled joints depending on the finger name
    if (finger_name == "thumb")
    {
        // up to now only thumb opposition is considered
        ctl_joints.push_back(8);
    }
    else if (finger_name == "index")
    {
        ctl_joints.push_back(11);
        ctl_joints.push_back(12);
    }
    else if (finger_name == "middle")
    {
        ctl_joints.push_back(13);
        ctl_joints.push_back(14);
    }
    else if (finger_name == "ring")
    {
        ctl_joints.push_back(15);
    }
    else
    {
        yError() << "FingerController:configure"
                 << "Error: finger"
                 << finger_name
                 << "is not valid or not supported";
        return false;
    }

    // get the current control modes for the controlled DoFs
    // initial_modes.resize(ctl_joints.size());
    // ok = imod->getControlModes(ctl_joints.size(),
    //                            ctl_joints.getFirst(),
    //                            initial_modes.getFirst());
    // if (!ok)
    // {
    //     yError() << "FingerController:configure"
    //              << "Error: unable to get the initial mode"
    //              << "for the joints of the"
    //              << hand_name << finger_name
    //              << "finger";

    //     return false;
    // }

    // set the velocity control mode for the controlled DoFs
    ok = setControlMode(VOCAB_CM_VELOCITY);
    if (!ok)
    {
        yError() << "FingerController:configure"
                 << "Error: unable to set the velocity control"
                 << "mode for the joints of the"
                 << hand_name << finger_name
                 << "finger";

        return false;
    }

    // compose jacobian coupling matrix
    coupling.resize(3, ctl_joints.size());
    coupling = 0;
    if (finger_name == "index" || finger_name == "middle")
    {
        coupling[0][0] = 1;   // proximal joint velocity = velocity of first DoF
        coupling[1][1] = 0.5; // first distal joint velocity = half velocity of second DoF
        coupling[2][1] = 0.5; // second distal joint velocity = half velocity of second DoF
    }
    else if (finger_name == "ring")
    {
        // within ring only one DoF is available
        coupling = 1.0 / 3.0;
    }
    else if (finger_name == "thumb")
    {
        // only thumb opposition is considered
        coupling.resize(1, 1);
        coupling = 1.0;
    }

    // extract the constant transformation between the hand
    // and the root frame of the finger once for all
    bool use_axis_angle = true;
    yarp::sig::Vector pose;
    pose = finger.Pose(0, use_axis_angle);
    finger_root_pos = pose.subVector(0,2);
    finger_root_att = yarp::math::axis2dcm(pose.subVector(3,6));
    finger_root_att = finger_root_att.submatrix(0, 2, 0, 2);

    // set default desired joints limits
    joints_des_limits.resize(ctl_joints.size());
    if (finger_name == "thumb")
        joints_des_limits[0] = 15.0;
    else if (finger_name == "index")
    {
        joints_des_limits[0] = 25.0;
        joints_des_limits[1] = 90.0;
    }
    else if (finger_name == "middle")
    {
        joints_des_limits[0] = 30.0;
        joints_des_limits[1] = 85.0;
    }
    else if (finger_name == "ring")
        joints_des_limits[0] = 85.0;

    // set default home joints position
    // and maximum limits
    joints_home.resize(ctl_joints.size());
    joints_max_limits.resize(ctl_joints.size());
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        double min;
        double max;
        ilim->getLimits(ctl_joints[i], &min, &max);
        joints_home[i] = min;
        joints_max_limits[i] = max;
    }

    // set defaults for proximal null based control
    prox_comfort_value = 0.0;
    prox_max_value = 0.0;
    prox_proj_gain = 0.0;

    return true;
}

bool FingerController::configure(const yarp::os::ResourceFinder &rf)
{
    double default_lim;
    if (finger_name == "thumb")
    {
        default_lim = 15.0;
        if (rf.find("thumbOpposeLimit").isNull())
            joints_des_limits[0] = default_lim;
        else
        {
            auto thumb_oppose_lim_v = rf.find("thumbOpposeLimit");
            if (thumb_oppose_lim_v.isDouble())
                joints_des_limits[0] = thumb_oppose_lim_v.asDouble();
            else
                joints_des_limits[0] = default_lim;
        }
    }
    else if (finger_name == "index")
    {
        default_lim = 25.0;
        if (rf.find("indexProximalLimit").isNull())
            joints_des_limits[0] = default_lim;
        else
        {
            auto index_prox_lim_v = rf.find("indexProximalLimit");
            if (index_prox_lim_v.isDouble())
                joints_des_limits[0] = index_prox_lim_v.asDouble();
            else
                joints_des_limits[0] = default_lim;
        }

        default_lim = 90.0;
        if (rf.find("indexDistalLimit").isNull())
            joints_des_limits[1] = default_lim;
        else
        {
            auto index_dist_lim_v = rf.find("indexDistalLimit");
            if (index_dist_lim_v.isDouble())
                joints_des_limits[1] = index_dist_lim_v.asDouble();
            else
                joints_des_limits[1] = default_lim;
        }
    }
    else if (finger_name == "middle")
    {
        default_lim = 30.0;
        if (rf.find("middleProximalLimit").isNull())
            joints_des_limits[0] = default_lim;
        else
        {
            auto middle_prox_lim_v = rf.find("middleProximalLimit");
            if (middle_prox_lim_v.isDouble())
                joints_des_limits[0] = middle_prox_lim_v.asDouble();
            else
                joints_des_limits[0] = default_lim;
        }

        default_lim = 85.0;
        if (rf.find("middleDistalLimit").isNull())
            joints_des_limits[1] = default_lim;
        else
        {
            auto middle_dist_lim_v = rf.find("middleDistalLimit");
            if (middle_dist_lim_v.isDouble())
                joints_des_limits[1] = middle_dist_lim_v.asDouble();
            else
                joints_des_limits[1] = default_lim;
        }
    }
    else if (finger_name == "ring")
    {
        default_lim = 85.0;
        if (rf.find("ringLittleLimit").isNull())
            joints_des_limits[0] = default_lim;
        else
        {
            auto ring_little_lim_v = rf.find("ringLittleLimit");
            if (ring_little_lim_v.isDouble())
                joints_des_limits[0] = ring_little_lim_v.asDouble();
            else
                joints_des_limits[0] = default_lim;
        }
    }

    if ((finger_name == "index") || (finger_name == "middle"))
    {
        if (rf.find("proxComfortValue").isNull())
            prox_comfort_value = 10.0;
        else
        {
            auto prox_comfort_value_v = rf.find("proxComfortValue");
            if (prox_comfort_value_v.isDouble())
                prox_comfort_value = prox_comfort_value_v.asDouble();
            else
                prox_comfort_value = 10.0;
        }

        if (rf.find("proxMaxValue").isNull())
            prox_max_value = 25.0;
        else
        {
            auto prox_max_value_v = rf.find("proxMaxValue");
            if (prox_max_value_v.isDouble())
                prox_max_value = prox_max_value_v.asDouble();
            else
                prox_max_value = 25.0;
        }

        if (rf.find("proxProjGain").isNull())
            prox_proj_gain = 10.0;
        else
        {
            auto prox_proj_gain_v = rf.find("proxProjGain");
            if (prox_proj_gain_v.isDouble())
                prox_proj_gain = prox_proj_gain_v.asDouble();
            else
                prox_proj_gain = 10.0;
        }

        yInfo() << "FingerController configuration for"
                << hand_name + "_" + finger_name;
        yInfo() << "Proximal Comfort:" << prox_comfort_value;
        yInfo() << "Proximal Max:" << prox_max_value;
        yInfo() << "Proximal Projector Gain" << prox_proj_gain;
    }
}

bool FingerController::setControlMode(const int &mode)
{
    bool ok;

    //get current control modes first
    // yarp::sig::VectorOf<int> modes(ctl_joints.size());
    // ok = imod->getControlModes(ctl_joints.size(),
    //                            ctl_joints.getFirst(),
    //                            modes.getFirst());
    // if (!ok)
    // {
    //     yError() << "FingerController:setControlMode"
    //              << "Error: unable to get current joints control modes for finger"
    //              << hand_name << finger_name;
    //     return false;
    // }

    // set only the control modes different from the desired one
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        // if (modes[i] != mode)
        // {
            ok = imod->setControlMode(ctl_joints[i], mode);
            if (!ok)
            {
                yError() << "FingerController:setControlMode"
                         << "Error: unable to set control mode for one of the joint of the finger"
                         << hand_name << finger_name;
                return false;
            }
        // }
    }

    // if all joints were set store the current control mode
    control_mode = mode;

    return true;
}

bool FingerController::close()
{
    bool ok;

    // stop motion
    ok = ivel->stop(ctl_joints.size(), ctl_joints.getFirst());
    if (!ok)
    {
        yError() << "FingerController::close"
                 << "WARNING: unable to stop joints motion for finger"
                 << hand_name << finger_name;
        return false;
    }

    // restore initial control mode
    //yarp::sig::VectorOf<int> pos_ctl_modes;
    //pos_ctl_modes.resize(ctl_joints.size(), VOCAB_CM_POSITION);
    //ok = imod->setControlModes(ctl_joints.size(),
                               //ctl_joints.getFirst(),
                               //pos_ctl_modes.getFirst());
    //if (!ok)
    //{
     //yError() << "FingerController:close"
              //<< "Error: unable to restore the initial control modes for finger"
              //<< hand_name << finger_name;
     //return false;
    //}

    setControlMode(VOCAB_CM_POSITION);

    return true;
}

void FingerController::setHomePosition(const yarp::sig::Vector &encoders)
{
    // extract values of controlled DoF
    for (size_t i=0; i<ctl_joints.size(); i++)
        joints_home[i] = encoders[ctl_joints[i]];
}

bool FingerController::updateFingerChain(const yarp::sig::Vector &encoders)
{
    bool ok;

    // store motor encoders that are required to enforce joints limits
    motors_encoders = encoders;

    // get subset of joints related to the finger
    ok = finger.getChainJoints(encoders, joints);
    if (!ok)
    {
        yError() << "FingerController::updateFingerChain"
                 << "Error: unable to retrieve the finger's joint values for finger"
                 << hand_name << finger_name;
        return false;
    }

    // convert to radians
    joints = joints * (M_PI/180.0);

    // update chain
    finger.setAng(joints);

    return true;
}

bool FingerController::updateFingerChain(const yarp::sig::Vector &motor_encs,
                                         const yarp::sig::Vector &analogs_encs)
{
    bool ok;

    // get subset of joints related to the finger
    ok = finger.getChainJoints(motor_encs, analogs_encs, joints);
    if (!ok)
    {
        yError() << "FingerController::updateFingerChain"
                 << "Error: unable to retrieve the finger's joint values for finger"
                 << hand_name << finger_name;
        return false;
    }

    // convert to radians
    joints = joints * (M_PI/180.0);

    // update chain
    finger.setAng(joints);
}

bool FingerController::getJacobianFingerFrame(yarp::sig::Matrix &jacobian)
{
    // jacobian for linear velocity part
    yarp::sig::Matrix j_lin;

    // jacobian for angular velocity part
    yarp::sig::Matrix j_ang;

    // get the jacobian
    jacobian = finger.GeoJacobian();

    // neglect abduction if index or ring
    if (finger_name == "index" || finger_name == "ring")
        jacobian.removeCols(0, 1);
    // retain only opposition if thumb
    else if (finger_name == "thumb")
        jacobian.removeCols(1, 3);

    // the number of columns should be one for the thumb
    bool valid_no_cols = true;
    if (finger_name == "thumb")
    {
        if (jacobian.cols() != 1)
            valid_no_cols = false;
    }
    // the number of columns should be three
    // for index, middle and ring
    else{
        if (jacobian.cols() != 3)
            valid_no_cols = false;
    }
    if (!valid_no_cols)
    {
        yError() << "FingerController::getJacobianFingerFrame"
                 << "Error: wrong number of columns"
                 << "for the jacobian of the finger"
                 << hand_name << finger_name;

        return false;
    }

    // extract linear velocity part
    if (finger_name == "thumb")
        j_lin = jacobian.submatrix(0, 2, 0, 0);
    else
        j_lin = jacobian.submatrix(0, 2, 0, 2);

    // express linear velocity in the root frame of the finger
    j_lin = finger_root_att.transposed() * j_lin;


    // the motion of the finger described w.r.t its root frame
    // is planar and velocities along the z axis (y axis for thumb opposition)
    // are zero hence the third row (second row) of the linear velocity jacobian can be dropped
    if (finger_name == "thumb")
        j_lin = j_lin.removeRows(1, 1);
    else
        j_lin = j_lin.removeRows(2, 1);

    // extract angular velocity part
    if (finger_name == "thumb")
        j_ang = jacobian.submatrix(3, 5, 0, 0);
    else
        j_ang = jacobian.submatrix(3, 5, 0, 2);

    // express angular velocity in root frame of the finger
    j_ang = finger_root_att.transposed() * j_ang;

    // the motion of the finger described w.r.t its root frame
    // is planar and angular velocities is all along the z axis
    // (-y axis for the thumb opposition)
    // hence first and second row (first and third) of the angular velocity jacobian
    // can be dropped
    if (finger_name == "thumb")
    {
        j_ang.removeRows(0, 1);
        // after the first removeRows
        // the third of the angular jacobian occupies the second row
        j_ang.removeRows(1, 1);
    }
    else
    {
        j_ang.removeRows(0, 2);
    }

    // compose the linear velocity and angular velocity parts together
    if (finger_name == "thumb")
        jacobian.resize(3, 1);
    else
        jacobian.resize(3, 3);
    jacobian.setSubmatrix(j_lin, 0, 0);
    jacobian.setSubmatrix(j_ang, 2, 0);

    // take into account coupling
    jacobian = jacobian * coupling;
}

// bool FingerController::getFingerTipPoseFingerFrame(yarp::sig::Vector &pose)
// {
//     // get the position of the finger tip
//     yarp::sig::Vector finger_tip = finger.EndEffPosition();

//     // evaluate the vector from the root frame of the finger
//     // to the finger tip
//     yarp::sig::Vector diff = finger_tip - finger_root_pos;

//     // express it in the root frame of the finger
//     diff = finger_root_att.transposed() * diff;

//     // evaluate the sum of the controlled joints
//     // representing the attitude of the planar chain
//     double att = 0;

//     // only opposition if thumb
//     if (finger_name == "thumb")
//         att = joints[0];
//     // neglect abduction if index or ring
//     else if (finger_name == "index" || finger_name == "ring")
//         att = joints[1] + joints[2] + joints[3];
//     // middle finger
//     else
//         att = joints[0] + joints[1] + joints[2];

//     pose.resize(3);
//     pose[0] = diff[0];
//     pose[1] = diff[1];
//     pose[2] = att;

//     return true;
// }

bool FingerController::goHome(const double &ref_vel)
{
    bool ok;

    // switch to position control
    ok = setControlMode(VOCAB_CM_POSITION);
    if (!ok)
    {
        yInfo() << "FingerController::goHome Error:"
                << "unable to set Position control mode for finger"
                << finger_name;

        return false;
    }

    // set reference joints velocities
    // the same velocity is used for all the joints
    yarp::sig::Vector speeds(ctl_joints.size(), ref_vel);
    ok = ipos->setRefSpeeds(ctl_joints.size(),
                            ctl_joints.getFirst(),
                            speeds.data());
    if (!ok)
    {
        yInfo() << "FingerController::goHome Error:"
                << "unable to set joints reference speeds for finger"
                << finger_name;

        return false;
    }

    // restore initial position of finger joints
    ok = ipos->positionMove(ctl_joints.size(),
                            ctl_joints.getFirst(),
                            joints_home.data());
    if (!ok)
    {
        yInfo() << "FingerController::goHome Error:"
                << "unable to restore initial positions of joints of finger"
                << finger_name;

        // stop movements for safety
        stop();

        return false;
    }

    return true;
}

bool FingerController::isPositionMoveDone(bool &done)
{
    bool ok;

    return ipos->checkMotionDone(ctl_joints.size(),
                                 ctl_joints.getFirst(),
                                 &done);
}

void FingerController::enforceJointsLimits(yarp::sig::Vector &vels)
{
    // enforce joints limits
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        int &joint_index = ctl_joints[i];
        double &des_limit = joints_des_limits[i];
        if (motors_encoders[joint_index] > des_limit)
            vels[i] = 0.0;
    }
}

void FingerController::enforceJointsMaxLimits(yarp::sig::Vector &vels)
{
    // enforce joints limits
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        int &joint_index = ctl_joints[i];
        double &max_limit = joints_max_limits[i];
        if (motors_encoders[joint_index] > max_limit)
            vels[i] = 0.0;
    }
}

bool FingerController::setJointsVelocities(const yarp::sig::Vector &vels,
                                           const bool &enforce_joints_limits)
{
    bool ok;

    // local copy of velocities
    yarp::sig::Vector velocities = vels;

    // switch to velocity control
    if (control_mode != VOCAB_CM_VELOCITY)
    {
        ok = setControlMode(VOCAB_CM_VELOCITY);
        if (!ok)
        {
            yInfo() << "FingerController::setJointsVelocites Error:"
                    << "unable to set Velocity control mode for finger"
                    << finger_name;

            return false;
        }
    }

    // enforce joints desired limits limits if required
    if (enforce_joints_limits)
        enforceJointsLimits(velocities);

    // always enforce joints max limits
    enforceJointsMaxLimits(velocities);

    // convert velocities to deg/s
    yarp::sig::Vector vels_deg = velocities * (180.0/M_PI);

    // issue velocity command
    return ivel->velocityMove(ctl_joints.size(), ctl_joints.getFirst(), vels_deg.data());
}

bool FingerController::moveFingerForward(const double &speed,
                                         const bool &enforce_joints_limits)
{
    // get the jacobian in the current configuration
    yarp::sig::Matrix jac;
    getJacobianFingerFrame(jac);

    // remove attitude part (i.e. third row)
    jac.removeRows(2, 1);

    // remove velocity along x part (i.e. first row)
    jac.removeRows(0, 1);

    // find joint velocities minimizing v_y - J_y * q_dot
    yarp::sig::Vector q_dot;
    yarp::sig::Vector vel(1, speed);
    yarp::sig::Matrix jac_inv;

    jac_inv = jac.transposed() *
        yarp::math::pinv(jac * jac.transposed());
    q_dot = jac_inv * vel;

    // try to avoid too much displacement for the first
    // joint for fingers index and middle
    if (finger_name == "index" ||
        finger_name == "middle")
    {
        // evaluate null projector
        yarp::sig::Matrix eye2(2, 2);
        yarp::sig::Matrix projector;

        eye2.eye();
        projector = eye2 - jac_inv * jac;

        // get current value of the first joint
        double joint;
        if (finger_name == "index")
            joint = joints[1];
        else
            joint = joints[0];

        // evaluate gradient of the repulsive potential
        yarp::sig::Vector q_dot_limits(2, 0.0);
        double joint_comfort = prox_comfort_value * (M_PI / 180);
        double joint_max = prox_max_value * (M_PI / 180);
        q_dot_limits[0] = -0.5 * (joint - joint_comfort) /
            pow(joint_max, 2);

        q_dot += projector * prox_proj_gain * q_dot_limits;
    }

    // issue velocity command
    bool ok = setJointsVelocities(q_dot, enforce_joints_limits);
    if (!ok)
    {
        yInfo() << "FingerController::moveFingerForward Error:"
                << "unable to set joints velocities for finger"
                << finger_name;

        // stop movements for safety
        stop();

        return false;
    }

    return true;
}

bool FingerController::stop()
{
    bool ok;

    // stop motion
    ok = ivel->stop(ctl_joints.size(), ctl_joints.getFirst());
    if (!ok)
        return false;

    return true;
}
