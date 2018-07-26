/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

#include <FingerController.h>

using namespace yarp::math;

bool FingerController::init(const std::string &hand_name,
                            const std::string &finger_name,
                            yarp::dev::IControlMode *imod,
                            yarp::dev::IControlLimits *ilim,
                            yarp::dev::IPositionControl *ipos,
                            yarp::dev::IVelocityControl *ivel)
{
    bool ok;

    // reset current control mode
    control_mode = -1;

    // store pointer to ControlMode instance
    this->imod = imod;

    // store pointer to ControlLimits instance
    this->ilim = ilim;

    // store pointer to PositionControl instance
    this->ipos = ipos;

    // store pointer to VelocityControl instance
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
    initial_modes.resize(ctl_joints.size());
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        ok = false;
        double t0 = yarp::os::SystemClock::nowSystem();
        while (!ok && (yarp::os::SystemClock::nowSystem() - t0 < 10.0))
        {
            ok = imod->getControlMode(ctl_joints[i], &(initial_modes[i]));
            // ok = imod->getControlModes(ctl_joints.size(),
            //                            ctl_joints.getFirst(),
            //                            initial_modes.getFirst());

            yarp::os::SystemClock::delaySystem(1.0);
        }
        if (!ok)
        {
            yError() << "FingerController:configure"
                     << "Error: unable to get the initial mode"
                     << "for the joints of the"
                     << hand_name << finger_name
                     << "finger";

            return false;
        }
    }

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

    // set defaults for smooth switching
    use_smooth_switch = false;

    // set default for thumb parking
    thumb_parking_mode = false;

    // setup analog bounds
    setupAnalogBounds();

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

        if (!rf.find("thumbParkingMode").isNull())
        {
            auto thumb_parking_mode_v = rf.find("thumbParkingMode");
            if (thumb_parking_mode_v.isBool())
            {
                thumb_parking_mode = thumb_parking_mode_v.asBool();
                if (thumb_parking_mode)
                    executeThumbParking();
            }
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

    if (rf.find("pinvDamping").isNull())
        pinv_damping = 0.0;
    else
    {
        auto pinv_damping_v = rf.find("pinvDamping");
        if (pinv_damping_v.isDouble())
            pinv_damping = pinv_damping_v.asDouble();
        else
            pinv_damping = 0.0;
    }

    if (!rf.find("enableSmoothSwitch").isNull())
    {
        use_smooth_switch = rf.find("enableSmoothSwitch").asBool();
        if (use_smooth_switch)
        {
            if (!rf.find("smoothSwitchThreshold").isNull())
            {
                auto smooth_switch_thr_v = rf.find("smoothSwitchThreshold");
                if (smooth_switch_thr_v.isDouble())
                    smooth_switch_threshold = smooth_switch_thr_v.asDouble();
                else
                    smooth_switch_threshold = 0.0;
            }
            if (!rf.find("smoothSwitchVariance").isNull())
            {
                auto smooth_switch_var_v = rf.find("smoothSwitchVariance");
                if (smooth_switch_var_v.isDouble())
                    smooth_switch_variance = smooth_switch_var_v.asDouble();
                else
                    smooth_switch_variance = 1.0;
            }
        }
    }
}

bool FingerController::alignJointsBounds()
{
    std::deque<yarp::dev::IControlLimits*> lims;
    lims.push_back(ilim);
    return finger.alignJointsBounds(lims);
}

bool FingerController::setControlMode(const int &mode)
{
    bool ok;

    //get current control modes first
    yarp::sig::VectorOf<int> modes(ctl_joints.size());
    ok = imod->getControlModes(ctl_joints.size(),
                               ctl_joints.getFirst(),
                               modes.getFirst());
    if (!ok)
    {
        yError() << "FingerController:setControlMode"
                 << "Error: unable to get current joints control modes for finger"
                 << hand_name << finger_name;
        return false;
    }

    // set only the control modes different from the desired one
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        if (modes[i] != mode)
        {
            ok = imod->setControlMode(ctl_joints[i], mode);
            if (!ok)
            {
                yError() << "FingerController:setControlMode"
                         << "Error: unable to set control mode for one of the joint of the finger"
                         << hand_name << finger_name;
                return false;
            }
        }
    }

    // if all joints were set store the current control mode
    control_mode = mode;

    return true;
}

bool FingerController::close()
{
    bool ok;

    // if the thumb is in parking mode
    // it is required to restore its status to a safe one
    if (thumb_parking_mode)
    {
        // undoThumbParking restore thumb opposition to a save value of 10.0 degrees
        ok = undoThumbParking();
        if (!ok)
        {
            yError() << "FingerController::close"
                     << "WARNING: unable to undo thumb parking";
            return false;
        }

        // restore position control for thumb
        yarp::sig::VectorOf<int> joints;
        joints.push_back(8);
        joints.push_back(9);
        joints.push_back(10);
        for (size_t i=0; i<joints.size(); i++)
        {
            ok = imod->setControlMode(joints[i],
                                      VOCAB_CM_POSITION);
            if (!ok)
            {
                yError() << "FingerController:close"
                         << "Error: unable to set position of joints of"
                         << hand_name << "thumb";
                return false;
            }
        }

        return true;
    }

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
    for (size_t i=0; i<ctl_joints.size(); i++)
    {
        ok = imod->setControlMode(ctl_joints[i],
                                  initial_modes[i]);
        if (!ok)
        {
            yError() << "FingerController:close"
                     << "Error: unable to restore the initial control modes for finger"
                     << hand_name << finger_name;
            return false;
        }
    }

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

    // store motor encoders that are required to enforce joints limits
    motors_encoders = motor_encs;

    // get subset of joints related to the finger
    ok = finger.getChainJoints(motor_encs, analogs_encs, joints, analog_bounds);
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

    ok = ipos->checkMotionDone(ctl_joints.size(),
                               ctl_joints.getFirst(),
                               &done);

    if (!ok)
    {
        yInfo() << "FingerController::isPositionMoveDone Error:"
                << "unable to get status from IPositionControl::checkMotionDone for finger"
                << finger_name;
        return false;
    }

    return true;
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

void FingerController::jacPsuedoInversion(const yarp::sig::Matrix &jac,
                                          const double &speed,
                                          yarp::sig::Vector &q_dot,
                                          double &singularity)
{
    yarp::sig::Vector vel(1, speed);
    yarp::sig::Matrix jac_inv;

    yarp::sig::Matrix jac_jac_tr = jac * jac.transposed();
    singularity = jac_jac_tr(0, 0);

    jac_inv = jac.transposed() *
        yarp::math::pinv(jac_jac_tr + pinv_damping * yarp::math::eye(1));
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
}

bool FingerController::moveFingerForward(const double &speed_x, const double &speed_y,
                                         const bool &enforce_joints_limits)
{
    // get the jacobian in the current configuration
    yarp::sig::Matrix jac;
    yarp::sig::Matrix jac_x;
    yarp::sig::Matrix jac_y;
    getJacobianFingerFrame(jac);
    jac_y = jac_x = jac;

    // remove attitude part (i.e. third row)
    jac_x.removeRows(2, 1);
    jac_y.removeRows(2, 1);

    // remove velocity along x part (i.e. first row)
    jac_y.removeRows(0, 1);

    // remove velocity along y part (i.e. second row)
    jac_x.removeRows(1, 1);

    // find joint velocities minimizing v - J * q_dot
    yarp::sig::Vector q_dot_x;
    yarp::sig::Vector q_dot_y;
    double jac_x_singularity;
    double jac_y_singularity;
    jacPsuedoInversion(jac_y, speed_y,
                       q_dot_y, jac_y_singularity);
    jacPsuedoInversion(jac_x, -speed_x,
                       q_dot_x, jac_x_singularity);

    // evaluate smooth transition between velocities
    double mu;
    if (jac_y_singularity < smooth_switch_threshold)
        mu = exp(-0.5 * pow((jac_y_singularity - smooth_switch_threshold) /
                            smooth_switch_variance, 2));
    else
        mu = 1.0;

    // issue velocity command
    yarp::sig::Vector q_dot;
    if (use_smooth_switch)
        q_dot = (1 - mu) * q_dot_x + mu * q_dot_y;
    else
        q_dot = q_dot_y;

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

void FingerController::setupAnalogBounds()
{
    // taken from real robot
    analog_bounds.resize(16, 2);
    if (hand_name == "left")
    {
        analog_bounds(0, 0) = 235.0;
        analog_bounds(1, 0) = 209.0;
        analog_bounds(2, 0) = 237.0;
        analog_bounds(3, 0) = 245.0;
        analog_bounds(4, 0) = 219.0;
        analog_bounds(5, 0) = 233.0;
        analog_bounds(6, 0) = 245.0;
        analog_bounds(7, 0) = 217.0;
        analog_bounds(8, 0) = 249.0;
        analog_bounds(9, 0) = 235.0;
        analog_bounds(10, 0) = 208.0;
        analog_bounds(11, 0) = 234.0;
        analog_bounds(12, 0) = 250.0;
        analog_bounds(13, 0) = 216.0;
        analog_bounds(14, 0) = 230.0;
        analog_bounds(15, 0) = 0.0;
        analog_bounds(0, 1) = 44.0;
        analog_bounds(1, 1) = 14.0;
        analog_bounds(2, 1) = 10.0;
        analog_bounds(3, 1) = 14.0;
        analog_bounds(4, 1) = 36.0;
        analog_bounds(5, 1) = 0.0;
        analog_bounds(6, 1) = 0.0;
        analog_bounds(7, 1) = 6.0;
        analog_bounds(8, 1) = 21.0;
        analog_bounds(9, 1) = 3.0;
        analog_bounds(10, 1) = 25.0;
        analog_bounds(11, 1) = 0.0;
        analog_bounds(12, 1) = 0.0;
        analog_bounds(13, 1) = 39.0;
        analog_bounds(14, 1) = 19.0;
        analog_bounds(15, 0) = 0.0;
    }
}

bool FingerController::executeThumbParking()
{
    if (!thumb_parking_mode)
        return false;

    bool ok;

    // store all indexes of thumb joints
    yarp::sig::VectorOf<int> joints_indexes;
    joints_indexes.push_back(8);
    joints_indexes.push_back(9);
    joints_indexes.push_back(10);

    // get minimum allowed values for thumb joints
    yarp::sig::Vector joints_values(3);
    for (size_t i=0; i<joints_indexes.size(); i++)
    {
        double min;
        double max;
        ilim->getLimits(joints_indexes[i], &min, &max);
        joints_values[i] = min;
    }

    // try to set position control
    for (size_t i=0; i<joints_indexes.size(); i++)
    {
        ok = imod->setControlMode(joints_indexes[i], VOCAB_CM_POSITION);
        if (!ok)
        {
            yInfo() << "FingerController::executeThumbParking Error:"
                    << "unable to set Position control mode for finger"
                    << finger_name;

            return false;
        }
    }

    // set reference joints velocities
    // the same velocity is used for all the joints
    yarp::sig::Vector speeds(joints_indexes.size(), 25.0);
    ok = ipos->setRefSpeeds(joints_indexes.size(),
                            joints_indexes.getFirst(),
                            speeds.data());
    if (!ok)
    {
        yInfo() << "FingerController::executeThumbParking Error:"
                << "unable to set joints reference speeds for finger"
                << finger_name;

        return false;
    }

    // try to set position of thumb joints
    // first proximal and distal joints
    yarp::sig::VectorOf<int> joints_indexes_reduced;
    joints_indexes_reduced.push_back(9);
    joints_indexes_reduced.push_back(10);
    yarp::sig::Vector joints_values_reduced;
    joints_values_reduced.push_back(joints_values[1]);
    joints_values_reduced.push_back(joints_values[2]);
    ok = ipos->positionMove(joints_indexes_reduced.size(),
                            joints_indexes_reduced.getFirst(),
                            joints_values_reduced.data());
    if (!ok)
    {
        yInfo() << "FingerController::executeThumbParking Error:"
                << "unable to set positions of joints of finger"
                << finger_name;
        return false;
    }

    bool done = false;
    double t0 = yarp::os::Time::now();
    // while((!done) && ((yarp::os::Time::now() - t0) < 5.0))
    while((yarp::os::Time::now() - t0) < 5.0)
    {
        ok = ipos->checkMotionDone(joints_indexes_reduced.size(),
                                   joints_indexes_reduced.getFirst(),
                                   &done);
        if (!ok)
        {
            yInfo() << "FingerController::isPositionMoveDone Error:"
                    << "unable to get status from IPositionControl::checkMotionDone for finger"
                    << finger_name;
            return false;
        }
    }

    // try to set position of thumb opposition
    joints_indexes_reduced.clear();
    joints_values_reduced.clear();
    joints_indexes_reduced.push_back(8);
    joints_values_reduced.push_back(joints_values[0]);
    ok = ipos->positionMove(joints_indexes_reduced.size(),
                            joints_indexes_reduced.getFirst(),
                            joints_values_reduced.data());
    if (!ok)
    {
        yInfo() << "FingerController::executeThumbParking Error:"
                << "unable to set positions of joints of finger"
                << finger_name;
        return false;
    }

    done = false;
    t0 = yarp::os::Time::now();
    // while(!done && ((yarp::os::Time::now() - t0) < 5.0))
    while((yarp::os::Time::now() - t0) < 5.0)
    {
        ok = ipos->checkMotionDone(joints_indexes_reduced.size(),
                                   joints_indexes_reduced.getFirst(),
                                   &done);
        if (!ok)
        {
            yInfo() << "FingerController::isPositionMoveDone Error:"
                    << "unable to get status from IPositionControl::checkMotionDone for finger"
                    << finger_name;
            return false;
        }
    }

    // put the joints in idle
    for (size_t i=0; i<joints_indexes.size(); i++)
    {
        ok = imod->setControlMode(joints_indexes[i], VOCAB_CM_IDLE);
        if (!ok)
        {
            yInfo() << "FingerController::executeThumbParking Error:"
                    << "unable to put joints of finger"
                    << finger_name
                    << "in idle";

            return false;
        }
    }
    return true;
}

bool FingerController::undoThumbParking()
{
    if (!thumb_parking_mode)
        return false;

    // thumb opposition index
    int joint_index = 8;

    // try to set position control for thumb opposition
    bool ok = imod->setControlMode(joint_index, VOCAB_CM_POSITION);
    if (!ok)
    {
        yInfo() << "FingerController::undoThumbParking Error:"
                << "unable to set Position control mode for thumb opposition";

        return false;
    }

    // safe minimum value for thumb opposition
    double min_value = 10.0;

    // try to set position of thumb opposition
    ok = ipos->positionMove(1,
                            &joint_index,
                            &min_value);
    if (!ok)
    {
        yInfo() << "FingerController::undoThumbParking Error:"
                << "unable to set the position of thumb opposition joint";
        return false;
    }

    bool done = false;
    double t0 = yarp::os::Time::now();
    // while(!done && ((yarp::os::Time::now() - t0) < 5.0))
    while((yarp::os::Time::now() - t0) < 5.0)
    {
        ok = ipos->checkMotionDone(1,
                                   &joint_index,
                                   &done);
        if (!ok)
        {
            yInfo() << "FingerController::isPositionMoveDone Error:"
                    << "unable to get status from IPositionControl::checkMotionDone for finger"
                    << finger_name;
            return false;
        }
    }
    return true;
}

bool FingerController::switchToPositionControl()
{
    bool ok = setControlMode(VOCAB_CM_POSITION);
    if (!ok)
    {
        yError() << "FingerController:switchToPositionControl"
                 << "Error: unable to set the position control"
                 << "mode for the joints of the"
                 << hand_name << finger_name
                 << "finger";

        return false;
    }

    return true;
}
