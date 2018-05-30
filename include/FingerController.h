/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef FINGER_CONTROLLER_H
#define FINGER_CONTROLLER_H

// yarp
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/api.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/math/SVD.h>
#include <yarp/os/ResourceFinder.h>

// icub-main
/* #include <iCub/iKin/iKinFwd.h> */

// std
#include <string>

#include <cmath>

#include <fwd_kin_ext.h>

class FingerController
{
private:
    //
    iCub::iKin::iCubFingerExt finger;

    // name of the finger
    std::string finger_name;

    // name of the hand
    std::string hand_name;

    // joints values
    yarp::sig::Vector joints;

    // list of joints to be controlled
    yarp::sig::VectorOf<int> ctl_joints;

    // list of desired joints limits
    yarp::sig::VectorOf<double> joints_des_limits;

    // list of max joints limits
    yarp::sig::VectorOf<double> joints_max_limits;

    // list of control modes at startup
    yarp::sig::VectorOf<int> initial_modes;

    // jacobian coupling matrix
    yarp::sig::Matrix coupling;

    // position and attitude of finger root frame
    // w.r.t hand root frame
    yarp::sig::Vector finger_root_pos;
    yarp::sig::Matrix finger_root_att;

    // initial joints configuration
    yarp::sig::Vector joints_home;

    // velocity control interface
    // common to all fingers
    yarp::dev::IVelocityControl2 *ivel;

    // position control interface
    // common to all fingers
    yarp::dev::IPositionControl2 *ipos;

    // control mode interface
    // common to all fingers
    yarp::dev::IControlMode2 *imod;

    // control limits interface
    // common to all fingers
    yarp::dev::IControlLimits2 *ilim;

    // current control mode
    int control_mode;

    // parameters for null based control
    // of proximal joints of index and middle fingers
    double prox_comfort_value;
    double prox_max_value;
    double prox_proj_gain;

    // copy of motor encoders values
    yarp::sig::Vector motors_encoders;

public:
    /*
     * Initialize the controller.
     *
     * @param hand_name the name of the hand
     * @param finger_name the name of the finger
     * @param imod pointer to a ControlMode2 instance
     * @param ivel pointer to a ControlLimits2 instance
     * @param ipos pointer to a PositionControl2 instance
     * @param ivel pointer to a VelocityControl2 instance
     * @return true/false on success/failure
     */
    bool init(const std::string &hand_name,
              const std::string &finger_name,
              yarp::dev::IControlMode2 *imod,
              yarp::dev::IControlLimits2 *ilim,
              yarp::dev::IPositionControl2 *ipos,
              yarp::dev::IVelocityControl2 *ivel);
    /*
     * Configure controller taking parameters from ResourceFinder.
     *
     */
    bool configure(const yarp::os::ResourceFinder &rf);

    /*
     * Align finger chain joints bounds using limits from IControlLimits
     */
    bool alignJointsBounds();

    /*
     * Set a given control mode for all the joints of the finger.
     *
     * @return true/false on success/failure
     */
    bool setControlMode(const int &mode);

    /*
     * Close the controller.
     *
     * @return true/false on success/failure
     */
    bool close();

    /*
     * Set the home position of the finger joints.
     *
     * @param encoders a vector containing the encoders
     * readings of the whole arm
     */
    void setHomePosition(const yarp::sig::Vector &encoders);

    /*
     * Update the state of the finger joints using
     * the motor encoders readings.
     *
     * @param motor_encs a vector containing the motor encoders
     *        readings of the whole arm
     * @return true/false on success/failure
     */
    bool updateFingerChain(const yarp::sig::Vector &motor_encs);

    /*
     * Update the state of the finger joints using
     * the motor encoders readings.
     *
     * @param motor_encs a vector containing the motor encoders
     *        readings of the whole arm
     * @param analogs_encs a vector containing the additional encoders
     *        of the fingers
     * @return true/false on success/failure
     */
    bool updateFingerChain(const yarp::sig::Vector &motor_encs,
                           const yarp::sig::Vector &analogs_encs);

    /*
     * Get the Jacobian of the finger considering the finger as a planar chain.
     * The jacobian is such that the linear velocity is expressed in the root frame
     * of the finger, the abduction is neglected and only the actuated DoFs are
     * considered (e.g. only one DoF is available for the coupled distal joints).
     *
     * @param jacobian a 3x2 matrix containing the extracted jacobian
     * @return true/false on success/failure
     */
    bool getJacobianFingerFrame(yarp::sig::Matrix &jacobian);

    /*
     * Get the pose of the finger tip considering the finger as a planar chain.
     * The 2D position is expressed in the root frame of the finger.
     * The attitude is the sum of the controlled joint angles.
     *
     * @param pose a vector containing the pose of the finger tip
     * @return true/false on success/failure
     */
    /* bool getFingerTipPoseFingerFrame(yarp::sig::Vector &pose); */

    /*
     * Restore the initial position of the finger.
     *
     * @param ref_vel reference joints velocity used during movement
     * @return true/false on success/failure
     */
    bool goHome(const double &ref_vel);

    /*
     * Return the status of a movement
     * issued with the position control inteface.
     *
     * May be used after a call to goHome() for example.
     *
     * @param done the status of the movement
     * @return true/false on success/failure
     */
    bool isPositionMoveDone(bool &done);

    /*
     * Set to zero commanded velocity of joints that are above desired limit
     *
     * @param vels a yarp::sig::Vector vector containing the velocities
     */
    void enforceJointsLimits(yarp::sig::Vector &vels);

    /*
     * Set to zero commanded velocity of joints that are above max allowed value
     *
     * @param vels a yarp::sig::Vector vector containing the velocities
     */
    void enforceJointsMaxLimits(yarp::sig::Vector &vels);

    /*
     * Set the velocities of the controlled joints of the finger.
     *
     * To be used in "streaming" mode.
     *
     * @param vels a yarp::sig::Vector vector containing the desired joints velocity
     * @param enforce_joints_limits whether to enforce joints limits or not
     * @return true/false on success/failure
     */
    bool setJointsVelocities(const yarp::sig::Vector &vels,
                             const bool &enforce_joints_limits);

    /*
     * Move the finger forward with a given speed.
     *
     * In details this method set the joint velocities
     * required to have the finger tip moving with a prescribed
     * velocity along the y-axis of the root frame of the finger.
     * The instantaneous velocity along the x-axis is not zero and
     * depends on the value of the inverse jacobian operator,
     * mapping the cartesian velocity to the joints velocity,
     * in the current configuration.
     *
     * To be used in "streaming" mode.
     *
     * @param speed the desired forward speed
     *        (negative speed move the finger backward)
     * @param enforce_joints_limits whether to enforce joints limits or not
     * @return true/false on success/failure
     */
    bool moveFingerForward(const double &speed,
                           const bool &enforce_joints_limits);

    /*
     * Stop the finger.
     *
     * @return true/false on success/failure
     */
    bool stop();
};

#endif
