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
#include <yarp/math/SVD.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>

// std
#include <string>

#include <cmath>

class FingerController
{
private:
    //
    iCub::iKin::iCubFinger finger;

    // name of the finger
    std::string finger_name;

    // name of the hand
    std::string hand_name;

    // joints values
    yarp::sig::Vector joints;

    // list of joints to be controlled
    yarp::sig::VectorOf<int> ctl_joints;

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

    // current control mode
    int control_mode;

public:
    /*
     * Initialize the controller.
     *
     * @param hand_name the name of the hand
     * @param finger_name the name of the finger
     * @param imod pointer to a ControlMode2 instance
     * @param ipos pointer to a PositionControl2 instance
     * @param ivel pointer to a VelocityControl2 instance
     * @return true/false on success/failure
     */
    bool init(const std::string &hand_name,
              const std::string &finger_name,
              yarp::dev::IControlMode2 *imod,
              yarp::dev::IPositionControl2 *ipos,
              yarp::dev::IVelocityControl2 *ivel);

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
     * the encoders readings.
     *
     * @param encoders a vector containing the encoders
     *        readings of the whole arm
     * @return true/false on success/failure
     */
    bool updateFingerChain(const yarp::sig::Vector &encoders);

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
     * Set the velocities of the controlled joints of the finger.
     *
     * To be used in "streaming" mode.
     *
     * @param vels a yarp::sig::Vector vector containing the desired joints velocity
     * @return true/false on success/failure
     */
    bool setJointsVelocities(const yarp::sig::Vector &vels);

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
     * @return true/false on success/failure
     */
    bool moveFingerForward(const double &speed);

    /*
     * Stop the finger.
     *
     * @return true/false on success/failure
     */
    bool stop();
};

#endif
