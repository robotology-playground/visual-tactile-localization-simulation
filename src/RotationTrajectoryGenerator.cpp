/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include <yarp/math/Math.h>

#include "RotationTrajectoryGenerator.h"

using namespace yarp::math;

RotationTrajectoryGenerator::RotationTrajectoryGenerator()
{
    // clear vectors
    object_center.resize(3, 0.0);
    push_point.resize(3, 0.0);

    // clear angular rate
    yaw_rate = 0;
}

void RotationTrajectoryGenerator::setObjectCenter(const yarp::sig::Vector &point)
{
    object_center = point;
}

void RotationTrajectoryGenerator::setPushingPoint(const yarp::sig::Vector &point)
{
    push_point = point;
}

void RotationTrajectoryGenerator::setYawRate(const double &rate)
{
    yaw_rate = rate;
}

bool RotationTrajectoryGenerator::getVelocity(const double &time,
					      yarp::sig::Vector &velocity)
{
    if (time < 0)
	return false;
    
    // evaluate displacement
    // from object center to pushing point
    yarp::sig::Vector diff = push_point - object_center;

    // evaluate yaw
    double yaw = yaw_rate * time;

    // evaluate rotation matrix
    yarp::sig::Vector axis_angle(4, 0.0);
    yarp::sig::Matrix rot(3, 3);
    axis_angle[2] = 1.0;
    axis_angle[3] = yaw;
    rot = yarp::math::axis2dcm(axis_angle).submatrix(0, 2, 0, 2);

    // rotate displacement vector
    yarp::sig::Vector diff_rotated = rot * diff;

    // eval angular velocity
    yarp::sig::Vector angular_velocity(3, 0.0);
    angular_velocity[2] = yaw_rate;

    // evaluate linear velocity of pushing point
    // assuming zero velocity of the center of the object
    velocity = yarp::math::cross(angular_velocity, diff_rotated);

    return true;
}
