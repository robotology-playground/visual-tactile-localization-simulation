/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef ROTATION_TRAJECTORY_GENERATOR_H
#define ROTATION_TRAJECTORY_GENERATOR_H

// yarp
#include <yarp/sig/Vector.h>

class RotationTrajectoryGenerator
{
private:
    // center of the object
    yarp::sig::Vector object_center;

    // pushing point
    yarp::sig::Vector push_point;

    // yaw rate
    double yaw_rate;
    
public:
    RotationTrajectoryGenerator();
    void setYawRate(const double &rate);
    bool getVelocity(const double &time,
		     yarp::sig::Vector &velocity);
    void setObjectCenter(const yarp::sig::Vector &point);
    void setPullingPoint(const yarp::sig::Vector &point);
};

#endif
