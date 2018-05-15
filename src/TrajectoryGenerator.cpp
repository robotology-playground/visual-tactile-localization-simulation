/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// yarp
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>

#include <cmath>

#include "TrajectoryGenerator.h"

using namespace yarp::math;

TrajectoryGenerator::TrajectoryGenerator()
{
    // clear initial and final position
    pos_i.resize(3, 0.0);
    pos_f.resize(3, 0.0);

    // clear trajectory duration
    traj_duration = 1.0;

    // clear trajectory constants
    a0.resize(3, 0.0);
    a3.resize(3, 0.0);
    a4.resize(3, 0.0);
    a5.resize(3, 0.0);    
}

bool TrajectoryGenerator::setInitialPosition(const yarp::sig::Vector &pos)
{
    if (pos.size() != 3)
	return false;
    
    pos_i = pos;

    return true;
}

bool TrajectoryGenerator::setFinalPosition(const yarp::sig::Vector &pos)
{
    if (pos.size() != 3)
	return false;
    
    pos_f = pos;

    return true;
}

bool TrajectoryGenerator::setDuration(const double &duration)
{
    if (duration <= 0)
	return false;
    
    traj_duration = duration;

    return true;
}

void TrajectoryGenerator::init()
{
    // set initial conditions
    a0 = pos_i;

    // set difference between initial
    // and final conditions
    a5 = a4 = a3 = a0 - pos_f;

    // eval the polynomial trajectory constants
    a3 *= (-10.0 / pow(traj_duration, 3));
    a4 *= (15.0 / pow(traj_duration, 4));
    a5 *= (-6.0 / pow(traj_duration, 5));
}

bool TrajectoryGenerator::getTrajectory(const double &time,
                                        yarp::sig::Vector &position,
					yarp::sig::Vector &velocity)
{
    // check for negative times
    if (time < 0)
	return false;
    
    // enforce duration of the trajectory
    double t = time;
    if (t > traj_duration)
	t = traj_duration;
    
    // eval current position
    position = a0
	     + a3 * pow(t, 3.0)
	     + a4 * pow(t, 4.0)
	     + a5 * pow(t, 5.0);

    // eval current velocity
    velocity = 3.0 * a3 * pow(t, 2.0)
	     + 4.0 * a4 * pow(t, 3.0)
	     + 5.0 * a5 * pow(t, 4.0);

    return true;
}
