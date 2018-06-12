/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

// yarp
#include <yarp/sig/Vector.h>

class TrajectoryGenerator
{
private:

    // initial position
    yarp::sig::Vector pos_i;

    // final position
    yarp::sig::Vector pos_f;

    // polynomoinal trajectory constants
    yarp::sig::Vector a0;
    yarp::sig::Vector a3;
    yarp::sig::Vector a4;
    yarp::sig::Vector a5;

    // trajectory duration
    double traj_duration;

public:
    /**
     * Constructor
     */
    TrajectoryGenerator();

    /**
     * Set the initial position.
     * @param pos the 3x1 initial position
     * @return true/false on success/failure
     */
    bool setInitialPosition(const yarp::sig::Vector &pos);

    /**
     * Set the final position.
     * @param pos the 3x1 final position
     * @return true/false on success/failure
     */
    bool setFinalPosition(const yarp::sig::Vector &pos);

    /**
     * Set the duration of the trajectory.
     * @param duration the positive duration in seconds
     * @return true/false on success/failure
     */
    bool setDuration(const double &duration);

    /**
     * Initialize the trajectory generator.
     */
    void init();

    /**
     * Get the trajectory position and velocity at specified time.
     * @param time the value of the time
     * @param position the 3x1 position at the specified time
     * @param velocity the 3x1 velocity at the specified time
     * @return true/false on success/failure
     */
    bool getTrajectory(const double &time,
                       yarp::sig::Vector &position,
                       yarp::sig::Vector &velocity);
};

#endif
