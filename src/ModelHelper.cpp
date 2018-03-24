/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef MODELHELPER_H

// yarp
#include <yarp/math/Math.h>
#include <yarp/os/LogStream.h>

#include "headers/ModelHelper.h"

#include <cmath>

using namespace yarp::math;

ModelHelper::ModelHelper() : x_dir(2, 0.0),
			      y_dir(2, 0.0)
{
    // precompute the rotation between the robot root frame and 
    // and the working frame used within this helper class
    yarp::sig::Vector axis_angle(4);

    // rotation about z-axis
    axis_angle[0] = 0.0;
    axis_angle[1] = 0.0;
    axis_angle[2] = 1.0;
    axis_angle[3] = +M_PI / 2.0;
    rob_2_work_frame = yarp::math::axis2dcm(axis_angle).submatrix(0, 2, 0, 2);
}

void ModelHelper::setModelAttitude(const yarp::sig::Matrix &rot)
{
    // transform to working frame
    yarp::sig::Matrix transformed_rot = rob_2_work_frame.transposed() * rot;
    
    // pick the first column of the rotation matrix
    // this is the x direction aligned with one of the
    // edges of the object
    yarp::sig::Vector x_dir_3d = transformed_rot.getCol(0);

    // same for the y direction
    yarp::sig::Vector y_dir_3d = transformed_rot.getCol(1);

    // project into the z plane
    x_dir = x_dir_3d.subVector(0, 1);
    y_dir = y_dir_3d.subVector(0, 1);

    // normalize the directions
    x_dir /= yarp::math::norm(x_dir);
    y_dir /= yarp::math::norm(y_dir);

    // evaluate the angle between this direction
    // and a given direction [1, 0]
    attitude = std::atan2(x_dir[1], x_dir[0]);

    // in case of negative angle the direction is rotated
    // by 180 degrees
    if (attitude < 0)
    {
	attitude += M_PI;
	x_dir *= -1;
	y_dir *= -1;
    }

    yInfo() << x_dir.toString();
    yInfo() << y_dir.toString();
    yInfo() << attitude;
}
#endif
