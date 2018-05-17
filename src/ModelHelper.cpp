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

// std
#include <string>

#include <ModelHelper.h>

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

void ModelHelper::configure(const yarp::os::ResourceFinder &rf)
{
    // sizes of the object model
    model_width = rf.find("modelWidth").asDouble();
    if (rf.find("modelWidth").isNull())
        model_width = 0.24;

    model_depth = rf.find("modelDepth").asDouble();
    if (rf.find("modelDepth").isNull())
        model_depth = 0.17;

    model_height = rf.find("modelHeight").asDouble();
    if (rf.find("modelHeight").isNull())
        model_height = 0.037;

    offset_x_y = rf.find("offsetXY").asDouble();
    if (rf.find("offsetXY").isNull())
        offset_x_y = 0.06;

    offset_h = rf.find("offsetHeight").asDouble();
    if (rf.find("offsetHeight").isNull())
        offset_h = 0.021;
}

void ModelHelper::setModelDimensions(const double &width,
                        const double &depth,
                        const double &height)
{
    model_width = width;
    model_depth = depth;
    model_height = height;
}

void ModelHelper::setModelAttitude(const yarp::sig::Matrix &rot)
{
    // transform to working frame
    yarp::sig::Matrix transformed_rot = rob_2_work_frame.transposed() * rot;

    // check if the z axis is pointing downwards
    yarp::sig::Vector z_down(3, 0.0);
    z_down[2] = -1.0;
    double cosine = yarp::math::dot(z_down, transformed_rot.getCol(2));
    if (cosine >= 0)
    {
        // in this case the z and y cosine director
        // are rotated so that the z axis points upwards
        transformed_rot.setCol(1, transformed_rot.getCol(1) * -1.0);
        transformed_rot.setCol(2, transformed_rot.getCol(2) * -1.0);
    }

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
}

void ModelHelper::setModelPosition(const yarp::sig::Vector &pos)
{
    model_center = pos;
}

void ModelHelper::setModelPose(const yarp::sig::Matrix &pose)
{
    setModelPosition(pose.getCol(3).subVector(0, 2));
    setModelAttitude(pose.submatrix(0, 2, 0, 2));
}

double ModelHelper::evalApproachYawAttitude()
{
    if (attitude >= (M_PI / 4.0) &&
        attitude < (M_PI / 4.0 + M_PI / 2.0))
        return (attitude - M_PI / 2.0);
    else if (attitude >= (M_PI / 4.0 + M_PI / 2.0) &&
             attitude <= M_PI)
        return (attitude - M_PI);
    else
        return attitude;
}

void ModelHelper::evalApproachPosition(yarp::sig::Vector &pos,
                                       const std::string &object_position_name)
{
    // assign the center of model
    pos = model_center;

    // pick the right direction and length
    // for evaluation of offset
    yarp::sig::Vector direction;
    double length;
    if (attitude >= (M_PI / 4.0) &&
        attitude < (M_PI / 4.0 + M_PI / 2.0))
    {
        length = model_width / 2.0 + offset_x_y;
        direction = x_dir * length;

        if (object_position_name == "left")
            direction += y_dir * model_depth / 4.0;
        else if (object_position_name == "right")
            direction += -1.0 * y_dir * model_depth / 4.0;

    }
    else if (attitude >= (M_PI / 4.0 + M_PI / 2.0) &&
             attitude <= M_PI)
    {
        length = model_depth / 2.0 + offset_x_y;
        direction = -1 * y_dir * length;

        if (object_position_name == "left")
            direction += x_dir * model_width / 4.0;
        else if (object_position_name == "right")
            direction += -1.0 * x_dir * model_width / 4.0;
    }
    else
    {
        length = model_depth / 2.0 + offset_x_y;
        direction = y_dir * length;

        if (object_position_name == "left")
            direction += -1.0 * x_dir * model_width / 4.0;
        else if (object_position_name == "right")
            direction += x_dir * model_width / 4.0;
    }

    // add position offset
    yarp::sig::Vector direction_3d(3, 0.0);
    direction_3d.setSubvector(0, direction);
    pos += rob_2_work_frame * direction_3d;

    // add height offset
    pos[2] += model_height / 2.0 + offset_h;
}
#endif
