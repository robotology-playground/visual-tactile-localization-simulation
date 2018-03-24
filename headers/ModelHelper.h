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
#include <yarp/sig/Vector.h>

class ModelHelper
{
private:
    yarp::sig::Vector x_dir;
    yarp::sig::Vector y_dir;
    yarp::sig::Vector model_center;
    yarp::sig::Matrix rob_2_work_frame;
    double attitude;
    double model_width;
    double model_depth;
    double model_height;

public:
    ModelHelper();
    void setModelDimensions(const double &width,
			    const double &depth,
			    const double &height);
    void setModelAttitude(const yarp::sig::Matrix &rot);
    void setModelPosition(const yarp::sig::Vector &pos);
    void setModelPose(const yarp::sig::Matrix &pose);
    double evalApproachYawAttitude();
    void evalApproachPosition(yarp::sig::Vector &pos);
};

#endif
