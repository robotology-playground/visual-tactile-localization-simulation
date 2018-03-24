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
    yarp::sig::Matrix rob_2_work_frame;
    double attitude;

public:
    ModelHelper();
    void setModelAttitude(const yarp::sig::Matrix &rot);
};

#endif
