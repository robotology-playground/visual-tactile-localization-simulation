/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iCub/ctrl/kalman.h>

class KalmanFilter : public iCub::ctrl::Kalman
{
protected:
    // sampling time
    double T;

    // state size
    int n;

    // which states are euler angles
    yarp::sig::VectorOf<int> euler_indexes;

    // continuous time Q and R
    yarp::sig::Matrix Q_cont;
    yarp::sig::Matrix R_cont;

    double wrapAngle(const double& angle);
    yarp::sig::Vector wrapAngles(const yarp::sig::Vector &state);
    void doCorrection(const yarp::sig::Vector &meas);
public:
    void setT(const double &T);
    void setStateSize(const int &n);
    void setEulerAngles(const yarp::sig::VectorOf<int> &indexes);
    void setQ(const yarp::sig::Matrix &Q);
    void setR(const yarp::sig::Matrix &R);
    void setInitialConditions(const yarp::sig::Vector &x0,
                              const yarp::sig::Matrix &P0);
    void init();
    yarp::sig::Vector step(const yarp::sig::Vector &meas);
};

#endif
