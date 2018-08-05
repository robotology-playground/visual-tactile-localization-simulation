/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

// yarp
#include <yarp/os/LogStream.h>
#include <yarp/math/SVD.h>

//
#include <KalmanFilter.h>
#include <cmath>

double KalmanFilter::wrapAngle(const double& angle)
{
    double wrapped = angle;
    while (wrapped > M_PI)
    {
        wrapped -= 2 * M_PI;
    }

    while (wrapped < -M_PI)
    {
        wrapped += 2* M_PI;
    }

    return wrapped;
}

yarp::sig::Vector KalmanFilter::wrapAngles(const yarp::sig::Vector &state)
{
    yarp::sig::Vector wrapped = state;
    
    // wrap angles
    for (size_t i=0; i<euler_indexes.size(); i++)
        wrapped[euler_indexes[i]] = wrapAngle(state[euler_indexes[i]]);

    return wrapped;
}

void KalmanFilter::doCorrection(const yarp::sig::Vector &meas)
{
    // wrap angles
    yarp::sig::Vector wrap_meas;
    wrap_meas = wrapAngles(meas);

    // eval Kalman gain
    yarp::sig::Matrix invS = yarp::math::pinv(S);
    K = P * Ht * invS;

    // eval error
    yarp::sig::Vector e = wrap_meas-get_y();

    // wrap angles
    e = wrapAngles(e);

    // add innovation
    x += K*e;

    // wrap angles
    x = wrapAngles(x);

    // correct covariance of estimate
    P = (I-K*H) * P;
    validationGate=yarp::math::dot(e,invS*e);
}

void KalmanFilter::setT(const double &T)
{
    this->T = T;
}

void KalmanFilter::setStateSize(const int& n)
{
    this->n = n;
}

void KalmanFilter::setEulerAngles(const yarp::sig::VectorOf<int> &indexes)
{
    euler_indexes = indexes;
}

void KalmanFilter::setQ(const yarp::sig::Matrix &Q)
{
    Q_cont = Q;
}

void KalmanFilter::setR(const yarp::sig::Matrix &R)
{
    R_cont = R;
}

void KalmanFilter::init()
{
    // nxn eye matrix
    yarp::sig::Matrix eye(n, n);
    eye.eye();
    
    // A matrix
    A.resize(2 * n, 2 * n);
    A.zero();
    A.setSubmatrix(eye, 0, 0);
    A.setSubmatrix(T * eye, 0, n);
    A.setSubmatrix(eye, n, 0);

    yInfo() << "A matrix is" << A.toString();

    // B matrix
    B.resize(2 * n, n);
    B.zero();
    B.setSubmatrix(eye * pow(T, 2) / 2.0, 0, 0);
    B.setSubmatrix(eye * T, n, 0);

    yInfo() << "B matrix is" << B.toString();

    // H matrix
    H.resize(n, 2 * n);
    H.zero();
    H.setSubmatrix(eye, 0, 0);

    yInfo() << "H matrix is" << H.toString();

    // Q matrix
    Q.resize(n, n);
    Q.zero();
    Q.setSubmatrix(Q_cont * pow(T, 3) / 3.0, 0, 0);
    Q.setSubmatrix(Q_cont * pow(T, 2) / 2.0, 0, n);
    Q.setSubmatrix(Q_cont * pow(T, 2) / 2.0, n, 0);
    Q.setSubmatrix(Q_cont * T, n, n);

    yInfo() << "Q matrix is" << Q.toString();

    // R matrix
    R.resize(n, n);
    R.zero();
    // R = R_cont / T;
    R = R_cont;

    yInfo() << "R matrix is" << R.toString();

    // initialize using method Kalman::initialize()
    initialize();
}

yarp::sig::Vector KalmanFilter::step(const yarp::sig::Vector &meas)
{
    // predict with zero inputs
    yarp::sig::Vector zero_input(n, 0.0);
    predict(zero_input);

    // wrap angles
    x = wrapAngles(x);

    // correction
    doCorrection(meas);

    return x;
}


