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

//
#include <ArucoBoardEstimator.h>

bool ArucoBoardEstimator::configure(const int &n_x, const int &n_y, const double &size1, const double &size2,
                   const cv::Mat &camMatrix, const cv::Mat &distCoeffs)
{
    if ((n_x <= 0) || (n_y <= 0) || (size1 <= 0) || (size2 < 0))
        return false;
    
    // configure a standard Aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // configure the board
    // size1 is the length of the marker
    // size2 is the distance between markers
    board = cv::aruco::GridBoard::create(n_x, n_y, size1, size2, dictionary);

    // configure camera parameters
    camIntrinsic = camMatrix;
    camDistortion = distCoeffs;

    return true;
}

void ArucoBoardEstimator::estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out)
{
    // copy input image to output image
    img_in.copyTo(img_out);

    // perform marker detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(img_in, dictionary, corners, ids);
    
    // if at least one marker detected
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(img_out, corners, ids);
        cv::Vec3d rvec, tvec;
        int valid = estimatePoseBoard(corners, ids, board,
                                      camIntrinsic, camDistortion,
                                      rvec, tvec);
        // if at least one board marker detected
        if(valid > 0)
            cv::aruco::drawAxis(img_out, camIntrinsic, camDistortion, rvec, tvec, 0.1);
    }
}

