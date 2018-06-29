/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

//
#include <UcoBoardEstimator.h>

bool UcoBoardEstimator::readCameraParameters(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    
    fs["camera_matrix"] >> cam_intrinsic;
    fs["distortion_coefficients"] >> cam_distortion;
    
    return true;
}

bool UcoBoardEstimator::setCameraIntrinsics(const double &fx, const double &fy,
                                            const double &cx, const double &cy)
{
    cam_intrinsic.at<double>(0, 0) = fx;
    cam_intrinsic.at<double>(0, 2) = cx;
    cam_intrinsic.at<double>(1, 1) = fy;
    cam_intrinsic.at<double>(1, 2) = cy;
}
