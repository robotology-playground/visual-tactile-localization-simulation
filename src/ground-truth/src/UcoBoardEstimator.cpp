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
