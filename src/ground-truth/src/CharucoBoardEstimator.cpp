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
#include <CharucoBoardEstimator.h>

bool CharucoBoardEstimator::readCameraParameters(std::string filename,
                                                 cv::Mat &camMatrix,
                                                 cv::Mat &distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    
    return true;
}

bool CharucoBoardEstimator::configure(const int &n_x, const int &n_y, const double &side, const double &separation,
                                    const double &fx, const double &fy, const double &cx, const double &cy,
                                    const double &k1, const double &k2, const double &p1, const double &p2)
{
    if ((n_x <= 0) || (n_y <= 0) || (side <= 0) || (separation < 0))
        return false;
    
    // configure a standard Aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // configure the board
    board = cv::aruco::GridBoard::create(5, 3, 0.03, 0.01, dictionary);

    yInfo() << n_x;
    yInfo() << n_y;
    yInfo() << side;
    yInfo() << separation;
    
    // configure camera parameters
    camIntrinsic = cv::Mat::zeros(3,3, CV_64F);
    camIntrinsic.at<double>(0,0) = 479.07809511943202;
    camIntrinsic.at<double>(1,1) = 478.04246095075132;
    camIntrinsic.at<double>(0,2) = 320.12051776071775;
    camIntrinsic.at<double>(1,2) = 242.83404139175835;
    camIntrinsic.at<double>(2,2) = 1.0;

    camDistortion = cv::Mat::zeros(1, 5, CV_64F);
    camDistortion.at<double>(0,0) = 0.41912777999907791;
    camDistortion.at<double>(0,1) = -1.4549165611881032;
    camDistortion.at<double>(0,2) = 0.00054516436997473368;
    camDistortion.at<double>(0,3) = 0.00091492336895065713;
    camDistortion.at<double>(0,4) = 1.7333880205465304;
    return true;
}

void CharucoBoardEstimator::estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out)
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

