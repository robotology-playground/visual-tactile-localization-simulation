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
                                    const std::string &cam_calib_path)
{
    if ((n_x <= 0) || (n_y <= 0) || (size1 <= 0) || (size2 < 0))
        return false;
    
    // configure a standard Aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    // configure the board
    // size1 is the marker side (m)
    // size2 is the distance between markers (m)
    cv::Ptr<cv::aruco::GridBoard> gridboard =cv::aruco::GridBoard::create(n_x, n_y, size1, size2, dictionary);
    board = gridboard.staticCast<cv::aruco::Board>();

    // configure the detector
    detector_params = cv::aruco::DetectorParameters::create();
    // enable corner refinement
    detector_params->doCornerRefinement = true;

    // set axis length
    axis_length = 0.5 * ((double)std::min(n_x, n_y) * (size1 + size2) + size2);

    // load camera calibration
    if (!readCameraParameters(cam_calib_path))
        return false;

    return true;
}

bool ArucoBoardEstimator::estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out,
                                            cv::Mat &pos, cv::Mat &att)
{
    // perform marker detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners, rejected;
    cv::aruco::detectMarkers(img_in, dictionary, corners, ids, detector_params, rejected);

    // perform refinement
    cv::aruco::refineDetectedMarkers(img_in, board, corners, ids, rejected,
                                     cam_intrinsic, cam_distortion);

    // copy input image to output image
    img_in.copyTo(img_out);
    
    // if at least one marker detected
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(img_out, corners, ids);
        int valid = estimatePoseBoard(corners, ids, board,
                                      cam_intrinsic, cam_distortion,
                                      att, pos);
        // if at least one board marker detected
        if(valid > 0)
        {
            cv::aruco::drawAxis(img_out, cam_intrinsic, cam_distortion, att, pos, axis_length);
            return true;
        }
    }

    return false;
}

