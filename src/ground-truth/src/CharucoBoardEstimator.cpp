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

bool CharucoBoardEstimator::configure(const int &n_x, const int &n_y, const double &size1, const double &size2,
                                      const std::string &cam_calib_path)
{
    if ((n_x <= 0) || (n_y <= 0) || (size1 <= 0) || (size2 < 0))
        return false;
    
    // configure a standard Aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // configure the board
    // size1 is the chess square side (m)
    // size2 is the marker side (m)
    charucoboard = cv::aruco::CharucoBoard::create(n_x, n_y, size1, size2, dictionary);
    board = charucoboard.staticCast<cv::aruco::Board>();

    // configure the detector
    detector_params = cv::aruco::DetectorParameters::create();    
    // enable corner refinement
    detector_params->doCornerRefinement = true;

    // set axis length
    axis_length = 0.5 * ((double)std::min(n_x, n_y) * (size2));

    // load camera calibration
    if (!readCameraParameters(cam_calib_path))
        return false;

    return true;
}

bool CharucoBoardEstimator::estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out,
                                              cv::Mat &pos, cv::Mat &att)
{
    std::vector<int> marker_ids, charuco_ids;
    std::vector<std::vector<cv::Point2f> > marker_corners, rejected_markers;
    std::vector<cv::Point2f> charuco_corners;

    // perform marker detection    
    cv::aruco::detectMarkers(img_in, dictionary, marker_corners, marker_ids, detector_params,
                             rejected_markers);

    // refind strategy to detect more markers
    cv::aruco::refineDetectedMarkers(img_in, board, marker_corners, marker_ids, rejected_markers,
                                     cam_intrinsic, cam_distortion);

    // interpolate charuco corners
    int interpolated_corners = 0;
    if(marker_ids.size() > 0)
        interpolated_corners =
            cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, img_in, charucoboard,
                                                 charuco_corners, charuco_ids, cam_intrinsic, cam_distortion);

    // estimate charuco board pose
    bool valid_pose = false;
    if(cam_intrinsic.total() != 0)
        valid_pose = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, charucoboard,
                                                         cam_intrinsic, cam_distortion, att, pos);

    // copy input image to output image
    img_in.copyTo(img_out);
    if(marker_ids.size() > 0)
        cv::aruco::drawDetectedMarkers(img_out, marker_corners);

    if(valid_pose)
    {
        cv::aruco::drawAxis(img_out, cam_intrinsic, cam_distortion, att, pos, axis_length);

        return true;
    }

    return false;
}

