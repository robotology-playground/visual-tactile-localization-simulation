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
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/math/Math.h>

// opencv
#include <opencv2/opencv.hpp>

// superimpose
#include <SuperimposeMesh/Superimpose.h>
#include <SuperimposeMesh/SICAD.h>

//
// #include <HeadKinematics.h>
#include <GazeController.h>
#include <cstdlib>

class SuperimposeViewer : public yarp::os::RFModule
{
private:
    // head kinematics
    // headKinematics head_kin;

    // gaze controller
    GazeController gaze_ctrl;

    // camera port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> image_input_port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> image_output_port;

    // selected eye name
    std::string eye_name;

    // SICAD objects
    SICAD::ModelPathContainer mesh_container;
    std::unique_ptr<SICAD> est_mesh_cad;
    std::unique_ptr<SICAD> gt_mesh_cad;

    // period
    double period;

    // frame transform client
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;
    std::string gt_tf_source;
    std::string gt_tf_target;
    std::string est_tf_source;
    std::string est_tf_target;

    // poses
    yarp::sig::Matrix estimate;
    yarp::sig::Matrix ground_truth;

    // flags
    bool is_est_available;
    bool is_gt_available;

public:
    bool configure(yarp::os::ResourceFinder &rf) override
    {
        // get parameter from the configuration
        std::string robot_name = "icub";
        if (!rf.find("robotName").isNull())
            robot_name = rf.find("robotName").asString();

        eye_name = "left";
        if (!rf.find("eyeName").isNull())
            eye_name = rf.find("eyeName").asString();

        double cam_width = 320;
        if (!rf.find("camWidth").isNull())
            cam_width = rf.find("camWidth").asDouble();

        double cam_height = 240;
        if (!rf.find("camHeight").isNull())
            cam_height = rf.find("camHeight").asDouble();

        std::string mesh_path = "./mesh.obj";
        if (!rf.find("meshPath").isNull())
            mesh_path = rf.findFile("meshPath");

        period = 0.03;
        if (!rf.find("period").isNull())
            period = rf.find("period").asDouble();

        est_tf_source = "/iCub/frame";
        est_tf_target = "/estimate/frame";
        gt_tf_source = "/iCub/frame";
        gt_tf_target = "/ground_truth/frame";
        if (!rf.find("estTfSource").isNull())
            est_tf_source = rf.find("estTfSource").asString();
        if (!rf.find("estTfTarget").isNull())
            est_tf_target = rf.find("estTfTarget").asString();
        if (!rf.find("gtTfSource").isNull())
            gt_tf_source = rf.find("gtTfSource").asString();
        if (!rf.find("gtTfTarget").isNull())
            gt_tf_target = rf.find("gtTfTarget").asString();

        std::string green_shaders_path = ".";
        std::string red_shaders_path = ".";
        if (!rf.find("greenShadersPath").isNull())
            green_shaders_path = rf.findPath("greenShadersPath");
        if (!rf.find("redShadersPath").isNull())
            red_shaders_path = rf.findPath("redShadersPath");

        // port prefix
        std::string port_prefix = "/si_estimate_viewer/";
        port_prefix += eye_name + "_eye";

        // open port
        bool port_ok = image_input_port.open(port_prefix + ":i");
        if (!port_ok)
            return false;
        port_ok = image_output_port.open(port_prefix + ":o");
        if (!port_ok)
            return false;

        // head forward kinematics
        // if (!head_kin.configure(robot_name, port_prefix))
        //     return false;
        if (!gaze_ctrl.configure(rf, port_prefix))
        {
            yError() << "SuperimposeViewer::configure"
                     << "error: cannot configure the gaze controller";
            return false;
        }

        // frame transform client
        yarp::os::Property propTfClient;
        propTfClient.put("device", "FrameTransformClient");
        propTfClient.put("local", port_prefix + "/transformClient");
        propTfClient.put("remote", "/transformServer");
        tf_client = nullptr;
        bool ok_drv = drv_transform_client.open(propTfClient);
        ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;
        if (!ok_drv)
            return false;

        // configure estimate CAD
        double cam_fx;
        double cam_cx;
        double cam_fy;
        double cam_cy;
        if (robot_name == "icubSim")
        {
            cam_fx = 343.121107282;
            cam_cx = 160;
            cam_fy = 343.121107282;
            cam_cy = 120;
        }
        else if (robot_name == "icub")
        {
            gaze_ctrl.getCameraIntrinsics(eye_name,
                                          cam_fx, cam_fy,
                                          cam_cx, cam_cy);
        }

        // initialize SICAD objects
        mesh_container.emplace("object", mesh_path);

        // estimate
        est_mesh_cad = std::unique_ptr<SICAD>(new SICAD(mesh_container,
                                                        cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy,
                                                        1,
                                                        red_shaders_path,
                                                        {1.0, 0.0, 0.0, static_cast<float>(M_PI)}));
        est_mesh_cad->setBackgroundOpt(true);
        est_mesh_cad->setWireframeOpt(true);

        // ground truth
        gt_mesh_cad = std::unique_ptr<SICAD>(new SICAD(mesh_container,
                                                       cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy,
                                                       1,
                                                       green_shaders_path,
                                                       {1.0, 0.0, 0.0, static_cast<float>(M_PI)}));
        gt_mesh_cad->setBackgroundOpt(true);
        gt_mesh_cad->setWireframeOpt(true);

        // reset flags
        is_est_available = false;
        is_gt_available = false;

        return true;
    }

    double getPeriod() override
    {
        return period;
    }

    bool getFrame(yarp::sig::ImageOf<yarp::sig::PixelRgb>* &yarp_image)
    {
        // try to read image from port
        yarp_image = image_input_port.read(false);

        if (yarp_image == NULL)
            return false;

        return true;
    }

    void homogeneousToSIModelPose(const yarp::sig::Matrix &homog,
                                  Superimpose::ModelPose& model_pose)
    {
        // convert a homogeneous transformation to a Superimpose::ModelPose
        model_pose.resize(7);
        model_pose[0] = homog(0, 3);
        model_pose[1] = homog(1, 3);
        model_pose[2] = homog(2, 3);
        yarp::sig::Vector axis_angle = yarp::math::dcm2axis(homog.submatrix(0, 2, 0, 2));
        model_pose[3] = axis_angle[0];
        model_pose[4] = axis_angle[1];
        model_pose[5] = axis_angle[2];
        model_pose[6] = axis_angle[3];
    }

    bool updateModule() override
    {
        // get image from camera
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* img_in;
        if (!getFrame(img_in))
            return true;

        // get current estimate from the filter
        if (tf_client->getTransform(est_tf_target, est_tf_source, estimate))
            is_est_available = true;
        // get current ground truth
        if (tf_client->getTransform(gt_tf_target, gt_tf_source, ground_truth))
            is_gt_available = true;

        // if never received anything stops here
        if ((!is_est_available) && (!is_gt_available))
            return true;

        Superimpose::ModelPoseContainer est_pose_map;
        Superimpose::ModelPoseContainer gt_pose_map;
        Superimpose::ModelPose est_pose;
        Superimpose::ModelPose gt_pose;
        if (is_est_available)
        {
            homogeneousToSIModelPose(estimate, est_pose);
            est_pose_map.emplace("object", est_pose);
        }
        if (is_gt_available)
        {
            homogeneousToSIModelPose(ground_truth, gt_pose);
            gt_pose_map.emplace("object", gt_pose);
        }

        // get current pose of eyes
        // yarp::sig::Vector left_eye_pose;
        // yarp::sig::Vector right_eye_pose;
        // yarp::sig::Vector eye_pose;
        // head_kin.getEyesPose(left_eye_pose, right_eye_pose);
        // eye_pose = (eye_name == "left") ? left_eye_pose : right_eye_pose;
        yarp::sig::Vector eye_pos;
        yarp::sig::Vector eye_att;
        gaze_ctrl.getCameraPose(eye_name, eye_pos, eye_att);

        // prepare output image
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
        img_out = *img_in;
        cv::Mat cv_img;
        cv_img = cv::cvarrToMat(img_out.getIplImage());

        // superimpose estimate on image
        if (is_est_available)
            est_mesh_cad->superimpose(est_pose_map,
                                      eye_pos.data(),
                                      eye_att.data(),
                                      cv_img);
        if (is_gt_available)
            gt_mesh_cad->superimpose(gt_pose_map,
                                     eye_pos.data(),
                                     eye_att.data(),
                                     cv_img);
        // send image
        image_output_port.write();

        return true;
    }

    bool close() override
    {
        // close ports
        image_input_port.close();
        image_output_port.close();

        // close head kinematics
        // head_kin.close();

        // close gaze controller
        gaze_ctrl.close();

        // close transform client
        drv_transform_client.close();

        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("si_viewer_config.ini");
    rf.configure(argc,argv);

    if (!yarp.checkNetwork())
    {
        yError() << "Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    SuperimposeViewer viewer;
    return viewer.runModule(rf);
}
