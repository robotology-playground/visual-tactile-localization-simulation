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

// opencv
#include <opencv2/opencv.hpp>

// superimpose
#include <SuperimposeMesh/Superimpose.h>
#include <SuperimposeMesh/SICAD.h>

//
#include <HeadKinematics.h>
#include <cstdlib>

class SuperimposeViewer : public yarp::os::RFModule
{
private:
    // head kinematics
    headKinematics head_kin;

    // camera port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> image_input_port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> image_output_port;

    // selected eye name
    std::string eye_name;

    // object mesh
    SICAD::ModelPathContainer mesh_container;
    std::unique_ptr<SICAD> mesh_cad;

    // period
    double period;

    // frame transform client
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;
    std::string gt_tf_source;
    std::string gt_tf_target;
    std::string est_tf_source;
    std::string est_tf_target;

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

        std::string shaders_path = ".";
        if (!rf.find("shadersPath").isNull())
            shaders_path = rf.findPath("shadersPath");

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
        if (!head_kin.configure(robot_name, port_prefix))
            return false;

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
        float cam_fx;
        float cam_cx;
        float cam_fy;
        float cam_cy;
        if (robot_name == "icubSim")
        {
            cam_fx = 343.121107282;
            cam_cx = 160;
            cam_fy = 343.121107282;
            cam_cy = 120;
        }
        else if (robot_name == "icub")
        {
            if (eye_name == "left")
            {
                cam_fx = 219.057;
                cam_cx = 174.742;
                cam_fy = 219.028;
                cam_cy = 102.874;
            }
            else if (eye_name == "right")
            {
                cam_fx = 225.904;
                cam_cx = 157.858;
                cam_fy = 227.041;
                cam_cy = 113.51;
            }
        }
        mesh_container.emplace("object", mesh_path);
        mesh_cad = std::unique_ptr<SICAD>(new SICAD(mesh_container,
                                                    cam_width, cam_height, cam_fx, cam_fy, cam_cx, cam_cy,
                                                    1,
                                                    shaders_path,
                                                    {1.0, 0.0, 0.0, static_cast<float>(M_PI)}));
        mesh_cad->setBackgroundOpt(true);
        mesh_cad->setWireframeOpt(true);

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

    bool updateModule() override
    {
        // get image from camera
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* img_in;
        if (!getFrame(img_in))
            return true;

        // get current estimate from the filter
        yarp::sig::Matrix estimate;
        if (!tf_client->getTransform(est_tf_target, est_tf_source, estimate))
            return true;

        // store estimate as a Superimpose::ModelPoseContainer
        Superimpose::ModelPose obj_pose(7);
        obj_pose[0] = estimate(0, 3);
        obj_pose[1] = estimate(1, 3);
        obj_pose[2] = estimate(2, 3);
        yarp::sig::Vector axis_angle = yarp::math::dcm2axis(estimate.submatrix(0, 2, 0, 2));
        obj_pose[3] = axis_angle[0];
        obj_pose[4] = axis_angle[1];
        obj_pose[5] = axis_angle[2];
        obj_pose[6] = axis_angle[3];
        Superimpose::ModelPoseContainer objpose_map;
        objpose_map.emplace("object", obj_pose);

        // get current pose of eyes
        yarp::sig::Vector left_eye_pose;
        yarp::sig::Vector right_eye_pose;
        yarp::sig::Vector eye_pose;
        head_kin.getEyesPose(left_eye_pose, right_eye_pose);
        eye_pose = (eye_name == "left") ? left_eye_pose : right_eye_pose;

        // prepare output image
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
        img_out = *img_in;
        cv::Mat cv_img;
        cv_img = cv::cvarrToMat(img_out.getIplImage());

        // superimpose estimate on image
        mesh_cad->superimpose(objpose_map,
                              eye_pose.subVector(0, 2).data(),
                              eye_pose.subVector(3, 6).data(),
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
        head_kin.close();

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
