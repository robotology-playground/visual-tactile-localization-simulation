/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkCubeSource.h>
#include <vtkHand.h>
#include <handKinematics.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

typedef yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> PointCloudXYZRGBA;

Mutex mutex;

/****************************************************************/
class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:
    /****************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /****************************************************************/
    static UpdateCommand *New()
    {
        return new UpdateCommand;
    }

    /****************************************************************/
    UpdateCommand() : closing(nullptr) { }

    /****************************************************************/
    void set_closing(const bool &closing)
    {
        this->closing=&closing;
    }

    /****************************************************************/
    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                 void *vtkNotUsed(callData))
    {
        LockGuard lg(mutex);
        vtkRenderWindowInteractor* iren=static_cast<vtkRenderWindowInteractor*>(caller);
        if (closing!=nullptr)
        {
            if (*closing)
            {
                iren->GetRenderWindow()->Finalize();
                iren->TerminateApp();
                return;
            }
        }

        iren->GetRenderWindow()->SetWindowName("View Filtering");
        iren->Render();
    }
};

/****************************************************************/
class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    /****************************************************************/
    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};


/****************************************************************/
class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    /****************************************************************/
    Points(const vector<Vector> &points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }

    /****************************************************************/
    void set_points(const vector<Vector> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
    bool set_colors(const vector<vector<unsigned char>> &colors)
    {
        if (colors.size()==vtk_points->GetNumberOfPoints())
        {
            vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
            vtk_colors->SetNumberOfComponents(3);
            for (size_t i=0; i<colors.size(); i++)
                vtk_colors->InsertNextTypedTuple(colors[i].data());

            vtk_polydata->GetPointData()->SetScalars(vtk_colors);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};

/****************************************************************/
class Cube : public Object
{
protected:
    vtkSmartPointer<vtkCubeSource> vtk_cube_source;
    vtkSmartPointer<vtkTransform> vtk_transform;
    double width;
    double depth;
    double height;

public:
    /****************************************************************/
    Cube(const double &opacity)
    {
        // create cube
        vtk_cube_source = vtkSmartPointer<vtkCubeSource>::New();

        // create mapper
        vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_cube_source->GetOutputPort());

        // create actor
        vtk_actor = vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);

        // set opacity
        vtk_actor->GetProperty()->SetOpacity(opacity);
    }

    /****************************************************************/
    void set_color(const double &r, const double &g, const double &b)
    {
        vtk_actor->GetProperty()->SetColor(r, g, b);
    }

    /****************************************************************/
    void set_sizes(const double &width, const double &depth, const double &height)
    {
        this->width = width;
        this->depth = depth;
        this->height = height;
        vtk_cube_source->SetXLength(width);
        vtk_cube_source->SetYLength(depth);
        vtk_cube_source->SetZLength(height);
    }

    /****************************************************************/
    void set_pose(const yarp::sig::Vector &pos, const yarp::sig::Matrix &att)
    {
        // create a new transform
        vtk_transform = vtkSmartPointer<vtkTransform>::New();

        // set translation
        vtk_transform->Translate(pos.data());

        // set rotation
        yarp::sig::Vector axis_angle(4);
        axis_angle = yarp::math::dcm2axis(att);
        vtk_transform->RotateWXYZ(axis_angle[3] * 180 / M_PI,
                                  axis_angle[0], axis_angle[1], axis_angle[2]);
        // apply transform
        vtk_actor->SetUserTransform(vtk_transform);
    }
};

/****************************************************************/
class Viewer : public RFModule, RateThread
{
    bool closing;

    vector<Vector> all_points;
    vector<vector<unsigned char>> all_colors;

    BufferedPort<PointCloudXYZRGBA> portPointsIn;

    RpcServer rpc_server;

    IFrameTransform* tf_client;
    PolyDriver drv_transform_client;
    std::string est_source_name;
    std::string est_target_name;
    std::string aux_est_source_name;
    std::string aux_est_target_name;
    std::string gt_source_name;
    std::string gt_target_name;

    unique_ptr<Points> vtk_all_points;
    // cube for the estimate of the pose
    unique_ptr<Cube> vtk_cube_est;
    // cube for the auxiliary estimate of the pose
    unique_ptr<Cube> vtk_cube_aux_est;
    // cube for the ground truth
    unique_ptr<Cube> vtk_cube_gt;

    // hands kinematics
    std::unique_ptr<handKinematics> hand_kin;

    // vtk hands
    std::unique_ptr<vtkHand> vtk_hand;

    // use analogs and use analogs bounds flags
    bool use_analogs;
    bool use_analogs_bounds;

    // flags relative to visualization of the hand
    bool enable_hand;
    bool enable_hand_axes;
    std::string hand_name;

    // flags relative to point cloud visualization
    bool enable_point_cloud;

    // flags relative to mesh visualization
    bool enable_estimate;
    bool enable_aux_estimate;
    bool enable_ground_truth;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    bool loadListDouble(yarp::os::ResourceFinder &rf,
                        const std::string &key,
                        const int &size,
                        yarp::sig::Vector &list)
    {
        if (rf.find(key).isNull())
            return false;

        yarp::os::Bottle* b = rf.find(key).asList();
        if (b == nullptr)
            return false;

        if (b->size() != size)
            return false;

        list.resize(size);
        for (size_t i=0; i<b->size(); i++)
        {
            yarp::os::Value item_v = b->get(i);
            if (item_v.isNull())
                return false;

            if (!item_v.isDouble())
            {
                list.clear();
                return false;
            }

            list[i] = item_v.asDouble();
        }
        return true;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {

        rpc_server.open("/view-filtering/rpc");
        attach(rpc_server);

        yarp::os::Property propTfClient;
        propTfClient.put("device", "transformClient");
        propTfClient.put("local", "/view-filtering/transformClient");
        propTfClient.put("remote", "/transformServer");
        tf_client = nullptr;
        bool ok_drv = drv_transform_client.open(propTfClient);
        ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;
        if (!ok_drv)
            return false;

        // get robot name
        std::string robot_name;
        robot_name = "icub";
        if (!rf.find("robotName").isNull())
            robot_name = rf.find("robotName").asString();

        // get show point cloud flag
        enable_point_cloud = false;
        if (!rf.find("showPointCloud").isNull())
            enable_point_cloud = rf.find("showPointCloud").asBool();

        if (enable_point_cloud)
        {
            // open point cloud port if required
            portPointsIn.open("/view-filtering/pointcloud:i");

            // instantiate point cloud if required
            vtk_all_points=unique_ptr<Points>(new Points(all_points,2));
        }

        // get the flags relative to hand visualization
        enable_hand = false;
        if (!rf.find("showHand").isNull())
            enable_hand = rf.find("showHand").asBool();
        if (enable_hand)
        {
            enable_hand_axes = false;
            if (!rf.find("showHandAxes").isNull())
                enable_hand_axes = rf.find("showHandAxes").asBool();
            if (!rf.find("handName").isNull())
                hand_name = rf.find("handName").asString();
            if ((hand_name != "right") && (hand_name != "left"))
            {
                yInfo() << "VtkViewer::configure"
                        << "error: a valid hand name should be specified";
                return false;
            }

            // get the flags useAnalogs and useAnalogsBounds
            use_analogs = false;
            use_analogs_bounds = false;
            if (!rf.find("useAnalogs").isNull())
                use_analogs = rf.find("useAnalogs").asBool();
            if (!rf.find("useAnalogsBounds").isNull())
                use_analogs_bounds = rf.find("useAnalogsBounds").asBool();

            hand_kin = std::unique_ptr<handKinematics>(new handKinematics());
            if (!hand_kin->configure(robot_name, hand_name,
                                     use_analogs, use_analogs_bounds))
                return false;

            // vtk hands
            vtk_hand = std::unique_ptr<vtkHand>(new vtkHand(enable_hand_axes));
        }

        // get the flags relative to mesh visualization
        enable_estimate = false;
        enable_aux_estimate = false;
        enable_ground_truth = false;

        if (!rf.find("showEstimate").isNull())
            enable_estimate = rf.find("showEstimate").asBool();
        if (!rf.find("showAuxEstimate").isNull())
            enable_aux_estimate = rf.find("showAuxEstimate").asBool();
        if (!rf.find("showGroundTruth").isNull())
            enable_ground_truth = rf.find("showGroundTruth").asBool();

        // get the size of the object
        if (rf.find("objName").isNull())
        {
            yInfo() << "VtkViewer::configure"
                    << "error: cannot find objName in the configuration file";
            return false;
        }
        std::string obj_name = rf.find("objName").asString();
        yarp::os::ResourceFinder rf_obj;
        rf_obj = rf.findNestedResourceFinder(obj_name.c_str());
        if (rf_obj.find("boxSize").isNull())
        {
            yInfo() << "VtkViewer::configure"
                    << "error: cannot find objSize within group"
                    << "[" << obj_name << "] in configuration file";
            return false;
        }
        yarp::sig::Vector obj_size;
        if (!loadListDouble(rf_obj, "boxSize", 3, obj_size))
        {
            yInfo() << "VtkViewer::configure"
                    << "error: cannot read objSize within group"
                    << "[" << obj_name << "] in configuration file";
            return false;
        }

        // get names of source and target frames
        // to be used with the FrameTransform client
        est_source_name = "/iCub/frame";
        aux_est_source_name = "/iCub/frame";
        gt_source_name = "/iCub/frame";
        est_target_name = "/estimate/frame";
        aux_est_target_name = "/estimate/aux/frame";
        gt_target_name = "/ground_truth/frame";

        if (!rf.find("estimateSourceFrame").isNull())
            est_source_name = rf.find("estimateSourceFrame").asString();
        if (!rf.find("estimateTargetFrame").isNull())
            est_target_name = rf.find("estimateTargetFrame").asString();
        if (!rf.find("auxEstimateSourceFrame").isNull())
            aux_est_source_name = rf.find("auxEstimateSourceFrame").asString();
        if (!rf.find("auxEstimateTargetFrame").isNull())
            aux_est_target_name = rf.find("auxEstimateTargetFrame").asString();
        if (!rf.find("groundTruthSourceFrame").isNull())
            gt_source_name = rf.find("groundTruthSourceFrame").asString();
        if (!rf.find("groundTruthTargetFrame").isNull())
            gt_target_name = rf.find("groundTruthTargetFrame").asString();

        vtk_cube_est=unique_ptr<Cube>(new Cube(0.6));
        vtk_cube_aux_est=unique_ptr<Cube>(new Cube(0.4));
        vtk_cube_gt=unique_ptr<Cube>(new Cube(0.2));

        vtk_cube_est->set_sizes(obj_size[0], obj_size[1], obj_size[2]);
        vtk_cube_aux_est->set_sizes(obj_size[0], obj_size[1], obj_size[2]);
        vtk_cube_gt->set_sizes(obj_size[0], obj_size[1], obj_size[2]);

        vtk_cube_est->set_color(1.0, 0.0, 0.0);
        vtk_cube_aux_est->set_color(0.0, 0.0, 1.0);
        vtk_cube_gt->set_color(0.0, 1.0, 0.0);

        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        if (enable_estimate)
            vtk_renderer->AddActor(vtk_cube_est->get_actor());
        if (enable_aux_estimate)
            vtk_renderer->AddActor(vtk_cube_aux_est->get_actor());
        if (enable_ground_truth)
            vtk_renderer->AddActor(vtk_cube_gt->get_actor());
        if (enable_point_cloud)
        {
            vtk_renderer->AddActor(vtk_all_points->get_actor());
        }
        if (enable_hand)
        {
            vtk_hand->attach_to_renderer(vtk_renderer);
        }
        vtk_renderer->SetBackground(0.1,0.2,0.2);

        vtk_axes=vtkSmartPointer<vtkAxesActor>::New();
        vtk_widget=vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

        vtk_camera=vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(1.0,0.0,0.5);
        vtk_camera->SetViewUp(0.0,0.0,1.0);
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        // start IO processing thread
        start();

        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(10);

        vtk_updateCallback=vtkSmartPointer<UpdateCommand>::New();
        vtk_updateCallback->set_closing(closing);
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent,vtk_updateCallback);
        vtk_renderWindowInteractor->Start();
        yInfo() << "here!";

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 1.0;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return true;
    }

    /****************************************************************/
    void run() override
    {
        // point cloud
        if (enable_point_cloud)
        {
            PointCloudXYZRGBA *new_pc = portPointsIn.read(false);
            if (new_pc != NULL)
            {
                process(*new_pc);
            }
        }

        // hand kinematics
        if (enable_hand)
        {
            LockGuard lg(mutex);

            fingersLinks fingers_links;
            if (hand_kin->getFingersLinks(fingers_links))
                vtk_hand->set_fingers_links(fingers_links);
        }

        // estimate and ground truth
        updateView();
    }

    /****************************************************************/
    void process(const PointCloudXYZRGBA &points)
    {
        if (points.size()>0)
        {
            LockGuard lg(mutex);

            all_points.clear();
            all_colors.clear();

            Vector p(3);
            vector<unsigned char> c(3);
            for (int i=0; i<points.size(); i++)
            {
                p[0]=points(i).x;
                p[1]=points(i).y;
                p[2]=points(i).z;
                c[0]=points(i).r;
                c[1]=points(i).g;
                c[2]=points(i).b;
                all_points.push_back(p);
                all_colors.push_back(c);
            }

            vtk_all_points->set_points(all_points);
            vtk_all_points->set_colors(all_colors);
        }
    }

    /****************************************************************/
    void updateView()
    {
        LockGuard lg(mutex);

        yarp::sig::Matrix estimate;
        yarp::sig::Matrix aux_estimate;
        yarp::sig::Matrix ground_truth;

        // try to get the current estimate of the filter
        if (enable_estimate)
        {
            if (tf_client->getTransform(est_target_name, est_source_name, estimate))
            {
                vtk_cube_est->set_pose(estimate.getCol(3).subVector(0, 2),
                                       estimate.submatrix(0, 2, 0, 2));
            }
        }

        // try to get the current auxiliary estimate of the filter
        if (enable_aux_estimate)
        {
            if (tf_client->getTransform(aux_est_target_name, aux_est_source_name, aux_estimate))
            {
                vtk_cube_aux_est->set_pose(aux_estimate.getCol(3).subVector(0, 2),
                                           aux_estimate.submatrix(0, 2, 0, 2));
            }
        }

        // try to get the current ground truth
        if (enable_ground_truth)
        {
            if (tf_client->getTransform(gt_target_name, gt_source_name, ground_truth))
            {
                vtk_cube_gt->set_pose(ground_truth.getCol(3).subVector(0, 2),
                                      ground_truth.submatrix(0, 2, 0, 2));
            }
        }
    }

    void calibrateHand()
    {
        hand_kin->calibrate();
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        LockGuard lg(mutex);

        std::string cmd = command.get(0).asString();
        if (cmd == "help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- calibrate_hand");
            reply.addString("- help");
            reply.addString("- quit");
        }
        else if (cmd == "calibrate_hand")
        {
            calibrateHand();

            reply.addString("- calibration ok");
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        closing=true;
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        // close the IO processing thread
        stop();

        portPointsIn.close();

        return true;
    }

public:
    /****************************************************************/
    Viewer() : closing(false), RateThread(33) { }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.setDefaultConfigFile("vtk_viewer_config.ini");
    rf.configure(argc,argv);

    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    Viewer viewer;
    return viewer.runModule(rf);
}
