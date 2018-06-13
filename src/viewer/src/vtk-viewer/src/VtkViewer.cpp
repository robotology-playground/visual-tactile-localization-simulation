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
    Cube()
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
	vtk_actor->GetProperty()->SetOpacity(0.25);
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
    
    unique_ptr<Points> vtk_all_points;
    unique_ptr<Cube> vtk_cube;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;     
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {

	portPointsIn.open("/view-filtering/pointcloud:i");

	rpc_server.open("/view-filtering/rpc");
	attach(rpc_server);

	yarp::os::Property propTfClient;
	propTfClient.put("device", "FrameTransformClient");
	propTfClient.put("local", "/view-filtering/transformClient");
	propTfClient.put("remote", "/transformServer");
	tf_client = nullptr;
	bool ok_drv = drv_transform_client.open(propTfClient);
	ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;
	if (!ok_drv)
	    return false;

        vtk_all_points=unique_ptr<Points>(new Points(all_points,2));

	vtk_cube=unique_ptr<Cube>(new Cube());
	//vtk_cube->set_sizes(0.23, 0.178, 0.04);
    vtk_cube->set_sizes(0.205, 0.13, 0.055);

        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        vtk_renderer->AddActor(vtk_all_points->get_actor());
	vtk_renderer->AddActor(vtk_cube->get_actor());
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
	PointCloudXYZRGBA *new_pc = portPointsIn.read(false);
	if (new_pc != NULL)
	{
	    process(*new_pc);
	}

	update_estimate_view();
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
    void update_estimate_view()
    {
	LockGuard lg(mutex);

	// get current estimate from the filter
	std::string source = "/iCub/frame";
	std::string target = "/estimate/frame";
	yarp::sig::Matrix estimate;
	if (!tf_client->getTransform(target, source, estimate))
	    return;

	// update the view of the estimate
	vtk_cube->set_pose(estimate.getCol(3).subVector(0, 2),
			   estimate.submatrix(0, 2, 0, 2));
    }

    /****************************************************************/
    void center_camera()
    {
        vector<double> bounds(6),centroid(3);
        vtk_all_points->get_polydata()->GetBounds(bounds.data());
        for (size_t i=0; i<centroid.size(); i++)
            centroid[i]=0.5*(bounds[i<<1]+bounds[(i<<1)+1]);

        vtk_camera->SetPosition(centroid[0]+1.0,centroid[1],centroid[2]+0.5);
        vtk_camera->SetFocalPoint(centroid.data());
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
	    reply.addString("- center-camera");
	    reply.addString("- help");
	    reply.addString("- quit");
	}
        else if (cmd == "center-camera")
        {
	    center_camera();
	    reply.addString("Camera centered.");
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
    rf.configure(argc,argv);

    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    Viewer viewer;
    return viewer.runModule(rf);
}
