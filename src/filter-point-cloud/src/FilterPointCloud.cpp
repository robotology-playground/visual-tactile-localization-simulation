/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Nicola Piga
 */

#include <cstdlib>
#include <memory>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkQuadric.h>
#include <vtkTransform.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkRadiusOutlierRemoval.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

typedef yarp::sig::PointCloud<yarp::sig::DataXYZ> PointCloudXYZ;

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

        iren->GetRenderWindow()->SetWindowName("Filter Point Cloud - Viewer");
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
class PointCloudFilter : public RFModule, RateThread
{
    Bottle outliersRemovalOptions;
    unsigned int uniform_sample;
    double random_sample;
    bool from_file;
    bool viewer_enabled;
    bool closing;

    vector<Vector> all_points,in_points,out_points,dwn_points;
    vector<vector<unsigned char>> all_colors;

    BufferedPort<PointCloudXYZ> portPointsIn;
    BufferedPort<PointCloudXYZ> portPointsOut;
    RpcServer rpc_server;
    
    unique_ptr<Points> vtk_all_points,vtk_out_points,vtk_dwn_points;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;     
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    /****************************************************************/
    void removeOutliers()
    {
        if (outliersRemovalOptions.size()>=2)
        {
            double radius=outliersRemovalOptions.get(0).asDouble();
            int neighbors=outliersRemovalOptions.get(1).asInt();

            vtkSmartPointer<vtkPoints> vtk_points=vtkSmartPointer<vtkPoints>::New();
            for (size_t i=0; i<all_points.size(); i++)
                vtk_points->InsertNextPoint(all_points[i][0],all_points[i][1],all_points[i][2]);

            vtkSmartPointer<vtkPolyData> vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
            vtk_polydata->SetPoints(vtk_points);

            vtkSmartPointer<vtkRadiusOutlierRemoval> removal=vtkSmartPointer<vtkRadiusOutlierRemoval>::New();
            removal->SetInputData(vtk_polydata);
            removal->SetRadius(radius);
            removal->SetNumberOfNeighbors(neighbors);
            removal->Update();

            // yInfo()<<"# of outliers removed / # of points ="
            //        <<removal->GetNumberOfPointsRemoved()<<"/"<<all_points.size();

            for (size_t i=0; i<all_points.size(); i++)
            {
                if (removal->GetPointMap()[i]<0)
                    out_points.push_back(all_points[i]);
                else
                    in_points.push_back(all_points[i]);
            }
        }
        else
            in_points=all_points;
    }

    /****************************************************************/
    void sampleInliers()
    {
        if (random_sample>=1.0)
        {
            unsigned int cnt=0;
            for (auto &p:in_points)
            {
                if ((cnt++%uniform_sample)==0)
                    dwn_points.push_back(p);
            }
        }
        else
        {
            set<unsigned int> idx;
            while (idx.size()<(size_t)(random_sample*in_points.size()))
            {
                unsigned int i=(unsigned int)(Rand::scalar(0.0,1.0)*in_points.size());
                if (idx.find(i)==idx.end())
                {
                    dwn_points.push_back(in_points[i]);
                    idx.insert(i);
                }
            }
        }

        yInfo()<<"# of samples / # of inliers ="<<dwn_points.size()<<"/"<<in_points.size();
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        Rand::init();

        from_file=rf.check("file");
        if (from_file)
        {
            string file=rf.find("file").asString();
            ifstream fin(file.c_str());
            if (!fin.is_open())
            {
                yError()<<"Unable to open file \""<<file<<"\"";
                return false;
            }

            Vector p(3);
            vector<unsigned int> c_(3);
            vector<unsigned char> c(3);

            string line;
            while (getline(fin,line))
            {
                istringstream iss(line);
                if (!(iss>>p[0]>>p[1]>>p[2]))
                    break;
                all_points.push_back(p);

                fill(c_.begin(),c_.end(),120);
                iss>>c_[0]>>c_[1]>>c_[2];
                c[0]=(unsigned char)c_[0];
                c[1]=(unsigned char)c_[1];
                c[2]=(unsigned char)c_[2];
                all_colors.push_back(c);
            }
        }
        else
        {
	    portPointsIn.open("/filter-point-cloud/pointcloud:i");
        }

	portPointsOut.open("/filter-point-cloud/filtered-pointcloud:o");
	rpc_server.open("/filter-point-cloud/service");
	attach(rpc_server);

        if (rf.check("remove-outliers"))
            if (const Bottle *ptr=rf.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;

        uniform_sample=(unsigned int)rf.check("uniform-sample",Value(1)).asInt();
        random_sample=rf.check("random-sample",Value(1.0)).asDouble();
        viewer_enabled=!rf.check("disable-viewer");

	if (from_file)
	{
	    removeOutliers();
	    sampleInliers();
	}

        vtk_all_points=unique_ptr<Points>(new Points(all_points,2));
        vtk_out_points=unique_ptr<Points>(new Points(out_points,4));
        vtk_dwn_points=unique_ptr<Points>(new Points(dwn_points,1));

	if (from_file)
	{
	    vtk_all_points->set_colors(all_colors);
	}
        vtk_out_points->get_actor()->GetProperty()->SetColor(1.0,0.0,0.0);
        vtk_dwn_points->get_actor()->GetProperty()->SetColor(1.0,1.0,0.0);

        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        vtk_renderer->AddActor(vtk_all_points->get_actor());
        vtk_renderer->AddActor(vtk_out_points->get_actor());
	vtk_renderer->AddActor(vtk_dwn_points->get_actor());
        vtk_renderer->SetBackground(0.1,0.2,0.2);

        vtk_axes=vtkSmartPointer<vtkAxesActor>::New();     
        vtk_widget=vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

	vector<double> bounds(6),centroid(3);
	if (from_file)
	{
	    vtk_all_points->get_polydata()->GetBounds(bounds.data());
	    for (size_t i=0; i<centroid.size(); i++)
		centroid[i]=0.5*(bounds[i<<1]+bounds[(i<<1)+1]);
	}

        vtk_camera=vtkSmartPointer<vtkCamera>::New();
	vtk_camera->SetPosition(centroid[0]+1.0,centroid[1],centroid[2]+0.5);
	vtk_camera->SetFocalPoint(centroid.data());
        vtk_camera->SetViewUp(0.0,0.0,1.0);
        vtk_renderer->SetActiveCamera(vtk_camera);

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

	start();

        if (viewer_enabled)
        {
            vtk_renderWindowInteractor->Initialize();
            vtk_renderWindowInteractor->CreateRepeatingTimer(10);

            vtk_updateCallback=vtkSmartPointer<UpdateCommand>::New();
            vtk_updateCallback->set_closing(closing);
            vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent,vtk_updateCallback);
            vtk_renderWindowInteractor->Start();
        }
        
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
	if (from_file)
	{
	    // if source is taken from file
	    // the filtered content is streamed continuosly
	    send();
	}
	else
	{
	    PointCloudXYZ *new_pc = portPointsIn.read(false);
	    if (new_pc != NULL)
	    {
		process(*new_pc);

		// if source is taken from port
		// filtered point cloud is sent only
		// when a new one is received from the source
		send();
	    }
	}
    }

    /****************************************************************/
    void process(const PointCloud<DataXYZ> &points)
    {   
        if (points.size()>0)
        {
            LockGuard lg(mutex);

            all_points.clear();
            all_colors.clear();
            in_points.clear();
            out_points.clear();
            dwn_points.clear();

            Vector p(3);
            vector<unsigned char> c(3);
            for (int i=0; i<points.size(); i++)
            {
                p[0]=points(i).x;
                p[1]=points(i).y;
                p[2]=points(i).z;
                c[0]=120;
                c[1]=120;
                c[2]=120;
                all_points.push_back(p);
                all_colors.push_back(c);
            }

            removeOutliers();
            sampleInliers();
            
            vtk_all_points->set_points(all_points);
            vtk_all_points->set_colors(all_colors);
            vtk_out_points->set_points(out_points);
            vtk_dwn_points->set_points(dwn_points);
        }
    }

    /****************************************************************/
    void send()
    {
	if (dwn_points.size() <= 0)
	    return;

	LockGuard lg(mutex);

	PointCloudXYZ &pc = portPointsOut.prepare();
	pc.clear();

	for (size_t i=0; i<dwn_points.size(); i++)
	{
	    Vector &v = dwn_points[i];
	    DataXYZ point;
	    point.x = v[0];
	    point.y = v[1];
	    point.z = v[2];
	    pc.push_back(point);
	}

	portPointsOut.write();
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        LockGuard lg(mutex);

        bool ok=false;
        if (command.check("remove-outliers"))
        {
            if (const Bottle *ptr=command.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;
            ok=true;
        }

        if (command.check("uniform-sample"))
        {
            uniform_sample=(unsigned int)command.find("uniform-sample").asInt();
            ok=true;
        }

        if (command.check("random-sample"))
        {
            random_sample=command.find("random-sample").asDouble();
            ok=true;
        }

        reply.addVocab(Vocab::encode(ok?"ack":"nack"));
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
	// stop the io processing thread
	stop();

	portPointsIn.close();
	portPointsOut.close();

        return true;
    }

public:
    /****************************************************************/
    PointCloudFilter() : closing(false), RateThread(10) { }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.configure(argc,argv);

    if (!rf.check("file"))
    {
        if (!yarp.checkNetwork())
        {
            yError()<<"Unable to find Yarp server!";
            return EXIT_FAILURE;
        }
    }

    PointCloudFilter finder;
    return finder.runModule(rf);
}

