/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file vtkFinger.cpp
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

// vtk
#include<vtkCaptionActor2D.h>
#include<vtkTextProperty.h>

#include <vtkFinger.h>

vtkFinger::vtkFinger(const bool& show_axes)
{
    // store boolean show_axes
    this->show_axes = show_axes;

    // create points
    double placeholder[3] = {0.0, 0.0, 0.0};
    vtk_coords = vtkSmartPointer<vtkPoints>::New();
    for (size_t i=0; i<4; i++)
        vtk_coords->InsertNextPoint(placeholder);

    // create polyline
    vtk_poly = vtkSmartPointer<vtkPolyLine>::New();
    vtk_poly->GetPointIds()->SetNumberOfIds(4);
    for(size_t i=0; i<4; i++)
	vtk_poly->GetPointIds()->SetId(i,i);

    // create cells
    vtk_cells = vtkSmartPointer<vtkCellArray>::New();
    vtk_cells->InsertNextCell(vtk_poly);

    // create cell array to store lines of polyline
    vtk_polydata = vtkSmartPointer<vtkPolyData>::New();

    // add points to polydata
    vtk_polydata->SetPoints(vtk_coords);

    // add lines to polydata
    vtk_polydata->SetLines(vtk_cells);

    // setup mapper
    vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtk_mapper->SetInputData(vtk_polydata);

    // setup actor
    vtk_poly_actor = vtkSmartPointer<vtkActor>::New();
    vtk_poly_actor->SetMapper(vtk_mapper);

    if (show_axes)
    {
        // setup axes for links reference frames
        // one for representing the axes of H0
        // and the other for the axes fixed to each link
        for (size_t i=0; i<5; i++)
        {
            vtk_frames.push_back(std::unique_ptr<vtk3DAxes>(new vtk3DAxes()));
            vtk_frames[i]->set_axes_lengths(0.01, 0.01, 0.01);
        }
    }
}

void vtkFinger::attach_to_renderer(vtkSmartPointer<vtkRenderer> &vtk_renderer)
{
    vtk_renderer->AddActor(vtk_poly_actor);
    if (show_axes)
    {
        for (size_t i=0; i<5; i++)
            vtk_frames[i]->attach_to_renderer(vtk_renderer);
    }
}

bool vtkFinger::set_links(const std::vector<yarp::sig::Matrix> &links)
{
    if (links.size() != 5)
	return false;

    // reset coordinates
    vtk_coords->Reset();

    for (size_t i=0; i<links.size(); i++)
    {
        if (show_axes)
        {
            // update reference frames
            vtk_frames[i]->set_frame(links[i]);
        }

	if (i != 0)
	{
	    // update origin of frames
	    yarp::sig::Vector frame_origin = links[i].getCol(3).subVector(0, 2);
	    vtk_coords->InsertNextPoint(frame_origin.data());
	}
    }
}
