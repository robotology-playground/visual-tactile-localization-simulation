/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file vtk3DAxes.cpp
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

// yarp
#include <yarp/math/Math.h>

// vtk
#include <vtkCellData.h>

#include <vtk3DAxes.h>

using namespace yarp::math;

vtk3DAxes::vtk3DAxes()
{
    // resize axes lengths
    axes_lengths.resize(3, 0.0);

    // create polydata
    vtk_polydata = vtkSmartPointer<vtkPolyData>::New();

    // create points
    double placeholder[3] = {0.0, 0.0, 0.0};
    vtk_points = vtkSmartPointer<vtkPoints>::New();
    for (size_t i=0; i<4; i++)
        vtk_points->InsertNextPoint(placeholder);

    // add points to polydata
    vtk_polydata->SetPoints(vtk_points);

    // create axes
    for(size_t i=0; i<3; i++)
    {
        vtk_lines.push_back(vtkSmartPointer<vtkLine>::New());
        vtk_lines[i]->GetPointIds()->SetId(0, 0);
        vtk_lines[i]->GetPointIds()->SetId(1, i + 1);
    }

    // add axes to a cell array
    vtk_cells = vtkSmartPointer<vtkCellArray>::New();
    for (size_t i=0; i<3; i++)
        vtk_cells->InsertNextCell(vtk_lines[i]);

    // add axes to polydata
    vtk_polydata->SetLines(vtk_cells);

    // create color for each axis
    unsigned char x_color[3] = {255, 0, 0}; // red
    unsigned char y_color[3] = {0, 255, 0}; // green
    unsigned char z_color[3] = {0, 0, 255}; // blue

    // store colors
    vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtk_colors->SetNumberOfComponents(3);
    vtk_colors->InsertNextTypedTuple(x_color);
    vtk_colors->InsertNextTypedTuple(y_color);
    vtk_colors->InsertNextTypedTuple(z_color);

    // set colors
    vtk_polydata->GetCellData()->SetScalars(vtk_colors);

    // instantiate mapper
    vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtk_mapper->SetInputData(vtk_polydata);

    // instantiate mapper
    vtk_poly_actor = vtkSmartPointer<vtkActor>::New();
    vtk_poly_actor->SetMapper(vtk_mapper);
}

void vtk3DAxes::set_axes_lengths(const double &x_len,
				 const double &y_len,
				 const double &z_len)
{
    axes_lengths[0] = x_len;
    axes_lengths[1] = y_len;
    axes_lengths[2] = z_len;
}

void vtk3DAxes::attach_to_renderer(vtkSmartPointer<vtkRenderer> &vtk_renderer)
{
    vtk_renderer->AddActor(vtk_poly_actor);
}

void vtk3DAxes::set_frame(const yarp::sig::Matrix &pose)
{
    // reset coordinates
    vtk_points->Reset();

    // set origin
    const yarp::sig::Vector &origin = pose.getCol(3).subVector(0, 2);
    vtk_points->InsertNextPoint(origin.data());

    // set tips of the axes
    for (size_t i=0; i<3; i++)
    {
	// get axis direction
	yarp::sig::Vector direction;
	direction = pose.getCol(i).subVector(0, 2);

	// normalize direction
	direction = direction / yarp::math::norm(direction);

	// evaluate scaled tip of the axis
	yarp::sig::Vector tip = origin + direction * axes_lengths[i];

	vtk_points->InsertNextPoint(tip.data());
    }
}
