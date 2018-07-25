/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file vtk3DAxes.h
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

#ifndef VTK_3D_AXES_H
#define VTK_3D_AXES_H

// vtk
#include <vtkSmartPointer.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkUnsignedCharArray.h>
#include <vtkRenderer.h>

// std
#include <vector>

// yarp
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

class vtk3DAxes
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkCellArray> vtk_cells;
    vtkSmartPointer<vtkPoints> vtk_points;
    std::vector<vtkSmartPointer<vtkLine>> vtk_lines;
    vtkSmartPointer<vtkActor> vtk_poly_actor;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;

    yarp::sig::Vector axes_lengths;

public:
    vtk3DAxes();
    void set_axes_lengths(const double &x_len,
			  const double &y_len,
			  const double &z_len);
    void attach_to_renderer(vtkSmartPointer<vtkRenderer> &vtk_renderer);
    void set_frame(const yarp::sig::Matrix &pose);
};

#endif
