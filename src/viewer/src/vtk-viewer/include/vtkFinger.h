/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file vtkFinger.h
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

#ifndef VTK_FINGER_H
#define VTK_FINGER_H

// vtk
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkRenderer.h>

// yarp
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

// std
#include <vector>

//
#include "vtk3DAxes.h"

class vtkFinger
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkPoints> vtk_coords;
    vtkSmartPointer<vtkPolyLine> vtk_poly;
    vtkSmartPointer<vtkCellArray> vtk_cells;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkActor> vtk_poly_actor;
    std::vector<std::unique_ptr<vtk3DAxes>> vtk_frames;

    // whether to show axes or not
    bool show_axes;
public:
    vtkFinger(const bool& show_axes);
    void attach_to_renderer(vtkSmartPointer<vtkRenderer> &vtk_renderer);
    bool set_links(const std::vector<yarp::sig::Matrix> &links);
};

#endif
