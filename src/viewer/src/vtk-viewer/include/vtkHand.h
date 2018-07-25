/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

#ifndef VTK_HAND_H
#define VTK_HAND_H

// vtk
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>

// std
#include <unordered_map>

//
#include "vtkFinger.h"

typedef std::unordered_map<std::string, std::vector<yarp::sig::Matrix> > fingersLinks;

class vtkHand
{
protected:
    // fingers names
    std::vector<std::string> fingers_names;

    // vtk fingers
    std::unordered_map<std::string, std::unique_ptr<vtkFinger> > vtk_fingers;

public:
    vtkHand(const bool &show_axes);
    void attach_to_renderer(vtkSmartPointer<vtkRenderer> &vtk_renderer);
    void set_fingers_links(const fingersLinks &links);
};

#endif
