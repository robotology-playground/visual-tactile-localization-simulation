/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file vtkHand.cpp
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

#include <vtkHand.h>

vtkHand::vtkHand(const bool& show_axes)
{
    // populate list of fingers names
    fingers_names.push_back("thumb");
    fingers_names.push_back("index");
    fingers_names.push_back("middle");
    fingers_names.push_back("ring");
    fingers_names.push_back("little");

    // vtk fingers
    for (std::string &name : fingers_names)
	vtk_fingers[name] = std::unique_ptr<vtkFinger>(new vtkFinger(show_axes));
}

void vtkHand::attach_to_renderer(vtkSmartPointer<vtkRenderer> &vtk_renderer)
{
    for (std::string &name : fingers_names)
	vtk_fingers[name]->attach_to_renderer(vtk_renderer);
}

void vtkHand::set_fingers_links(const fingersLinks &links)
{
    for (std::string &name : fingers_names)
	vtk_fingers[name]->set_links(links.at(name));
}



