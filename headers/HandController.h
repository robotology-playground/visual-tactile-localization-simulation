/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef HAND_CONTROLLER_H
#define HAND_CONTROLLER_H

// yarp
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/api.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/dev/IControlMode2.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>

// std
#include <string>
#include <vector>
#include <unordered_map>

#include <headers/FingerController.h>

class HandController
{
private:
    // hand name
    std::string hand_name;
    
    // driver
    yarp::dev::PolyDriver drv_arm;

    // views
    yarp::dev::IEncoders *ienc_arm;
    yarp::dev::IControlMode2 *imod_arm;

    yarp::dev::IPositionControl2 *ipos_arm;
    yarp::dev::IVelocityControl2 *ivel_arm;

    // fingers
    std::vector<std::string> fingers_names;
    std::unordered_map<std::string, FingerController> fingers;
    std::unordered_map<std::string, bool> contacts;
    
public:
    /*
     * Configure the hand controller.
     * @param hand_name is the name of the hand
     * @return true/false con success/failure
     */
    bool configure(const std::string &hand_name);

    /*
     * Close all the finger controllers and
     * close the drivers.
     * @return true/false con success/failure
     */
    bool close();

    /*
     * Get joints angles associated to the chain of the whole arm.
     * @param joints a yarp::sig::Vector containing the extracted joints
     * @return true/false con success/failure
     */
    bool getJoints(yarp::sig::Vector &joints);

    /*
     * Reset the internal state used within the method moveFingersUntilContact.
     * @return true/false con success/failure
     */
    void resetFingersContacts();

    /*
     * Move the fingers forward until a contact is detected. Once a contact
     * is detected for a given finger the finger stops moving forward.
     *
     * To be used in "streaming" mode by providing tactile feedback using the
     * input number_contacts. 
     *
     * @param speed the desired speed in the positive y direction 
     *              of the root frame of the finger
     * @param number_contacts the current number of contacts for each finger
     * @param done true if all fingers reached contact, false otherwise
     * @return true/false con success/failure
     */
    bool moveFingersUntilContact(const double &speed,
				 const std::unordered_map<std::string, int> &number_contacts,
				 bool &done);
    
    /* Restore the initial configuration of all the fingers.
     * @return true/false con success/failure
     */
    bool restoreFingersPosition();

    /*
     * Stop all the fingers.
     * @return true/false con success/failure
     */
    bool stopFingers();
    
};

class RightHandController : public HandController
{
public:
    bool configure();
};

class LeftHandController : public HandController
{
public:
    bool configure();
};

#endif
