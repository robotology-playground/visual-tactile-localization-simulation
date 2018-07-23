/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

// yarp
#include <yarp/os/Property.h>
#include <yarp/math/Math.h>

#include "HandController.h"

#include <cmath>

using namespace yarp::math;

bool HandController::configure(yarp::os::ResourceFinder &rf,
                               const std::string &robot_name,
                               const std::string &hand_name,
                               const bool &use_analogs)
{
    // store name of the hand
    this->hand_name = hand_name;

    // store use_analogs
    this->use_analogs = use_analogs;

    // prepare properties for the Encoders
    yarp::os::Property prop;
    prop.put("device", "remote_controlboard");
    prop.put("remote", "/" + robot_name + "/" + hand_name + "_arm");
    prop.put("local", "/hand_controller/" + hand_name + "_arm/encoders");
    bool ok = drv_arm.open(prop);
    if (!ok)
    {
        yError() << "HandController::configure"
                 << "Error: unable to open the Remote Control Board driver";
        return false;
    }

    if (use_analogs)
    {
	prop.put("device", "analogsensorclient");
	prop.put("remote", "/" + robot_name + "/" + hand_name + "_hand/analog:o");
	prop.put("local", "/hand_controller/" + hand_name + "_hand/analogs");
	ok = drv_analog.open(prop);
	if (!ok)
	{
	    yError() << "HandController::configure error:"
		     << "unable to open the Analog Sensor Client driver";
	    return false;
	}
    }

    // try to retrieve the views
    ok = drv_arm.view(ienc_arm);
    if (!ok || ienc_arm == 0)
    {
        yError() << "HandController:configure"
                 << "Error: unable to retrieve the Encoders view";
        return false;
    }

    ok = drv_arm.view(ipos_arm);
    if (!ok || ipos_arm == 0)
    {
        yError() << "HandController:configure"
                 << "Error: unable to retrieve the PositionControl view";
        return false;
    }

    ok = drv_arm.view(ivel_arm);
    if (!ok || ivel_arm == 0)
    {
        yError() << "HandController:configure"
                 << "Error: unable to retrieve the VelocityControl view";
        return false;
    }

    ok = drv_arm.view(imod_arm);
    if (!ok || imod_arm == 0)
    {
        yError() << "HandController:configure"
                 << "Error: unable to retrieve the ControlMode view";
        return false;
    }

    if (use_analogs)
    {
        ok = drv_analog.view(ianalog_arm);
        if (!ok || ianalog_arm == 0)
        {
            yError() << "HandController:configure"
                     << "Error: unable to retrieve the IAnalogSensor view";
            return false;
        }
    }

    ok = drv_arm.view(ilimits_arm);
    if (!ok || ilimits_arm == 0)
    {
        yError() << "HandController::configure"
                 << "Error: unable to retrieve the IControlLimits view";
        return false;
    }

    // handle fingers
    fingers_names = {"thumb", "index", "middle", "ring"};
    for (std::string finger_name : fingers_names)
    {
        // instantiate and configure fingers
        FingerController finger;
        ok = finger.init(hand_name, finger_name,
                         imod_arm, ilimits_arm,
                         ipos_arm, ivel_arm);
        if (!ok)
        {
            yError() << "HandController::configure"
                     << "Error: unable to initialize controller for"
                     << hand_name << finger_name;
            return false;
        }

        // see whether there is additional configuration for this finger
        std::string key_name = hand_name + "_" + finger_name;
        yarp::os::ResourceFinder finger_rf = rf.findNestedResourceFinder(key_name.c_str());
        if (!finger_rf.isNull())
            finger.configure(finger_rf);

        if (use_analogs)
        {
            ok = finger.alignJointsBounds();
            if (!ok)
            {
                yError() << "HandController::configure"
                         << "Error: unable to initialize controller for"
                         << hand_name << finger_name;
                return false;
            }
        }

        fingers[finger_name] = finger;

        // reset fingers contacts
        contacts[finger_name] = false;
    }

    return true;
}

bool HandController::close()
{

    // close finger controllers
    for (std::string finger_name : fingers_names)
        fingers[finger_name].close();

    // close driver
    drv_arm.close();
    drv_analog.close();

    return true;
}

bool HandController::getMotorEncoders(yarp::sig::Vector &encs)
{
    bool ok;

    // extract encoders for the whole arm
    int n_encs;
    ok = ienc_arm->getAxes(&n_encs);
    if (!ok)
    {
        yError() << "HandController::getMotorEncoders"
                 << "Error: unable to retrieve the number of controlled axes";
        return false;
    }
    encs.resize(n_encs);
    ok = ienc_arm->getEncoders(encs.data());
    if (!ok)
    {
        yError() << "HandController::getMotorEncoders"
                 << "Error: unable to retrieve the encoder readings";
        return false;
    }

    return true;
}

bool HandController::getAnalogsEncoders(yarp::sig::Vector &encs)
{
    bool ok;

    ok = ianalog_arm->read(encs);
    if(ok != yarp::dev::IAnalogSensor::AS_OK)
        return false;

    return true;
}

void HandController::resetFingersContacts()
{
    for (auto &contact : contacts)
    {
        bool &is_contact = contact.second;

        is_contact = false;
    }
}

bool HandController::moveFingersUntilContact(const std::vector<std::string> names,
                                             const double &speed,
                                             const std::unordered_map<std::string, bool> &fingers_contacts,
                                             bool &done)
{
    bool ok;

    // get current joints
    yarp::sig::Vector motor_encs;
    yarp::sig::Vector analogs_encs;
    getMotorEncoders(motor_encs);
    if (use_analogs)
        getAnalogsEncoders(analogs_encs);

    for (std::string finger_name : names)
    {
        // if finger never reached contact
        if (!contacts[finger_name])
        {
            // get the finger controller
            FingerController &ctl = fingers[finger_name];

            // try to update finger chain
            if (use_analogs)
                ok = ctl.updateFingerChain(motor_encs, analogs_encs);
            else
                ok = ctl.updateFingerChain(motor_encs);
            if (!ok)
                return false;

            // check if contact is reached now
            if ((fingers_contacts.at(finger_name) == true) ||
                // this is because the ring and the little are coupled
                // and the little could touch before the ring finger
                ((finger_name == "ring") && (fingers_contacts.at("little") == true)))
            {
                yInfo() << finger_name << "reached contact!";

                // stop the finger
                ok = ctl.stop();
                if (!ok)
                    return false;

                // remember that contact was reached
                contacts[finger_name] = true;
            }
            else
            {
                // continue finger movements
                bool enforce_joints_limits = true;
                ok = ctl.moveFingerForward(speed, speed, enforce_joints_limits);
                if (!ok)
                    return false;
            }
        }
    }

    // check if all the contacts were reached
    done = true;
    for (std::string finger_name : names)
    {
        // get contact status
        bool &is_contact = contacts[finger_name];

        done &= is_contact;
    }

    return true;
}

bool HandController::moveFingersMaintainingContact(const std::vector<std::string> names,
                                                   const double &speed,
                                                   const std::unordered_map<std::string, bool> &fingers_contacts)
{
    bool ok;

    // get current joints
    yarp::sig::Vector motor_encs;
    yarp::sig::Vector analogs_encs;
    getMotorEncoders(motor_encs);
    if (use_analogs)
        getAnalogsEncoders(analogs_encs);

    for (std::string finger_name : names)
    {
        // get the finger controller
        FingerController &ctl = fingers[finger_name];

        // try to update finger chain
        if (use_analogs)
            ok = ctl.updateFingerChain(motor_encs, analogs_encs);
        else
            ok = ctl.updateFingerChain(motor_encs);
        if (!ok)
            return false;

        // check if contact is reached
        if ((fingers_contacts.at(finger_name) == true) ||
            // this is because the ring and the little are coupled
            // and the little could touch before the ring finger
            ((finger_name == "ring") && (fingers_contacts.at("little") == true)))
        {
            // stop the finger
            ok = ctl.stop();
            if (!ok)
                return false;

        }
        else
        {
            // continue finger movements
            bool enforce_joints_limits = false;
            ok = ctl.moveFingerForward(speed, speed, enforce_joints_limits);
            if (!ok)
                return false;
        }
    }

    return true;
}


bool HandController::restoreFingersPosition(const std::vector<std::string> &finger_list,
                                            const double &ref_vel)
{
    bool ok;

    // command the requested fingers
    for (const std::string &finger_name : finger_list)
    {
        // get the finger controller
        FingerController &ctl = fingers[finger_name];

        // try to restore the initial position of the finger
        // with the requested joints reference velocity
        ok = ctl.goHome(ref_vel);
        if (!ok)
            return false;
    }

    return true;
}

bool HandController::isFingersRestoreDone(const std::vector<std::string> &finger_list,
                                          bool &is_done)
{
    bool ok;
    bool finger_done;
    is_done = true;
    for (std::string finger_name : fingers_names)
    {
        // get the finger controller
        FingerController &ctl = fingers[finger_name];

        // update done
        ok = ctl.isPositionMoveDone(finger_done);
        if (!ok)
            return false;
        is_done &= finger_done;
    }

    return true;
}

bool HandController::stopFingers(const std::vector<std::string> finger_list)
{
    bool ok;

    for (const std::string &finger_name : finger_list)
    {
        // get the finger controller
        FingerController &ctl = fingers[finger_name];

        // try to stop finger
        ok = ctl.stop();
        if (!ok)
            return false;
    }

    return true;
}

bool HandController::switchToPositionControl(const std::vector<std::string> finger_list)
{
    bool ok;

    for (const std::string &finger_name : finger_list)
    {
        // get the finger controller
        FingerController &ctl = fingers[finger_name];

        // try to switch to position control
        ok = ctl.switchToPositionControl();
        if (!ok)
            return false;
    }

    return true;
}
