/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

#ifndef HAND_CONTROL_RESPONSE_H
#define HAND_CONTROL_RESPONSE_H

// yarp
#include <yarp/os/Portable.h>

enum class Response { Empty = 0, ApproachStatus = 1, RestoreStatus = 2};

class HandControlResponse : public yarp::os::Portable
{
private:
    /*
     * Type of the response
     */
    Response response;

    /*
     * Status of the controller
     */
    bool is_approach_done;
    bool is_restore_done;

public:
    /*
     * Constructor
     */
    HandControlResponse();

    /*
     * Set the status of the approach phase
     * @param is_done whether the approach phase is done or not
     */
    void setIsApproachDone(const bool &is_done);

    /*
     * Set the status of the restore phase
     * @param is_done whether the restore phase is done or not
     */
    void setIsRestoreDone(const bool &is_done);

    /*
     * Return the status of the approach phase
     * @param is_done whether the approach phase is done or not
     * @return true if the response contains this information
     */
    bool isApproachDone(bool &is_done) const;

    /*
     * Return the status of the restore phase
     * @param is_done whether the approach phase is done or not
     * @return true if the response contains this information
     */
    bool isRestoreDone(bool &is_done) const;

    /*
     * Clear the response
     */
    void clear();

    /*
     * Return true iff a HandControlResponse was received succesfully
     */
    bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;

    /*
     * Return true iff a HandControlResponse was sent succesfully
     */
    bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE;
};
#endif
