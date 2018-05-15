/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include "HandControlResponse.h"

HandControlResponse::HandControlResponse() : is_approach_done(false),
					     is_restore_done(false),
					     response(Response::Empty) {};

void HandControlResponse::setIsApproachDone(const bool &is_done)
{
    response = Response::ApproachStatus;
    is_approach_done = is_done;
}

void HandControlResponse::setIsRestoreDone(const bool &is_done)
{
    response = Response::RestoreStatus;    
    is_restore_done = is_done;
}

bool HandControlResponse::isApproachDone(bool &is_done) const
{
    if (response != Response::ApproachStatus)
	return false;
    
    is_done = is_approach_done;

    return true;
}

bool HandControlResponse::isRestoreDone(bool &is_done) const
{
    if (response != Response::RestoreStatus)
	return false;

    is_done = is_restore_done;
    
    return true;
}

void HandControlResponse::clear()
{
    response = Response::Empty;
    is_approach_done = false;
    is_restore_done = false;
}

bool HandControlResponse::read(yarp::os::ConnectionReader& connection)
{
    clear();
    
    response = static_cast<Response>(connection.expectInt());

    if (response != Response::Empty)
    {
	if (response == Response::ApproachStatus)
	    is_approach_done = connection.expectInt();
	else if (response == Response::RestoreStatus)
	    is_restore_done = connection.expectInt();
    }

    return !connection.isError();
}

bool HandControlResponse::write(yarp::os::ConnectionWriter& connection)
{
    connection.appendInt(static_cast<int>(response));

    if (response != Response::Empty)
    {
	if (response == Response::ApproachStatus)
	    connection.appendInt(is_approach_done);
	else if (response == Response::RestoreStatus)
	    connection.appendInt(is_restore_done);
    }

    return !connection.isError();
}


