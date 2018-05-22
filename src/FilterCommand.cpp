/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// yarp
#include <yarp/os/Vocab.h>

#include <FilterCommand.h>

void yarp::sig::FilterCommand::resetFilter()
{
    this->cmd_value = VOCAB3('R', 'E', 'S');
}

void yarp::sig::FilterCommand::enableFiltering()
{
    this->cmd_value = VOCAB2('O', 'N');
}

void yarp::sig::FilterCommand::disableFiltering()
{
    this->cmd_value = VOCAB3('O', 'F', 'F');
}

void yarp::sig::FilterCommand::enableVisualFiltering()
{
    this->tag_value = VOCAB3('V', 'I', 'S');
}

void yarp::sig::FilterCommand::enableTactileFiltering()
{
    this->tag_value = VOCAB3('T', 'A', 'C');
}

int yarp::sig::FilterCommand::tag() const
{
    return tag_value;
}

int yarp::sig::FilterCommand::command() const
{
    return cmd_value;
}

void yarp::sig::FilterCommand::clear()
{
    tag_value = VOCAB4('E','M','P','T');
    cmd_value = VOCAB4('E','M','P','T');
}

bool yarp::sig::FilterCommand::read(yarp::os::ConnectionReader& connection)
{
    // TODO: add checks

    // get command
    cmd_value = connection.expectInt();

    // get tag
    tag_value = connection.expectInt();

    return true;
}

bool yarp::sig::FilterCommand::write(yarp::os::ConnectionWriter& connection)
{
    // append command
    connection.appendInt(cmd_value);

    // append tag
    connection.appendInt(tag_value);

    return true;
}
