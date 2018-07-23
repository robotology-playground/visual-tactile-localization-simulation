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
#include <yarp/os/Vocab.h>

#include <FilterCommand.h>

void yarp::sig::FilterCommand::resetFilter()
{
    this->cmd_value = yarp::os::createVocab('R', 'E', 'S');
}

void yarp::sig::FilterCommand::enableFiltering()
{
    this->cmd_value = yarp::os::createVocab('O', 'N');
}

void yarp::sig::FilterCommand::disableFiltering()
{
    this->cmd_value = yarp::os::createVocab('O', 'F', 'F');
}

void yarp::sig::FilterCommand::enableVisualFiltering()
{
    this->tag_value = yarp::os::createVocab('V', 'I', 'S');
}

void yarp::sig::FilterCommand::enableTactileFiltering(const std::string &hand_name)
{
    if (hand_name == "right")
        this->tag_value = yarp::os::createVocab('T', 'A', 'C', 'R');
    else if (hand_name == "left")
        this->tag_value = yarp::os::createVocab('T', 'A', 'C', 'L');
}

void yarp::sig::FilterCommand::probeContactsOn(const std::string &hand_name)
{
    this->cmd_value = yarp::os::createVocab('P', 'R', 'O', 'N');
    if (hand_name == "right")
        this->tag_value = yarp::os::createVocab('R', 'I', 'G', 'H');
    else if (hand_name == "left")
        this->tag_value = yarp::os::createVocab('L', 'E', 'F', 'T');
}

void yarp::sig::FilterCommand::probeContactsOff()
{
    this->cmd_value = yarp::os::createVocab('P', 'R', 'O', 'F');
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
    tag_value = yarp::os::createVocab('E','M','P','T');
    cmd_value = yarp::os::createVocab('E','M','P','T');
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
