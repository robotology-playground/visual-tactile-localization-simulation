/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

#ifndef FILTER_COMMAND_H
#define FILTER_COMMAND_H

// yarp
#include <yarp/os/Portable.h>
#include <yarp/sig/api.h>

namespace yarp {
    namespace sig {
        class FilterCommand;
    }
}

class yarp::sig::FilterCommand : public yarp::os::Portable
{
private:
    /*
     * Tag describing the type of FilterCommand, i.e.
     * something meaningful for the filtering algorithm
     */
    int tag_value;

    /*
     * Command describing something meaningful for the
     * filtering algorithm, e.g., start/stop commands
     */
    int cmd_value;

    void setTag(int);
    void setCommand(int);

public:
    void resetFilter();
    void enableFiltering();
    void disableFiltering();
    void enableVisualFiltering();
    void enableTactileFiltering(const std::string &hand_name);
    void probeContactsOn(const std::string &hand_name);
    void probeContactsOff();
    int tag() const;
    int command() const;

    void clear();

    /*
     * Read a FilterCommand from a connection.
     * Return true iff a FilterCommand was read correctly.
     */
    bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;
    
    bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE;
};
#endif
