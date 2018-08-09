/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

// std
#include <string>
#include <map>
#include <unordered_map>

// yarp os
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>

// yarp sig
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

// yarp math
#include <yarp/math/Math.h>

// yarp dev
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

#include <cmath>

#include <ArmController.h>
#include <HandControlCommand.h>
#include <HandControlResponse.h>
#include <FilterCommand.h>
#include <TrajectoryGenerator.h>
#include <RotationTrajectoryGenerator.h>
#include <ModelHelper.h>
#include <VIS_TAC_IDL.h>
#include <GazeController.h>

using namespace yarp::math;

enum class Status { Idle,
                    VisualLocalizationOn, VisuoTactileMatching, ContactConstraintsAcquisition, LocalizationOff, ResetFilter,
                    MoveHeadHome, WaitMoveHeadDone,
                    MoveHandUpward, WaitMoveHandUpwardDone,
                    MoveArmRestPosition, WaitMoveArmRestPositionDone,
                    ArmApproach, WaitArmApproachDone,
                    FingersApproach, WaitFingersApproachDone,
                    PrepareRotation, PerformRotation,
                    PreparePull, PerformPull,
                    ArmRestore, WaitArmRestoreDone,
                    FingersRestore, WaitFingersRestoreDone,
                    Stop };

class VisuoTactileLocalizationDemo: public yarp::os::RFModule,
                                    public VIS_TAC_IDL
{
protected:
    double module_period;

    /**
     * Controllers
     */

    // arm controllers
    ArmController right_arm;
    ArmController left_arm;

    // gaze controller
    // used to block vergence before performing localization
    // this is required since point clouds used for localization
    // are obtained using SFM
    GazeController gaze_ctrl;

    // hand controller modules ports
    yarp::os::RpcClient port_hand_right;
    yarp::os::RpcClient port_hand_left;

    // cartesian controller trajectory times
    double default_traj_time;
    double tracking_traj_time;

    // move_hand_upward shift
    double move_hand_upward_shift;

    // length of the positive shift
    // along x direction for pulling action
    double pull_x_shift;

    // open loop yaw rate used for rotation action
    double rot_yaw_rate;

    // minimum allowed z coordinate for
    // approaching phase
    double min_allowed_z;

    // pitch and roll angle used during
    // approaching phase
    double hand_approach_pitch;
    double right_hand_approach_roll;
    double left_hand_approach_roll;

    // fingers opening and closing speeds
    double finger_opening_speed;
    double finger_closing_speed;
    double finger_following_speed;

    // whether to use or not fingers following
    bool use_fingers_following;

    // fingers to be used in approaching phase
    std::vector<std::string> fingers_list_approach;

    // fingers to be used in following mode
    std::vector<std::string> fingers_list_following;

    // fingers to be used when stop is required
    std::vector<std::string> fingers_list_stop;

    // fingers to be used in restore mode
    std::vector<std::string> fingers_list_restore;

    // left and right arm rest positions
    yarp::sig::Vector left_arm_rest_pos;
    yarp::sig::Vector left_arm_rest_att;
    yarp::sig::Vector right_arm_rest_pos;
    yarp::sig::Vector right_arm_rest_att;

    /**
     * Filter
     */

    // filter port
    yarp::os::BufferedPort<yarp::sig::FilterCommand> port_filter;

    // last estimate published by the filter
    yarp::sig::Matrix estimate;
    bool is_estimate_available;

    // FrameTransformClient to read published poses
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;
    std::string est_tf_source;
    std::string est_tf_target;

    /**
     * Trajectory generation and helpers
     */

    // trajectory generator
    TrajectoryGenerator traj_gen;
    RotationTrajectoryGenerator rot_traj_gen;

    // default trajectory length
    double pull_traj_duration;
    double rot_traj_duration;

    // model helper class
    ModelHelper mod_helper;

    /**
     * Rpc
     */

    // rpc server
    yarp::os::RpcServer rpc_port;

    // rpc clients
    yarp::os::RpcClient rpc_tracker;
    yarp::os::RpcClient rpc_lbpextract;
    yarp::os::RpcClient rpc_pcr;
    std::string pcr_object_name;

    // mutexes required to share data between
    // the RFModule thread and the rpc server thread
    yarp::os::Mutex mutex;

    /**
     * Status, booleans, defaults and names
     */

    // simulation modes
    bool simulation_mode;

    // whether to use gaze or not
    bool use_gaze;

    // whether to use ground truth tracker or not
    bool use_tracker;

    // eyes vergence for SFM
    double sfm_vergence;

    // status of the module
    Status status;
    Status previous_status;

    // indicates whether approach have been done
    bool is_approach_done;

    // location of object to reach
    // during approaching phase
    std::string object_approach_pos;

    // name of arm used in sequences of actions
    std::string seq_action_arm_name;

    // name of arm used in single actions
    std::string single_action_arm_name;

    // required to implement counters
    // and timeouts
    bool is_timer_started;
    double last_time;
    double arm_approach_timeout;
    double arm_restore_timeout;
    double arm_upward_timeout;
    double arm_rest_timeout;
    double head_motion_timeout;
    double fingers_approach_timeout;
    double fingers_restore_timeout;

    /**
     * Thrift
     */

    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    std::string init()
    {
        std::string reply;

        // set max area used by module lbpExtract
        setLbpExtractMaxArea(11000);

        if (use_gaze)
        {
            // block vergence of eyes
            // as required by SFM
            gaze_ctrl.blockEyes(sfm_vergence);
        }

        if (!simulation_mode)
        {
            // start point cloud streaming
            startPointCloudReadStream(pcr_object_name);
        }

        reply = "[OK] initialization completed.";

        return reply;
    }

    std::string start_visual_localization()
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            // change status
            previous_status = status;
            status = Status::VisualLocalizationOn;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string visuo_tactile_matching()
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            // change status
            previous_status = status;
            status = Status::VisuoTactileMatching;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string contact_constraints_acquisition()
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            // change status
            previous_status = status;
            status = Status::ContactConstraintsAcquisition;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string stop_localization()
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            // change status
            previous_status = status;
            status = Status::LocalizationOff;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string reset_filter()
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            // change status
            previous_status = status;
            status = Status::ResetFilter;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string set_min_allowed_z(const double min_z)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            min_allowed_z = min_z;

            reply = "[OK] Minimum allowed z changed to " + std::to_string(min_allowed_z);
        }

        mutex.unlock();

        return reply;
    }

    std::string get_min_allowed_z()
    {
        mutex.lock();

        std::string reply;

        reply = "[OK] Minimum allowed z is " + std::to_string(min_allowed_z);

        mutex.unlock();

        return reply;
    }

    std::string get_approach_position(const std::string &object_position)
    {
        mutex.lock();

        std::string reply;

        // check if the estimate is available
        if (!is_estimate_available)
            reply = "[FAILED] Estimate is not available";
        else
        {
            // evaluate the desired hand pose
            // according to the current estimate
            mod_helper.setModelPose(estimate);
            double yaw = mod_helper.evalApproachYawAttitude();
            yarp::sig::Vector pos(3, 0.0);
            mod_helper.evalApproachPosition(pos, object_position);

            reply = "[OK] Planned position is ";
            reply +=  "x: " + std::to_string(pos[0]) +
                    ", y: " + std::to_string(pos[1]) +
                    ", z: " + std::to_string(pos[2]) +
                    ", yaw: " + std::to_string(yaw);

            if (pos[2] < min_allowed_z)
                reply += "(z is TOO LOW)";
        }

        mutex.unlock();

        return reply;
    }

    std::string move_hand_upward(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid hand name";
        else
        {
            // change status
            previous_status = status;
            status = Status::MoveHandUpward;

            // set current hand
            single_action_arm_name = hand_name;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string move_arm_rest_pose(const std::string &arm_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((arm_name != "right") && (arm_name != "left"))
            reply = "[FAILED] You should specify a valid hand name";
        else
        {
            // change status
            previous_status = status;
            status = Status::MoveArmRestPosition;

            // set current hand
            single_action_arm_name = arm_name;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string home_arm(const std::string &arm_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((arm_name != "right") && (arm_name != "left"))
            reply = "[FAILED] You should specify a valid arm name";
        else
        {
            // change status
            previous_status = status;
            status = Status::ArmRestore;

            // set current hand
            single_action_arm_name = arm_name;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string home_head()
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if (!use_gaze)
            reply = "[FAILED] Control of gaze is disabled";
        else
        {
            // change status
            previous_status = status;
            status = Status::MoveHeadHome;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string approach(const std::string &hand_name,
                         const std::string &object_position)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if (!is_estimate_available)
            reply = "[FAILED] Estimate is not available";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid arm name";
        else if ((object_position != "center") &&
                 (object_position != "right") &&
                 (object_position != "left"))
            reply = "[FAILED] You should specify a valid object location";
        else
        {
            // check if planned approach position
            // is safe
            mod_helper.setModelPose(estimate);
            double yaw = mod_helper.evalApproachYawAttitude();
            yarp::sig::Vector pos(3, 0.0);
            mod_helper.evalApproachPosition(pos, object_position);
            if (pos[2] < min_allowed_z)
            {
                reply = "[FAILED] Planned z position (" +
                    std::to_string(pos[2]) +
                    ") is TOO LOW";

                mutex.unlock();
                return reply;
            }

            // change status
            previous_status = status;
            status = Status::ArmApproach;

            // set current hand
            seq_action_arm_name = hand_name;

            // set object approaching position
            object_approach_pos = object_position;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string fingers_approach(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid hand name";
        // TESTING
        // else if (!is_approach_done)
        //     reply = "[FAILED]You should approach the object before approaching with fingers";
        // else if (hand_name != seq_action_arm_name)
        //     reply = "[FAILED]You should continue this sequence of actions with the " +
        //             seq_action_arm_name + " arm";
        else
        {
            // testing
            seq_action_arm_name = hand_name;

            // change status
            previous_status = status;
            status = Status::FingersApproach;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string pull(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED]Wait for completion of the current phase";
        else if (!is_approach_done)
            reply = "[FAILED]You should approach the object before pulling";
        else if (hand_name != seq_action_arm_name)
            reply = "[FAILED]You should continue this sequence of actions with the " +
                    seq_action_arm_name + " arm";
        else
        {
            // change status
            previous_status = status;
            status = Status::PreparePull;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string rotate(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid arm name";
        else if (hand_name != seq_action_arm_name)
            reply = "[FAILED]You should continue this sequence of actions with the " +
                    seq_action_arm_name + " arm";
        else
        {
            // change status
            previous_status = status;
            status = Status::PrepareRotation;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string fingers_restore(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid hand name";
        else
        {
            // change status
            previous_status = status;
            status = Status::FingersRestore;

            // set current hand
            single_action_arm_name = hand_name;

            reply = "[OK] Command issued";
        }

        mutex.unlock();

        return reply;
    }

    std::string enable_contacts_probe(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid hand name";
        else if (setFingersContactsProbe(true, hand_name) &&
                 sendCommandToFilter("contacts_probe_on", "", hand_name))
            reply = "[OK] Fingers contacts probe enabled for "
                + hand_name + " hand";

        mutex.unlock();

        return reply;
    }

    std::string disable_contacts_probe(const std::string &hand_name)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid hand name";
        else if (setFingersContactsProbe(false, hand_name) &&
                 sendCommandToFilter("contacts_probe_off", "", hand_name))
            reply = "[OK] Fingers contacts probe disabled for "
                + hand_name + " hand";

        mutex.unlock();

        return reply;
    }

    std::string stop()
    {
        mutex.lock();

        std::string reply;

        // change status
        previous_status = status;
        status = Status::Stop;

        reply = "[OK] Command issued";

        mutex.unlock();

        return reply;
    }

    std::string quit()
    {
        // stop the module
        stopModule();

        return "[OK] Closing...";
    }

    /*
     * Implementation
     */

    /*
     * Send command to the filtering algorithm.
     * @param enable whether to enable or disable the filtering
     * @param type the type of filtering,
     *        i.e. 'visual' or 'tactile'
     * @param hand_name the name of the hand to be used with tactile filtering
     * @return true/false on success/failure
     */
    bool sendCommandToFilter(const std::string &cmd,
                             const std::string &type = "",
                             const std::string &hand_name = "")
    {
        if (cmd == "enable" && type.empty())
            return false;

        yarp::sig::FilterCommand &filter_cmd = port_filter.prepare();

        // clear the storage
        filter_cmd.clear();

        // enable filtering
        if (cmd == "enable")
            filter_cmd.enableFiltering();
        else if (cmd == "disable")
            filter_cmd.disableFiltering();
        else if (cmd == "reset")
            filter_cmd.resetFilter();
        else if (cmd == "contacts_probe_on")
            filter_cmd.probeContactsOn(hand_name);
        else if (cmd == "contacts_probe_off")
            filter_cmd.probeContactsOff();
        else if (cmd == "contact_constraints_acq")
            filter_cmd.enableContactConstraintsAcqusition(hand_name);

        // enable the correct type of filtering
        if (cmd == "enable")
        {
            if (type == "visual")
                filter_cmd.enableVisualFiltering();
            else if (type == "tactile")
                filter_cmd.enableTactileFiltering(hand_name);
            else if (type == "vis_tac_matching")
                filter_cmd.enableVisuoTactileMatching(hand_name);
        }

        // send command to the filter
        port_filter.writeStrict();

        return true;
    }

    bool sendCommandToTracker(const std::string &cmd_name)
    {
        yarp::os::Bottle cmd;
        yarp::os::Bottle reply;
        cmd.addString(cmd_name);
        rpc_tracker.write(cmd, reply);

        if ((reply.size() == 1) &&
            (reply.get(0).isString()) &&
            (reply.get(0).asString() == "ok"))
            return true;

        return false;
    }

    bool setLbpExtractMaxArea(const int &max_area)
    {
        yarp::os::Bottle cmd;
        yarp::os::Bottle reply;
        cmd.addString("setMaxArea");
        cmd.addInt(max_area);
        rpc_lbpextract.write(cmd, reply);

        if ((reply.size() == 1) &&
            (reply.get(0).asVocab() == yarp::os::Vocab::encode("ok")))
            return true;

        return false;
    }

    bool startPointCloudReadStream(const std::string &object_name)
    {
        yarp::os::Bottle cmd;
        yarp::os::Bottle reply;
        cmd.addString("stream_start");
        cmd.addString(object_name);
        rpc_lbpextract.write(cmd, reply);

        if ((reply.size() == 1) &&
            (reply.get(0).asVocab() == yarp::os::Vocab::encode("ok")))
            return true;

        return false;
    }

    /*
     * Get an arm controller.
     * @param arm_name the required arm controller
     * @return a pointer to the arm controller in case of success,
     *         a null pointer in case of failure
     */
    ArmController* getArmController(const std::string &arm_name)
    {
        if (arm_name == "right")
            return &right_arm;
        else if (arm_name == "left")
            return &left_arm;
        else
            return nullptr;
    }

    /*
     * Get a port connected to the hand controller module.
     * @param hand_name the required hand control module
     * @return true/false on success/failure
     */
    yarp::os::RpcClient* getHandPort(const std::string &hand_name)
    {
        if (hand_name == "right")
            return &port_hand_right;
        else if (hand_name == "left")
            return &port_hand_left;
        else
            return nullptr;
    }

    /*
     * Check if arm motion is done.
     * @param arm_name which arm to ask the status of the motion for
     * @param is_done whether the arm motion is done or not
     * @return true for success, false for failure
     */
    bool checkArmMotionDone(const std::string &arm_name,
                            bool &is_done)
    {
        ArmController *arm = getArmController(arm_name);

        if (arm != nullptr)
            return arm->cartesian()->checkMotionDone(&is_done);
        else
            return false;
    }

    /*
     * Check if head motion is done.
     * @param is_done whether the head motion is done or not
     * @return true for success, false for failure
     */
    bool checkHeadMotionDone(bool &is_done)
    {
        return gaze_ctrl.isMotionDone(is_done);
    }

    /*
     * Check if approaching phase with fingers is done.
     * @param hand_name which hand to ask for
     * @param is_done whether the approach phase is done or not
     * @return true for success, false for failure
     */
    bool checkFingersMotionDone(const std::string &hand_name,
                                const std::string &motion_type,
                                bool &is_done)
    {
        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        HandControlCommand hand_cmd;
        HandControlResponse response;

        // clear messages
        hand_cmd.clear();
        response.clear();

        // request for status
        hand_cmd.setCommandedHand(hand_name);
        if (motion_type == "fingers_approach")
            hand_cmd.requestFingersApproachStatus();
        else if (motion_type == "fingers_restore")
            hand_cmd.requestFingersRestoreStatus();
        else
            return false;
        hand_port->write(hand_cmd, response);

        // check status
        bool ok;
        if (motion_type == "fingers_approach")
            ok = response.isApproachDone(is_done);
        else if (motion_type == "fingers_restore")
            ok = response.isRestoreDone(is_done);
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo::checkFingersMotionDone"
                     << "Error: unable to get the status of the fingers"
                     << "motion from the hand control module";
            return false;
        }

        return true;
    }

    /*
     * Move the hand upward with respect to its current position.
     */
    bool moveHandUpward(const std::string &hand_name)
    {
        bool ok;

        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(hand_name);
        if (arm == nullptr)
            return false;

        // get the current position of the palm of the hand
        yarp::sig::Vector pos;
        yarp::sig::Vector att;
        arm->cartesian()->getPose(pos, att);

        // shift position upward
        pos[2] += move_hand_upward_shift;

        // set trajectory time
        ok = arm->cartesian()->setTrajTime(default_traj_time);
        if (!ok)
            return false;

        // issue command
        arm->cartesian()->goToPoseSync(pos, att);

        return true;
    }

    /*
     * Move the arm in the default rest position.
     */
    bool moveArmRestPosition(const std::string &hand_name)
    {
        bool ok;

        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(hand_name);
        if (arm == nullptr)
            return false;

        // set trajectory time
        ok = arm->cartesian()->setTrajTime(default_traj_time);
        if (!ok)
            return false;

        // issue command
        if (hand_name == "right")
            arm->cartesian()->goToPoseSync(right_arm_rest_pos,
                                           right_arm_rest_att);
        else if (hand_name == "left")
            arm->cartesian()->goToPoseSync(left_arm_rest_pos,
                                           left_arm_rest_att);
        return true;
    }

    /*
     * Perform approaching phase with the specified arm.
     * @param arm_name which arm to use
     * @return true/false on success/failure
     */
    bool approachObjectWithArm(const std::string &arm_name)
    {
        bool ok;

        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // check if the estimate is available
        if (!is_estimate_available)
            return false;

        // check if the object location have been set
        if (object_approach_pos.empty())
            return false;

        // evaluate the desired hand pose
        // according to the current estimate
        mod_helper.setModelPose(estimate);
        double yaw = mod_helper.evalApproachYawAttitude();
        yarp::sig::Vector pos(3, 0.0);
        mod_helper.evalApproachPosition(pos, object_approach_pos);

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        // remove a previously attached finger tip
        ok = arm->detachFingerTip();
        if (!ok)
            return false;

        // change effector to the middle finger
        ok = arm->attachFingerTip("middle");
        if (!ok)
            return false;

        // set desired attitude
        double hand_approach_roll;
        if (arm_name == "right")
            hand_approach_roll = right_hand_approach_roll;
        else if (arm_name == "left")
            hand_approach_roll = left_hand_approach_roll;
        else
            return false;

        arm->setHandAttitude(yaw * 180 / M_PI,
                             hand_approach_pitch,
                             hand_approach_roll);

        // set trajectory time
        ok = arm->cartesian()->setTrajTime(default_traj_time);
        if (!ok)
            return false;

        // request pose to the cartesian interface
        arm->goToPos(pos);

        return true;
    }

    /*
     * Perform approaching phase with the fingers of the specified hand.
     * @param hand_name which hand to use
     * @return true/false on success/failure
     */
    bool approachObjectWithFingers(const std::string &hand_name)
    {
        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        // move fingers towards the object
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(fingers_list_approach);
        hand_cmd.setFingersForwardSpeed(finger_closing_speed);
        hand_cmd.commandFingersApproach();
        hand_port->write(hand_cmd, response);

        return true;
    }

    /*
     * Enable fingers following mode.
     * @param arm_name hand arm to use
     * @return true/false con success/failure
     */
    bool enableFingersFollowing(const std::string &hand_name)
    {
        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        // enable fingers movements towards the object
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(fingers_list_following);
        hand_cmd.setFingersForwardSpeed(finger_following_speed);
        hand_cmd.commandFingersFollow();
        hand_port->write(hand_cmd, response);

        return true;
    }

    /*
     * Enable fingers following mode.
     * @param arm_name hand arm to use
     * @return true/false con success/failure
     */
    bool switchFingersToPositionControl(const std::string &hand_name)
    {
        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        // enable fingers movements towards the object
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(fingers_list_following);
        hand_cmd.switchToPositionControl();
        hand_port->write(hand_cmd, response);

        return true;
    }

    /*
     * Enable/disable fingers contacts probe.
     * @param enable true if the probe should be activated
     * @param hand_name hand to be used
     * @return true/false con success/failure
     */
    bool setFingersContactsProbe(const bool &enable,
                                 const std::string &hand_name)
    {

        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        // enable contacts probe
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        if (enable)
            hand_cmd.probeContacts();
        else
            hand_cmd.goIdle();
        hand_port->write(hand_cmd, response);

        return true;
    }

    /*
     * Configure cartesian controller for pushing phase.
     * @param arm_name which arm to use
     * @return true/false con success/failure
     */
    bool preparePullObject(const std::string &arm_name)
    {
        bool ok;

        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        // remove a previously attached finger tip
        ok = arm->detachFingerTip();
        if (!ok)
            return false;

        // change effector to the middle finger
        ok = arm->attachFingerTip("middle");
        if (!ok)
            return false;

        // get the current position of the finger
        yarp::sig::Vector pos;
        yarp::sig::Vector att;
        arm->cartesian()->getPose(pos, att);

        // set final position
        yarp::sig::Vector pos_f(3, 0.0);
        pos_f = pos;
        pos_f[0] += pull_x_shift;
        // pos_f[1] += pull_y_shift;
        // pos_f[2] += pull_z_shift;

        // configure trajectory generator
        traj_gen.setInitialPosition(pos);
        traj_gen.setFinalPosition(pos_f);
        traj_gen.setDuration(pull_traj_duration);
        traj_gen.init();

        // store the current context because we are going
        // to change the trajectory time
        arm->storeContext();

        // set trajectory time
        // which determines the responsiveness
        // of the cartesian controller
        arm->cartesian()->setTrajTime(tracking_traj_time);

        return true;
    }

    bool prepareRotateObject(const std::string &arm_name)
    {
        bool ok;

        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // check if the estimate is available
        if (!is_estimate_available)
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        // remove a previously attached finger tip
        ok = arm->detachFingerTip();
        if (!ok)
            return false;

        // change effector to the middle finger
        ok = arm->attachFingerTip("middle");
        if (!ok)
            return false;

        // get the current position of the finger
        yarp::sig::Vector finger_pos;
        yarp::sig::Vector attitude;
        arm->cartesian()->getPose(finger_pos, attitude);

        // get the current estimate of the center of the object
        yarp::sig::Vector object_center = estimate.getCol(3).subVector(0, 2);

        // configure the trajectory generator
        rot_traj_gen.setYawRate(rot_yaw_rate * M_PI / 180);
        rot_traj_gen.setObjectCenter(object_center);
        rot_traj_gen.setPullingPoint(finger_pos);

        // store the current context because we are going
        // to change the trajectory time
        arm->storeContext();

        // set trajectory time
        // which determines the responsiveness
        // of the cartesian controller
        ok = arm->cartesian()->setTrajTime(tracking_traj_time);
        if (!ok)
            return false;

        return true;
    }

    bool setArmLinearVelocity(const std::string &arm_name,
                              const yarp::sig::Vector &velocity)
    {
        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        // compose null attitude velocity
        yarp::sig::Vector att_dot(4, 0.0);

        return arm->cartesian()->setTaskVelocities(velocity, att_dot);
    }

    /*
     * Restore the initial configuration of the specified arm.
     * @param arm_name which hand to use
     */
    bool restoreArm(const std::string &arm_name)
    {
        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        // issue restore command
        return arm->goHome();
    }

    /*
     * Restore the initial configuration of the fingers of the specified hand.
     * @param hand_name which hand to use
     */
    bool restoreFingers(const std::string &hand_name)
    {
        // check if the hand name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        // issue restore command
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.clear();
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(fingers_list_restore);
        hand_cmd.setFingersRestoreSpeed(finger_opening_speed);
        hand_cmd.commandFingersRestore();
        hand_port->write(hand_cmd, response);

        return true;
    }

    bool restoreArmControllerContext(const std::string &arm_name)
    {
        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        // restore the context
        arm->restoreContext();

        return true;
    }

    /*
     * Stop control of the specified arm.
     * @param arm_name which arm to stop
     * @return true/false on success/failure

     */
    bool stopArm(const std::string &arm_name)
    {
        // check if the arm name is valid
        if ((arm_name.empty()) ||
            ((arm_name != "right") && (arm_name != "left")))
            return false;

        // pick the correct arm
        ArmController* arm = getArmController(arm_name);
        if (arm == nullptr)
            return false;

        return arm->cartesian()->stopControl();
    }

    /*
     * Stop control of the fingers of the specified hand.
     * @param hand_name which hand to stop
     * @return true/false on success/failure
     */
    bool stopFingers(const std::string &hand_name)
    {
        // check if the arm name is valid
        if ((hand_name.empty()) ||
            ((hand_name != "right") && (hand_name != "left")))
            return false;

        // pick the correct hand
        yarp::os::RpcClient* hand_port = getHandPort(hand_name);
        if (hand_port == nullptr)
            return false;

        // stop all the fingers
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(fingers_list_stop);
        hand_cmd.commandStop();
        hand_port->write(hand_cmd, response);

        return true;
    }

    bool loadListStrings(const yarp::os::ResourceFinder &rf,
                         const std::string &tag_name,
                         std::vector<std::string> &list)
    {
        bool strings_found = false;
        if (!rf.find(tag_name).isNull())
        {
            yarp::os::Bottle* strings_list = rf.find(tag_name).asList();
            if (strings_list != nullptr)
            {
                for (size_t i=0; i<strings_list->size(); i++)
                {
                    yarp::os::Value string_v = strings_list->get(i);
                    if (string_v.isString())
                        list.push_back(string_v.asString());
                    else
                        break;

                    if (i == strings_list->size()-1)
                        strings_found = true;
                }
            }
        }
        if (!strings_found)
            list.clear();

        return strings_found;
    }

    bool loadListDouble(yarp::os::ResourceFinder &rf,
                        const std::string &key,
                        const int &size,
                        yarp::sig::Vector &list)
    {
        if (rf.find(key).isNull())
            return false;

        yarp::os::Bottle* b = rf.find(key).asList();
        if (b == nullptr)
            return false;

        if (b->size() != size)
            return false;

        list.resize(size);
        for (size_t i=0; i<b->size(); i++)
        {
            yarp::os::Value item_v = b->get(i);
            if (item_v.isNull())
                return false;

            if (!item_v.isDouble())
            {
                list.clear();
                return false;
            }

            list[i] = item_v.asDouble();
        }
        return true;
    }
public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
        /**
         * Extract separate resource finders
         */
        yarp::os::ResourceFinder rf_module;
        rf_module = rf.findNestedResourceFinder("module");

        yarp::os::ResourceFinder rf_mod_helper;
        rf_mod_helper = rf.findNestedResourceFinder("model_helper");

        /**
         * Parameters from configuration
         */

        // simulation mode
        simulation_mode = rf_module.find("simulationMode").asBool();
        if (rf_module.find("simulationMode").isNull())
            simulation_mode = "false";

        // whether to use gaze or not
        use_gaze = rf_module.find("useGaze").asBool();
        if (rf_module.find("useGaze").isNull())
            use_gaze = false;

        // whether to use ground truth tracker or not
        use_tracker = rf_module.find("useTracker").asBool();
        if (rf_module.find("useTracker").isNull())
            use_tracker = false;

        // vergence of eyes to be used for SFM
        sfm_vergence = rf_module.find("vergenceForSFM").asDouble();
        if (rf_module.find("vergenceForSFM").isNull())
            sfm_vergence = 5.0;

        // port names
        std::string filter_port_name = rf_module.find("filterPort").asString();
        if (rf_module.find("filterPortName").isNull())
            filter_port_name = "/vis_tac_localization/filter:o";

        std::string right_ctl_port_name = rf_module.find("rightHandCtlPort").asString();
        if (rf_module.find("rightHandCtlPort").isNull())
            right_ctl_port_name = "/vis_tac_localization/hand-control/right/rpc:o";

        std::string left_ctl_port_name = rf_module.find("leftHandCtlPort").asString();
        if (rf_module.find("leftHandCtlPort").isNull())
            left_ctl_port_name = "/vis_tac_localization/hand-control/left/rpc:o";

        std::string tfclient_local_port_name = rf_module.find("tfClientLocalPort").asString();
        if (rf_module.find("tfClientLocalPort").isNull())
            tfclient_local_port_name = "/vis_tac_localization/transformClient";

        std::string tracker_rpc_port_name;
        if (use_tracker)
        {
            tracker_rpc_port_name = rf_module.find("trackerRpcPort").asString();
            if (rf_module.find("trackerRpcPort").isNull())
                tracker_rpc_port_name = "/vis_tac_localization/tracker/rpc:o";
        }

        std::string lbpextract_rpc_port_name;
        std::string pcr_rpc_port_name;
        if (!simulation_mode)
        {
            lbpextract_rpc_port_name = rf_module.find("lbpextractRpcPort").asString();
            if (rf_module.find("lbpextractRpcPort").isNull())
                lbpextract_rpc_port_name = "/vis_tac_localization/lbpextract/rpc:o";

            pcr_rpc_port_name = rf_module.find("pcrRpcPort").asString();
            if (rf_module.find("pcrRpcPort").isNull())
                pcr_rpc_port_name = "/vis_tac_localization/pcr/rpc:o";

            pcr_object_name = rf_module.find("pointCloudReadObjName").asString();
            if (rf_module.find("pointCloudReadObjName").isNull())
                pcr_object_name = "Box";
        }

        // robot name
        std::string robot_name = rf_module.find("robotName").asString();
        if (rf_module.find("robotName").isNull())
            robot_name = "icub";

        std::string arm_with_torso = rf_module.find("armWithTorso").asString();
        if (rf_module.find("armWithTorso").isNull())
            arm_with_torso = "right";

        bool limit_torso_pitch = rf_module.find("limitTorsoPitch").asBool();
        double max_torso_pitch;
        if(rf_module.find("limitTorsoPich").isNull())
            limit_torso_pitch = true;

        if (limit_torso_pitch)
        {
            max_torso_pitch = rf_module.find("maxTorsoPitch").asDouble();
            if (rf_module.find("maxTorsoPitch").isNull())
                max_torso_pitch = 30.0;
        }

        // default trajectory times for Cartesian Controller
        default_traj_time = rf_module.find("cartDefaultTrajTime").asDouble();
        if (rf_module.find("cartDefaultTrajTime").isNull())
            default_traj_time = 4.0;
        // print since important
        yInfo() << "VisuoTactileLocalizationDemo: Cartesian default trajectory time is"
                << default_traj_time;

        tracking_traj_time = rf_module.find("cartTrackingTrajTime").asDouble();
        if (rf_module.find("cartTrackingTrajTime").isNull())
            tracking_traj_time = 0.6;
        // print since important
        yInfo() << "VisuoTactileLocalizationDemo: Cartesian tracking trajectory time is"
                << tracking_traj_time;

        // set default trajectory durations
        pull_traj_duration = rf_module.find("pullTrajDuration").asDouble();
        if (rf_module.find("pullTrajDuration").isNull())
            pull_traj_duration = 4.0;
        // print since important
        yInfo() << "VisuoTactileLocalizationDemo: Pulling trajectory duration is"
                << pull_traj_duration;

        rot_traj_duration = rf_module.find("rotTrajDuration").asDouble();
        if (rf_module.find("rotTrajDuration").isNull())
            rot_traj_duration = 4.0;
        // print since important
        yInfo() << "VisuoTactileLocalizationDemo: Rotation trajectory duration is"
                << rot_traj_duration;

        // set default timeouts
        arm_approach_timeout = rf_module.find("armApproachTimeout").asDouble();
        if (rf_module.find("armApproachTimeout").isNull())
            arm_approach_timeout = 7.0;

        arm_restore_timeout = rf_module.find("armRestoreTimeout").asDouble();
        if (rf_module.find("armRestoreTimeout").isNull())
            arm_restore_timeout = 7.0;

        arm_upward_timeout = rf_module.find("armUpwardTimeout").asDouble();
        if (rf_module.find("armUpwardTimeout").isNull())
            arm_upward_timeout = 7.0;

        arm_rest_timeout = rf_module.find("armRestTimeout").asDouble();
        if (rf_module.find("armRestTimeout").isNull())
            arm_rest_timeout = 7.0;

        if(use_gaze)
        {
            head_motion_timeout = rf_module.find("headMotionTimeout").asDouble();
            if (rf_module.find("headMotionTimeout").isNull())
                head_motion_timeout = 5.0;
        }

        fingers_approach_timeout = rf_module.find("fingersApproachTimeout").asDouble();
        if (rf_module.find("fingersApproachTimeout").isNull())
            fingers_approach_timeout = 10.0;

        fingers_restore_timeout = rf_module.find("fingersRestoreTimeout").asDouble();
        if (rf_module.find("fingersRestoreTimeout").isNull())
            fingers_restore_timeout = 10.0;

        // hand pitch and roll used in approaching phase
        hand_approach_pitch = rf_module.find("handApproachPitch").asDouble();
        if (rf_module.find("handApproachPitch").isNull())
            hand_approach_pitch = 15.0;

        right_hand_approach_roll = rf_module.find("rightHandApproachRoll").asDouble();
        if (rf_module.find("rightHandApproachRoll").isNull())
            right_hand_approach_roll = -90.0;

        left_hand_approach_roll = rf_module.find("leftHandApproachRoll").asDouble();
        if (rf_module.find("leftHandApproachRoll").isNull())
            left_hand_approach_roll = 90.0;

        // fingers list for approaching phase
        if(!loadListStrings(rf_module, "fingersForApproach", fingers_list_approach))
            fingers_list_approach = {"index", "middle", "ring"};

        // fingers list for following mode
        if(!loadListStrings(rf_module, "fingersForFollowing", fingers_list_following))
            fingers_list_following = {"index", "middle", "ring"};

        // fingers list for fingers stop
        if(!loadListStrings(rf_module, "fingersForStop", fingers_list_stop))
            fingers_list_stop = {"index", "middle", "ring"};

        // fingers list for fingers restore
        if(!loadListStrings(rf_module, "fingersForRestore", fingers_list_restore))
            fingers_list_restore = {"index", "middle", "ring"};

        // fingers following enabler
        use_fingers_following = rf_module.find("enableFingersFollowing").asBool();
        if (rf_module.find("enableFingersFollowing").isNull())
            use_fingers_following = false;

        // finger speeds
        finger_opening_speed = rf_module.find("fingerOpeningSpeed").asDouble();
        if (rf_module.find("fingerOpeningSpeed").isNull())
            finger_opening_speed = 25.0;

        finger_closing_speed = rf_module.find("fingerClosingSpeed").asDouble();
        if (rf_module.find("fingerClosingSpeed").isNull())
            finger_closing_speed = 0.009;

        finger_following_speed = rf_module.find("fingerFollowingSpeed").asDouble();
        if (rf_module.find("fingerFollowingSpeed").isNull())
            finger_following_speed = 0.005;

        // module period
        module_period = rf_module.find("modulePeriod").asDouble();
        if (rf_module.find("modulePeriod").isNull())
            module_period = 0.02;

        // shift used for move hand upward action
        move_hand_upward_shift = rf_module.find("moveHandUpwardShift").asDouble();
        if (rf_module.find("moveHandUpwardShift").isNull())
            move_hand_upward_shift = 0.05;

        // shift used during pulling action
        pull_x_shift = rf_module.find("pullXShift").asDouble();
        if (rf_module.find("pullXShift").isNull())
            pull_x_shift = 0.17;

        // yaw rate used during rotation action
        rot_yaw_rate = rf_module.find("rotYawRate").asDouble();
        if (rf_module.find("rotYawRate").isNull())
            rot_yaw_rate = -20.0;

        min_allowed_z = rf_module.find("minimumAllowedZ").asDouble();
        if (rf_module.find("minimumAllowedZ").isNull())
            min_allowed_z = -0.10;

        // filter transform source and target frame
        est_tf_source = rf_module.find("estimateTfSource").asString();
        if (rf_module.find("estimateTfSource").isNull())
            est_tf_source = "/iCub/frame";

        est_tf_target = rf_module.find("estimateTfTarget").asString();
        if (rf_module.find("estimateTfTarget").isNull())
            est_tf_target = "/box_alt/estimate/frame";

        // rest poses for left and right arm
        yarp::sig::Vector left_arm_rest_pose;
        if (!loadListDouble(rf_module, "leftArmRestPose",
                            7, left_arm_rest_pose))
        {
            yError() << "VisuoTactileLocalizationDemo: unable to load rest pose for left arm";
            return false;
        }
        left_arm_rest_pos = left_arm_rest_pose.subVector(0, 2);
        left_arm_rest_att = left_arm_rest_pose.subVector(3, 6);
        yInfo() << left_arm_rest_pos.toString();
        yInfo() << left_arm_rest_att.toString();
        yarp::sig::Vector right_arm_rest_pose;
        if (!loadListDouble(rf_module, "rightArmRestPose",
                            7, right_arm_rest_pose))
        {
            yError() << "VisuoTactileLocalizationDemo: unable to load rest pose for right arm";
            return false;
        }
        right_arm_rest_pos = right_arm_rest_pose.subVector(0, 2);
        right_arm_rest_att = right_arm_rest_pose.subVector(3, 6);
        yInfo() << right_arm_rest_pos.toString();
        yInfo() << right_arm_rest_att.toString();

        // home poses for left and right arm
        yarp::sig::Vector left_arm_home_pose;
        if (!loadListDouble(rf_module, "leftArmHomePose",
                            7, left_arm_home_pose))
        {
            yError() << "VisuoTactileLocalizationDemo: unable to load home pose for left arm";
            return false;
        }
        yarp::sig::Vector right_arm_home_pose;
        if (!loadListDouble(rf_module, "rightArmHomePose",
                            7, right_arm_home_pose))
        {
            yError() << "VisuoTactileLocalizationDemo: unable to load home pose for right arm";
            return false;
        }

        // home fixation point
        yarp::sig::Vector home_fixation;
        if (use_gaze)
        {
            if (!loadListDouble(rf_module, "homeFixationPoint", 3, home_fixation))
            {
                yError() << "VisuoTactileLocalizationDemo: unable to load home fixation point";
                return false;
            }
        }

        /**
         * Ports
         */
        bool ok = port_filter.open(filter_port_name);
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to open the filter port";
            return false;
        }

        ok = port_hand_right.open(right_ctl_port_name);
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to open the right hand control module port";
            return false;
        }

        ok = port_hand_left.open(left_ctl_port_name);
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to open the left hand control module port";
            return false;
        }

        /**
         * Frame Transform Client
         */

        // prepare properties for the FrameTransformClient
        yarp::os::Property propTfClient;
        propTfClient.put("device", "transformClient");
        propTfClient.put("local", tfclient_local_port_name);
        propTfClient.put("remote", "/transformServer");

        // try to open the driver
        ok = drv_transform_client.open(propTfClient);
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to open the FrameTransformClient driver.";
            return false;
        }

        // try to retrieve the view
        ok = drv_transform_client.view(tf_client);
        if (!ok || tf_client == 0)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to retrieve the FrameTransformClient view.";
            return false;
        }

        /**
         * Arm Controllers
         */

        // configure arm controllers
        ok = right_arm.configure(robot_name, "right");
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to configure the right arm controller";
            return false;
        }
        right_arm.setHomePose(right_arm_home_pose.subVector(0, 2),
                              right_arm_home_pose.subVector(3, 6));

        ok = left_arm.configure(robot_name, "left");
        if (!ok)
        {
            yError() << "VisuoTactileLocalizationDemo: unable to configure the left arm controller";
            return false;
        }
        left_arm.setHomePose(left_arm_home_pose.subVector(0, 2),
                             left_arm_home_pose.subVector(3, 6));

        /**
         * Gaze Controller
         */

        if (use_gaze)
        {
            ok = gaze_ctrl.configure(rf, "/vtl-demo");
            if (!ok)
            {
                yError() << "VisuoTactileLocalizationDemo: unable to configure the gaze controller";
                return false;
            }
            gaze_ctrl.setHomeFixation(home_fixation);
        }

        /**
         * Defaults
         */

        // set default value of flags
        is_estimate_available = false;
        is_approach_done = false;
        is_timer_started = false;

        // set default status
        status = Status::Idle;
        previous_status = Status::Idle;

        // configure model helper
        mod_helper.configure(rf_mod_helper);

        // clear arm names
        seq_action_arm_name.clear();
        single_action_arm_name.clear();

        // enable torso on the right arm only
        if (arm_with_torso == "right")
        {
            right_arm.enableTorso();
            if (limit_torso_pitch)
                right_arm.limitTorsoPitch(max_torso_pitch);
        }
        else if (arm_with_torso == "left")
        {
            left_arm.enableTorso();
            if (limit_torso_pitch)
                left_arm.limitTorsoPitch(max_torso_pitch);
        }

        /**
         * Rpc clients
         */

        if (use_tracker)
        {
            ok = rpc_tracker.open(tracker_rpc_port_name);
            if (!ok)
            {
                yError() << "VisuoTactileLocalizationDemo: unable to open the tracker rpc client port";
                return false;
            }
        }

        if (!simulation_mode)
        {
            ok = rpc_lbpextract.open(lbpextract_rpc_port_name);
            if (!ok)
            {
                yError() << "VisuoTactileLocalizationDemo: unable to open the lbpextract rpc client port";
                return false;
            }

            ok = rpc_pcr.open(pcr_rpc_port_name);
            if (!ok)
            {
                yError() << "VisuoTactileLocalizationDemo: unable to open the pcr rpc client port";
                return false;
            }
        }

        /**
         * Rpc server
         */

        // open the rpc server
        rpc_port.open("/vtl-demo/rpc:i");
        attach(rpc_port);

        return true;
    }

    bool close()
    {
        // stop control of arms
        stopArm("right");
        stopArm("left");

        // stop control of fingers
        stopFingers("right");
        stopFingers("left");

        // close arm controllers
        right_arm.close();
        left_arm.close();

        // close ports
        rpc_port.close();
        port_filter.close();
        if (!simulation_mode)
        {
            rpc_tracker.close();
            rpc_lbpextract.close();
            rpc_pcr.close();
        }
    }

    double getPeriod()
    {
        return module_period;
    }

    bool updateModule()
    {
        if(isStopping())
            return false;

        mutex.lock();

        // get the current and previous status
        Status curr_status;
        Status prev_status;
        curr_status = status;
        prev_status = previous_status;

        // get the current arm names

        // this is used in a sequence of actions
        std::string seq_act_arm = seq_action_arm_name;

        // this is used in single actions
        std::string single_act_arm = single_action_arm_name;

        // get current estimate from the filter
        std::string source = est_tf_source;
        std::string target = est_tf_target;
        if (tf_client->getTransform(target, source, estimate))
            is_estimate_available = true;

        // get the current minimum allowed z for approaching phase
        double min_z = min_allowed_z;

        mutex.unlock();

        switch(curr_status)
        {
        case Status::Idle:
        {
            // nothing to do here
            break;
        }

        case Status::VisualLocalizationOn:
        {
            bool ok;

            if (use_gaze)
            {
                // block eyes vergence
                gaze_ctrl.blockEyes(sfm_vergence);
            }

            // issue localization
            ok = sendCommandToFilter("enable", "visual");

            if (!ok)
                yError() << "[VISUAL LOCALIZATION ON] error while sending command to the filter";

            // go back to Idle
            mutex.lock();
            status = Status::Idle;
            mutex.unlock();

            break;
        }

        case Status::VisuoTactileMatching:
        {
            bool ok;

            // issue localization
            ok = sendCommandToFilter("enable", "vis_tac_matching", seq_act_arm);

            if (!ok)
                yError() << "[VISUO TACTILE MATCHING] error while sending command to the filter";

            // go back to Idle
            mutex.lock();
            status = Status::Idle;
            mutex.unlock();

            break;
        }

        case Status::ContactConstraintsAcquisition:
        {
            bool ok;

            // issue localization
            ok = sendCommandToFilter("contact_constraints_acq", "", seq_act_arm);

            if (!ok)
                yError() << "[CONTACT CONSTRAINTS ACQUISITION] error while sending command to the filter";

            // go back to Idle
            mutex.lock();
            status = Status::Idle;
            mutex.unlock();

            break;
        }

        case Status::LocalizationOff:
        {
            bool ok;

            // issue localization
            ok = sendCommandToFilter("disable");

            if (!ok)
                yError() << "[LOCALIZATION OFF] error while sending command to the filter";

            // go back to Idle
            mutex.lock();
            status = Status::Idle;
            mutex.unlock();

            break;
        }

        case Status::ResetFilter:
        {
            bool ok;

            // issue filter reset
            ok = sendCommandToFilter("reset");

            if (!ok)
                yError() << "[RESET FILTER] error while sending command to the filter";

            // go back to Idle
            mutex.lock();
            status = Status::Idle;
            mutex.unlock();

            break;
        }

        case Status::MoveHeadHome:
        {
            bool ok;

            // store current context
            gaze_ctrl.storeContext();

            // clear eyes
            gaze_ctrl.clearEyes();

            // set default trajectory times
            gaze_ctrl.setTrajectoryTimes();

            // issue head home command
            ok = gaze_ctrl.goHome();

            // restore context
            gaze_ctrl.restoreContext();

            if (!ok)
                yError() << "[MOVE HOME HEAD] error while trying to move head in home position";

            // reset timer
            last_time = yarp::os::Time::now();

            // go to WaitMoveHeadDone
            mutex.lock();
            status = Status::WaitMoveHeadDone;
            mutex.unlock();

            break;
        }

        case Status::WaitMoveHeadDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkHeadMotionDone(is_done);

            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > head_motion_timeout)))
            {
                yError() << "[WAIT MOVE HEAD DONE] check motion done failed or timeout reached";

                // stop control
                gaze_ctrl.stop();

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT MOVE HEAD DONE] done";

                // stop control
                gaze_ctrl.stop();

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                mutex.unlock();
            }

            break;
        }

        case Status::MoveHandUpward:
        {
            bool ok;

            // issue command
            ok = moveHandUpward(single_act_arm);

            if (!ok)
            {
                yError() << "[MOVE HAND UPWARD] error while trying to move hand upward";

                // stop control
                stopArm(single_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            mutex.lock();

            // go to WaitMoveHandUpwardDone
            status = Status::WaitMoveHandUpwardDone;

            mutex.unlock();

            // reset timer
            last_time = yarp::os::Time::now();

            break;
        }

        case Status::WaitMoveHandUpwardDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkArmMotionDone(single_act_arm, is_done);

            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > arm_upward_timeout)))
            {
                yError() << "[WAIT MOVE HAND UPWARD DONE] check motion done failed or timeout reached";

                // stop control
                stopArm(single_act_arm);

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT MOVE HAND UPWARD DONE] done";

                // stop control
                stopArm(single_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // clear name
                single_action_arm_name.clear();

                mutex.unlock();
            }

            break;
        }

        case Status::MoveArmRestPosition:
        {
            bool ok;

            // issue command
            ok = moveArmRestPosition(single_act_arm);

            if (!ok)
            {
                yError() << "[MOVE ARM REST POSITION] error while trying to move arm to rest position";

                // stop control
                stopArm(single_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            mutex.lock();

            // go to WaitMoveHandUpwardDone
            status = Status::WaitMoveArmRestPositionDone;

            mutex.unlock();

            // reset timer
            last_time = yarp::os::Time::now();

            break;
        }

        case Status::WaitMoveArmRestPositionDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkArmMotionDone(single_act_arm, is_done);

            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > arm_rest_timeout)))
            {
                yError() << "[WAIT MOVE ARM REST POSITION DONE] check motion done failed or timeout reached";

                // stop control
                stopArm(single_act_arm);

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT MOVE ARM REST POSITION DONE] done";

                // stop control
                stopArm(single_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // clear name
                single_action_arm_name.clear();

                mutex.unlock();
            }

            break;
        }

        case Status::ArmApproach:
        {
            bool ok;

            // disable visual filtering
            ok = sendCommandToFilter("disable");

            if (use_tracker)
            {
                // fixate with eyes
                ok = ok && sendCommandToTracker("eyes-fixate-and-hold");
            }

            // issue approach with arm
            ok = ok && approachObjectWithArm(seq_act_arm);

            if (!ok)
            {
                yError() << "[ARM APPROACH] error while trying to issue arm approach phase";

                // stop control
                stopArm(seq_act_arm);

                mutex.lock();

                if (use_tracker)
                {
                    sendCommandToTracker("eyes-stop");
                }

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // go to state WaitArmApproachDone
            mutex.lock();
            status = Status::WaitArmApproachDone;
            mutex.unlock();

            // reset timer
            last_time = yarp::os::Time::now();

            break;
        }

        case Status::WaitArmApproachDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkArmMotionDone(seq_act_arm, is_done);

            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > arm_approach_timeout)))
            {
                yError() << "[WAIT ARM APPROACH DONE] check motion done failed or timeout reached";

                // stop control
                stopArm(seq_act_arm);

                if (use_tracker)
                {
                    // stop eyes
                    sendCommandToTracker("eyes-stop");
                }

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                // TESTING
                // reset arm name
                // seq_action_arm_name.clear();

                // TESTING
                // update flag
                is_approach_done = true;

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT ARM APPROACH DONE] done";

                mutex.lock();

                if (use_tracker)
                {
                    // stop eyes
                    sendCommandToTracker("eyes-stop");
                }

                // go to Idle
                status = Status::Idle;

                // update flag
                is_approach_done = true;

                mutex.unlock();
            }

            break;
        }

        case Status::FingersApproach:
        {
            bool ok;

            // issue approach with fingers
            ok = approachObjectWithFingers(seq_act_arm);

            if (!ok)
            {
                yError() << "[FINGERS APPROACH] error while trying to issue fingers approach phase";

                // stop control
                stopFingers(seq_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // go to state WaitFingersApproachDone
            mutex.lock();
            status = Status::WaitFingersApproachDone;
            mutex.unlock();

            // reset timer
            last_time = yarp::os::Time::now();

            break;
        }

        case Status::WaitFingersApproachDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkFingersMotionDone(seq_act_arm,
                                             "fingers_approach",
                                             is_done);
            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > fingers_approach_timeout)))
            {
                yError() << "[WAIT FINGERS APPROACH] check motion done failed or timeout reached";

                // stop control
                stopFingers(seq_act_arm);

                // switch to position control
                switchFingersToPositionControl(seq_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                // TESTING
                // seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT FINGERS APPROACH DONE] done";

                // stop control
                stopFingers(seq_act_arm);

                // switch to position control
                switchFingersToPositionControl(seq_act_arm);

                // go to Idle
                mutex.lock();
                status = Status::Idle;
                mutex.unlock();
            }

            break;
        }

        case Status::PreparePull:
        {
            bool ok;

            // once pulling is issued
            // the flag is_approach_done is cleared
            mutex.lock();

            is_approach_done = false;

            mutex.unlock();

            // prepare controller for push
            ok = preparePullObject(seq_act_arm);

            if (use_tracker)
            {
                // enable tracking with eyes
                ok = ok && sendCommandToTracker("eyes-track");
            }

            if (!ok)
            {
                yError() << "[PREPARE PULL] error while trying to issue pulling phase";

                // stop control
                stopArm(seq_act_arm);

                // restore arm controller context
                // that was changed in preparePullObject(seq_act_arm)
                restoreArmControllerContext(seq_act_arm);

                // stop eyes
                if (use_tracker)
                {
                    sendCommandToTracker("eyes-stop");
                }

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // enable tactile filtering
            sendCommandToFilter("enable", "tactile", seq_act_arm);

            // enable fingers following mode
            if (use_fingers_following)
                enableFingersFollowing(seq_act_arm);

            // reset flag
            is_timer_started = false;

            // go to state PerformPull
            mutex.lock();
            status = Status::PerformPull;
            mutex.unlock();

            break;
        }

        case Status::PerformPull:
        {
            bool ok;

            if (!is_timer_started)
            {
                is_timer_started = true;

                // reset time
                last_time = yarp::os::Time::now();
            }

            // eval elapsed time
            double elapsed = yarp::os::Time::now() - last_time;

            // get current trajectory
            yarp::sig::Vector pos(3, 0.0);
            yarp::sig::Vector vel(3, 0.0);
            traj_gen.getTrajectory(elapsed, pos, vel);

            // issue velocity command
            ok = setArmLinearVelocity(seq_act_arm, vel);

            if (!ok)
            {
                yError() << "[PEFORM PULL] error while trying to command linear velocity";

                // stop control
                stopArm(seq_act_arm);

                // restore arm controller context
                // that was changed in preparePullObject(seq_act_arm)
                restoreArmControllerContext(seq_act_arm);

                // stop fingers control
                stopFingers(seq_act_arm);
                // switch fingers to position control
                switchFingersToPositionControl(seq_act_arm);

                // stop eyes tracking
                if (use_tracker)
                {
                    // disable tracking with eyes
                    sendCommandToTracker("eyes-stop");
                }

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // check for trajectory completion
            if (elapsed > pull_traj_duration)
            {
                // issue zero velocities
                vel = 0;
                setArmLinearVelocity(seq_act_arm, vel);

                // stop fingers control
                stopFingers(seq_act_arm);
                // switch fingers to position control
                switchFingersToPositionControl(seq_act_arm);

                // disable filtering
                sendCommandToFilter("disable");

                // stop eyes tracking
                if (use_tracker)
                {
                    // // disable tracking with eyes
                    sendCommandToTracker("eyes-stop");
                }

                // restore arm controller context
                // that was changed in preparePullObject(seq_act_arm)
                restoreArmControllerContext(seq_act_arm);

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();
            }

            break;
        }

        case Status::PrepareRotation:
        {
            bool ok;

            // once rotation is issued
            // the flag is_approach_done is cleared
            mutex.lock();

            is_approach_done = false;

            mutex.unlock();

            // prepare controller for rotation
            ok = prepareRotateObject(seq_act_arm);

            if (use_tracker)
            {
                // enable tracking with eyes
                // ok = ok && sendCommandToTracker("eyes-track");
            }

            if (!ok)
            {
                yError() << "[PREPARE ROTATION] error while trying to issue rotation phase";

                // stop control
                stopArm(seq_act_arm);

                // restore arm controller context
                // that was changed in preparePullObject(seq_act_arm)
                restoreArmControllerContext(seq_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // enable tactile filtering
            sendCommandToFilter("enable", "tactile", seq_act_arm);

            // enable fingers following mode
            if (use_fingers_following)
                enableFingersFollowing(seq_act_arm);

            // reset flags
            is_timer_started = false;

            // go to state PerformRotation
            mutex.lock();
            status = Status::PerformRotation;
            mutex.unlock();

            break;
        }

        case Status::PerformRotation:
        {
            bool ok;

            if (!is_timer_started)
            {
                is_timer_started = true;

                // reset time
                last_time = yarp::os::Time::now();
            }

            // eval elapsed time
            double elapsed = yarp::os::Time::now() - last_time;

            // get current trajectory
            yarp::sig::Vector vel(3, 0.0);
            rot_traj_gen.getVelocity(elapsed, vel);

            // issue velocity command
            ok = setArmLinearVelocity(seq_act_arm, vel);

            if (!ok)
            {
                yError() << "[PEFORM ROTATION] error while trying to command linear velocity";

                // stop control
                stopArm(seq_act_arm);

                // restore arm controller context
                // that was changed in preparePullObject(seq_act_arm)
                restoreArmControllerContext(seq_act_arm);

                // stop fingers control
                stopFingers(seq_act_arm);
                // switch fingers to position control
                switchFingersToPositionControl(seq_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // check for trajectory completion
            if (elapsed > rot_traj_duration)
            {
                // issue zero velocities
                vel = 0;
                setArmLinearVelocity(seq_act_arm, vel);

                // stop fingers control
                stopFingers(seq_act_arm);
                // switch fingers to position control
                switchFingersToPositionControl(seq_act_arm);

                // disable filtering
                sendCommandToFilter("disable");

                // stop eyes tracking
                if (use_tracker)
                {
                    // // disable tracking with eyes
                    sendCommandToTracker("eyes-stop");
                }

                // restore arm controller context
                // that was changed in prepareRotateObject(seq_act_arm)
                restoreArmControllerContext(seq_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();

                mutex.unlock();
            }

            break;
        }

        case Status::FingersRestore:
        {
            bool ok;

            // issue fingers restore
            ok = restoreFingers(single_act_arm);

            if (!ok)
            {
                yError() << "[FINGERS RESTORE] error while trying to issue fingers restore";

                // stop control
                stopFingers(single_act_arm);

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // reset timer
            last_time = yarp::os::Time::now();

            // go to WaitFingersRestoreDone
            mutex.lock();
            status = Status::WaitFingersRestoreDone;
            mutex.unlock();

            break;
        }

        case Status::WaitFingersRestoreDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkFingersMotionDone(single_act_arm,
                                             "fingers_restore",
                                             is_done);
            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > fingers_restore_timeout)))
            {
                yError() << "[WAIT FINGERS RESTORE] check motion done failed or timeout reached";

                // stop control
                stopFingers(single_act_arm);

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // restore completed
                yInfo() << "[WAIT FINGERS RESTORE] done";

                mutex.lock();

                // go back to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();
            }

            break;
        }

        case Status::ArmRestore:
        {
            bool ok;

            is_approach_done = false;

            if (use_tracker)
            {
                // fixate with eyes
                sendCommandToTracker("eyes-fixate-and-hold");
            }

            // issue arm restore
            ok = restoreArm(single_act_arm);

            if (!ok)
            {
                yError() << "[ARM RESTORE] error while trying to issue arm restore";

                // stop control
                stopArm(single_act_arm);

                // stop eyes
                if (use_tracker)
                {
                    sendCommandToTracker("eyes-stop");
                }

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            // go to state WaitArmApproachDone
            mutex.lock();
            status = Status::WaitArmRestoreDone;
            mutex.unlock();

            // reset timer
            last_time = yarp::os::Time::now();

            break;
        }

        case Status::WaitArmRestoreDone:
        {
            // check status
            bool is_done = false;
            bool ok = checkArmMotionDone(single_act_arm, is_done);

            // handle failure and timeout
            if (!ok ||
                ((yarp::os::Time::now() - last_time > arm_restore_timeout)))
            {
                yError() << "[WAIT ARM RESTORE] check motion failed or timeout reached";

                // stop control
                stopArm(single_act_arm);

                restoreArmControllerContext(single_act_arm);

                // stop eyes
                if (use_tracker)
                {
                    sendCommandToTracker("eyes-stop");
                }

                mutex.lock();

                // go to Idle
                status = Status::Idle;

                // reset arm name
                single_action_arm_name.clear();

                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // restore completed
                yInfo() << "[WAIT ARM RESTORE] done";

                // stop control
                stopArm(single_act_arm);

                // stop eyes
                if (use_tracker)
                {
                    sendCommandToTracker("eyes-stop");
                }

                restoreArmControllerContext(single_act_arm);

                // reset arm name
                single_action_arm_name.clear();

                // go back to idle
                mutex.lock();
                status = Status::Idle;
                mutex.unlock();
            }

            break;
        }

        case Status::Stop:
        {
            mutex.lock();

            // stop control
            stopArm("right");
            stopArm("left");
            stopFingers("right");
            stopFingers("left");

            // stop eyes
            if (use_tracker)
            {
                sendCommandToTracker("eyes-stop");
            }

            // disable filtering
            sendCommandToFilter("disable");

            // in case pushing was initiated
            // the previous context of the cartesian controller
            // has to be restored
            if (prev_status == Status::PreparePull   ||
                prev_status == Status::PerformPull   ||
                prev_status == Status::PrepareRotation ||
                prev_status == Status::PerformRotation)
            {
                // restore arm controller context
                restoreArmControllerContext(seq_act_arm);
            }
            else if(prev_status == Status::ArmRestore ||
                    prev_status == Status::WaitArmRestoreDone)
            {
                // restore arm controller context
                restoreArmControllerContext(single_act_arm);
            }

            // reset flag
            // TESTING
            // is_approach_done = false;

            // go back to Idle
            status = Status::Idle;

            mutex.unlock();

            break;
        }
        }

        return true;
    }
};

int main(int argc, char** argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "YARP doesn't seem to be available";
        return 1;
    }

    VisuoTactileLocalizationDemo localizer;
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("demo_config.ini");
    rf.configure(argc,argv);

    return localizer.runModule(rf);

}
