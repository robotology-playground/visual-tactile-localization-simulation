/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
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
#include <filterCommand.h>
#include <TrajectoryGenerator.h>
#include <RotationTrajectoryGenerator.h>
#include <ModelHelper.h>
#include <VIS_TAC_IDL.h>

using namespace yarp::math;

enum class Status { Idle,
                    Localize,
                    MoveHandUpward,
                    ArmApproach, WaitArmApproachDone,
                    FingersApproach, WaitFingersApproachDone,
                    PrepareRotation, PerformRotation,
                    PreparePull, PerformPull,
                    ArmRestore, WaitArmRestoreDone,
                    FingersRestore, WaitFingersRestoreDone,
                    Stop };

class VisTacLocSimModule: public yarp::os::RFModule,
                          public VIS_TAC_IDL
{
protected:

    /**
     * Controllers
     */
    
    // arm controllers
    ArmController right_arm;
    ArmController left_arm;

    // hand controller modules ports
    yarp::os::RpcClient port_hand_right;
    yarp::os::RpcClient port_hand_left;

    // cartesian controller trajectory times
    double default_traj_time;
    double tracking_traj_time;

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
     * Rpc server
     */

    // rpc server
    yarp::os::RpcServer rpc_port;

    // mutexes required to share data between
    // the RFModule thread and the rpc thread
    yarp::os::Mutex mutex;

    /**
     * Status, booleans, defaults and names
     */
    
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
    double fingers_approach_timeout;
    double fingers_restore_timeout;

    /**
     * Thrift
     */

    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    std::string localize()
    {
        mutex.lock();

        std::string reply;        

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else
        {
            // change status
            previous_status = status;
            status = Status::Localize;

            reply = "[OK] Command issued";
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

    std::string home(const std::string &arm_name)
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

    std::string approach(const std::string &hand_name,
                         const std::string &object_position)
    {
        mutex.lock();

        std::string reply;

        if (status != Status::Idle)
            reply = "[FAILED] Wait for completion of the current phase";
        else if ((hand_name != "right") && (hand_name != "left"))
            reply = "[FAILED] You should specify a valid arm name";
        else if ((object_position != "center") &&
                 (object_position != "right") &&
                 (object_position != "left"))
            reply = "[FAILED] You should specify a valid object location";
        else
        {
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
        else if (!is_approach_done)
            reply = "[FAILED]You should approach the object before approaching with fingers";
        else if (hand_name != seq_action_arm_name)
            reply = "[FAILED]You should continue this sequence of actions with the " +
                    seq_action_arm_name + " arm";
        else
        {
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
     * @return true/false on success/failure
     */
    bool sendCommandToFilter(const bool& enable, const std::string &type = "")
    {
        if (enable && type.empty())
            return false;

        yarp::sig::FilterCommand &filter_cmd = port_filter.prepare();

        // clear the storage
        filter_cmd.clear();

        // enable filtering
        if (enable)
            filter_cmd.enableFiltering();
        else
            filter_cmd.disableFiltering();

        // enable the correct type of filtering
        if (enable)
        {
            if (type == "visual")
                filter_cmd.enableVisualFiltering();
            else if (type == "tactile")
                filter_cmd.enableTactileFiltering();
        }

        // send command to the filter
        port_filter.writeStrict();

        return true;
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
            yError() << "VisTacLocSimModule::checkFingersMotionDone"
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
        pos[2] += 0.05;

        // set trajectory time
        ok = arm->cartesian()->setTrajTime(default_traj_time);
        if (!ok)
            return false;

        // issue command
        arm->cartesian()->goToPose(pos, att);
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
        arm->setHandAttitude(yaw * 180 / M_PI, 15, -90);

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
        std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(finger_list);
        hand_cmd.setFingersForwardSpeed(0.009);
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
        std::vector<std::string> finger_list = {"index", "middle", "ring"};
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(finger_list);
        hand_cmd.setFingersForwardSpeed(0.005);
        hand_cmd.commandFingersFollow();
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
        pos_f[0] += 0.17;

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
        rot_traj_gen.setYawRate(-20 * M_PI / 180);
        rot_traj_gen.setObjectCenter(object_center);
        rot_traj_gen.setPullingPoint(finger_pos);

        // store the current context because we are going
        // to change the trajectory time
        arm->storeContext();

        // set trajectory time
        // which determines the responsiveness
        // of the cartesian controller
        arm->cartesian()->setTrajTime(tracking_traj_time);
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
        arm->goHome();

        return true;
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
        std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.clear();
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(finger_list);
        hand_cmd.setFingersRestoreSpeed(25.0);
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
        std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
        HandControlCommand hand_cmd;
        HandControlResponse response;
        hand_cmd.setCommandedHand(hand_name);
        hand_cmd.setCommandedFingers(finger_list);
        hand_cmd.commandStop();
        hand_port->write(hand_cmd, response);

        return true;
    }

public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
        /**
         * Ports
         */
        bool ok = port_filter.open("/vis_tac_localization/filter:o");
        if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the filter port";
            return false;
        }

        ok = port_hand_right.open("/vis_tac_localization/hand-control/right/rpc:o");
        if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the right hand control module port";
            return false;
        }

        ok = port_hand_left.open("/vis_tac_localization/hand-control/left/rpc:o");
        if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the left hand control module port";
            return false;
        }

        /**
         * Frame Transform Client
         */
        
        // prepare properties for the FrameTransformClient
        yarp::os::Property propTfClient;
        propTfClient.put("device", "FrameTransformClient");
        propTfClient.put("local", "/vis_tac_localization/transformClient");
        propTfClient.put("remote", "/transformServer");

        // try to open the driver
        ok = drv_transform_client.open(propTfClient);
        if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the FrameTransformClient driver.";
            return false;
        }

        // try to retrieve the view
        ok = drv_transform_client.view(tf_client);
        if (!ok || tf_client == 0)
        {
            yError() << "VisTacLocSimModule: unable to retrieve the FrameTransformClient view.";
            return false;
        }

        /**
         * Arm Controllers
         */

        // configure arm controllers
        ok = right_arm.configure("icubSim", "right");
        if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to configure the right arm controller";
            return false;
        }

        ok = left_arm.configure("icubSim", "left");
        if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to configure the left arm controller";
            return false;
        }

        // default trajectory times for Cartesian Controller
        default_traj_time = 4.0;
        tracking_traj_time = 1.0;

        /**
         * Trajectory times
         */

        // set default trajectory times
        pull_traj_duration = 4.0;
        rot_traj_duration = 4.0;

        // set default timeouts
        arm_approach_timeout = 5.0;
        arm_restore_timeout = 5.0;        
        fingers_approach_timeout = 10.0;
        fingers_restore_timeout = 10.0;

        /**
         * Defaults
         */

        // set default hands orientation
        right_arm.setHandAttitude(0, 15, 0);
        left_arm.setHandAttitude(0, 15, 0);

        // set default value of flags
        is_estimate_available = false;
        is_approach_done = false;
        is_timer_started = false;

        // set default status
        status = Status::Idle;
        previous_status = Status::Idle;

        // configure model helper
        mod_helper.setModelDimensions(0.24, 0.17, 0.037);

        // clear arm names
        seq_action_arm_name.clear();
        single_action_arm_name.clear();

        // enable torso on the right arm only
        right_arm.enableTorso();

        /**
         * Rpc server
         */

        // open the rpc server
        // TODO: take name from config
        rpc_port.open("/service");
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
    }

    double getPeriod()
    {
        return 0.02;
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
        std::string source = "/iCub/frame";
        std::string target = "/box_alt/estimate/frame";
        is_estimate_available = tf_client->getTransform(target, source, estimate);

        mutex.unlock();

        switch(curr_status)
        {
        case Status::Idle:
        {
            // nothing to do here
            break;
        }

        case Status::Localize:
        {
            bool ok;
            
            // issue localization
            ok = sendCommandToFilter(true, "visual");

            if (!ok)
                yError() << "[LOCALIZE] error while sending command to the filter";

            // go back to Idle
            mutex.lock();
            status = Status::Idle;
            mutex.unlock();

            break;
        }

        case Status::MoveHandUpward:
        {
            bool ok;
            
            // issue command
            ok = moveHandUpward(single_act_arm);

            if (!ok)
                yError() << "[MOVE HAND UPWARD] error while trying to move hand upward";

            mutex.lock();
            
            // go back to Idle            
            status = Status::Idle;

            // clean arm anme
            single_act_arm = "";
            
            mutex.unlock();

            break;
        }

        case Status::ArmApproach:
        {
            bool ok;
            
            // issue approach with arm
            ok = approachObjectWithArm(seq_act_arm);

            if (!ok)
            {
                yError() << "[ARM APPROACH] error while trying to issue arm approach phase";

                // stop control
                stopArm(seq_act_arm);
                
                mutex.lock();

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

                mutex.lock();

                // go back to Idle                
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();
                
                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT ARM APPROACH DONE] done";

                // go to Idle
                mutex.lock();
                status = Status::Idle;
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

                mutex.lock();

                // go to Idle                
                status = Status::Idle;

                // reset arm name
                seq_action_arm_name.clear();
                
                mutex.unlock();

                break;
            }

            if (is_done)
            {
                // approach completed
                yInfo() << "[WAIT FINGERS APPROACH DONE] done";                


                mutex.lock();

                // go to Idle                
                status = Status::Idle;

                // update flag
                is_approach_done = true;
                
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

            if (!ok)
            {
                yError() << "[PREPARE PULL] error while trying to issue pulling phase";

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
            sendCommandToFilter(true, "tactile");

            // enable fingers following mode
            // enableFingersFollowing(seq_act_arm);

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
                // stopFingers(seq_act_arm);

                // disable filtering
                sendCommandToFilter(false);

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
            sendCommandToFilter(true, "tactile");

            // enable fingers following mode
            // enableFingersFollowing(seq_act_arm);

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
                // stopFingers(seq_act_arm);

                // disable filtering
                sendCommandToFilter(false);

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
                yError() << "[WAIT FINGERS RESTORE] done";                

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
            
            // issue arm restore
            ok = restoreArm(single_act_arm);

            if (!ok)
            {
                yError() << "[ARM RESTORE] error while trying to issue arm restore";

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
                yError() << "[WAIT ARM RESTORE] done";

                // go back to Idle
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

            // disable filtering
            sendCommandToFilter(false);

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

            // reset flag
            is_approach_done = false;

            // go back to Idle
            status = Status::Idle;

            mutex.unlock();

            break;
        }
        }
        
        return true;
    }
};

int main()
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "YARP doesn't seem to be available";
        return 1;
    }

    VisTacLocSimModule mod;
    yarp::os::ResourceFinder rf;
    return mod.runModule(rf);

}

