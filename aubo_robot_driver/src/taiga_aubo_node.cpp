#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Int64.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <aubo_msgs/SetPayload.h>
#include <aubo_msgs/SetIO.h>
#include <ros/console.h>
#include <ros_control_hw_interface/IROSHardware.h>

// AUBO SDK
#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

// my aubo class:
class AuboController : public IROSHardware
{
    //TODO: sort out public and private later.
    public:
        JointCmdMode robot_cmd_mode_ = MD_NONE;
        bool estopped_;
        
        std::shared_ptr<RpcClient> rpc_cli_;
        std::shared_ptr<RtdeClient> rtde_client_;
        std::string robot_ip_ = "192.168.100.3";
        std::string robot_name_;
        RobotInterfacePtr robot_interface_;

        // RTDE subscriber data:
        std::mutex rtde_mtx_;
        std::vector<double> actual_q_{ std::vector<double>(6, 0.) };
        std::vector<double> actual_qd_{ std::vector<double>(6, 0.) };
        std::vector<double> actual_TCP_pose_{ std::vector<double>(6, 0.) };
        RobotModeType robot_mode_ = RobotModeType::NoController;
        SafetyModeType safety_mode_ = SafetyModeType::Normal;
        RuntimeState runtime_state_ = RuntimeState::Stopped;
        int line_{ -1 };
        bool rtde_data_valid_=false;
        // RTDE IO data
        std::mutex rtde_input_mtx_;
        uint64_t IO_inputs_;
        uint64_t TOOL_IO_inputs_;
        bool rtde_input_data_valid_=false;

        //other
        std::vector<double> actual_joint_torque_{ std::vector<double>(6, 0.) };
        std::vector<double> actual_current_{ std::vector<double>(6, 0.1) };
        std::vector<double> target_q_{ std::vector<double>(6, 0) };
        std::vector<double> target_qd_{ std::vector<double>(6, 0) };
        std::vector<double> actual_current_e{ std::vector<double>(6, 0) };
        std::vector<double> actual_TCP_speed_{ std::vector<double>(6, 0.) };
        std::vector<double> actual_TCP_force_{ std::vector<double>(6, 0.) };
        std::vector<double> target_TCP_pose_{ std::vector<double>(6, 0.) };
        std::vector<double> target_TCP_speed_{ std::vector<double>(6, 0.) };



        // ROS Publishers, Services and subscribers
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > cmd_out_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool> > estop_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Int64> > robot_mode_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Int64> > safety_mode_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Int64> > runtime_state_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::UInt64> > IO_inputs_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::UInt64> > TOOL_IO_inputs_pub_;
        ros::ServiceServer go_op_svc_;
        ros::ServiceServer set_load_svc_;
        ros::ServiceServer set_IO_output_svc_;


        AuboController(ros::NodeHandle &node_handle): IROSHardware(node_handle) // example of how to call a specific parent constructor
        {

            //don't waste time if we won't be able to configure this anyways
            //base class errors should be informative enough
            if(hw_state_== ST_ERROR || hw_state_ == ST_FINAL)
                return;

            control_hz_ = 500;

            //make w/e you need to here.
            std::cout<<"aubo Controller Constructor\n";
            std::cout<<"----------------------------\n";
            std::cout<<"num joints: " << num_joints_<<std::endl;
            std::cout<<"frequency: " << control_hz_<<std::endl;
            std::cout<<"----------------------------\n";

            // realtime publisher
            cmd_out_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(node_handle, "aubo_hw_cmd_out", 1));
            cmd_out_pub_->msg_.data.resize(num_joints_);

            // all the other publishers of robot state and mode and the like
            init_ancillary_pubs(node_handle);

            //not ready at the end, because it needs to init--> recieve some data, and wait for controllers to load.
            hw_state_ = ST_NOT_READY;

        }
        ~AuboController(){};


        /*
            READ()
        */
        int read(ros::Duration &dt){

            {
                std::unique_lock<std::mutex> lck(rtde_mtx_);
                if (rtde_data_valid_)
                {
                    for(int jnt=0; jnt<6; jnt++)
                    {
                        joint_pos_[jnt] = actual_q_[jnt];
                        joint_vel_[jnt] = actual_qd_[jnt];
                        joint_eff_[jnt] = actual_joint_torque_[jnt];
                    }
                    rtde_data_valid_ = false;
                    data_valid_ = true; //latch for now
                }
            }
            ROS_DEBUG_THROTTLE(1,"[HW] [read]");
            return 0;
        };


        /*
            WRITE()
        */
        int write(ros::Duration &dt){
            int ret=-9;
            ROS_DEBUG_THROTTLE(1, "[HW] [write]");
            enforceLimit(dt); // will work with pos/vel/effort, depending on configuration

            if(robot_cmd_mode_==MD_POSITION){
                ROS_DEBUG_THROTTLE(1, "[HW] [write] MD_POSITION");
                double target_time = 2.0/control_hz_;
                ret=robot_interface_->getMotionControl()->servoJoint(joint_pos_cmd_, 31.4, 3.14, target_time, 1.0, 1.0);

                //publish the cmd that we received at full loop rate for now.
                if (cmd_out_pub_->trylock()){
                    for(int jid=0; jid<num_joints_; jid++) cmd_out_pub_->msg_.data[jid]=joint_pos_cmd_[jid];
                }
                cmd_out_pub_->unlockAndPublish();

            }
            else if (robot_cmd_mode_==MD_NONE)
            {
                // roscontrol is idle, this is fine.
                ret=0;
            }
            else
            {
                ROS_DEBUG_THROTTLE(10, "[HW] [write] Unsupported Mode");
                ret=-9;
            }

            return ret;
        };


        /*
            CONFIGURE()
        */
        int configure()
        {
            ROS_INFO("[AUBO HW] Connecting RPC...");
            // set up an RPC client
            rpc_cli_ = std::make_shared<RpcClient>();
            rpc_cli_->setRequestTimeout(1000);  //TODO: perhaps this should be shorter? 
            rpc_cli_->connect(robot_ip_, 30004);
            rpc_cli_->login("aubo", "123456");   //no evidence the u/p combo is sent to the robot at any time

            ROS_INFO("[AUBO HW] Connecting RTDE...");
            // set up an RTDE client
            rtde_client_ = std::make_shared<RtdeClient>();
            rtde_client_->connect(robot_ip_, 30010);
            rtde_client_->login("aubo", "123456");

            
            ROS_INFO("[AUBO HW] Starting RTDE Stream...");
            // subscribe to an RTDE stream for robot position data
            int topic1 = rtde_client_->setTopic(false,
                { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
                "runtime_state", "line_number", "R1_actual_TCP_pose" },
                control_hz_, 0);

            rtde_client_->subscribe(topic1, [this](InputParser &parser) 
                {
                    /* -- THIS ANONYMOUS FUNCTION IS THE CALLBACK THAT HANDLES UNLOADING THE RTDE MESSAGE -- */
                    std::unique_lock<std::mutex> lck(rtde_mtx_);
                    actual_q_ = parser.popVectorDouble();
                    actual_qd_ = parser.popVectorDouble();
                    robot_mode_ = parser.popRobotModeType();
                    safety_mode_ = parser.popSafetyModeType();
                    runtime_state_ = parser.popRuntimeState();
                    line_ = parser.popInt32();
                    actual_TCP_pose_ = parser.popVectorDouble();
                    rtde_data_valid_=true;
                    // process estop bool immediately
                    if((safety_mode_ == SafetyModeType::Normal) || (safety_mode_ == SafetyModeType::ReducedMode))  
                        estopped_ = false;
                    else
                        estopped_ = true;

                });

            // subscribe to an RTDE stream for IO data @ 20Hz
            topic1 = rtde_client_->setTopic(false, { "R1_standard_digital_input_bits", "R1_tool_digital_input_bits" }, 20, 1);

            rtde_client_->subscribe(topic1, [this](InputParser &parser) 
                {
                    /* -- THIS ANONYMOUS FUNCTION IS THE CALLBACK THAT HANDLES UNLOADING THE RTDE INPUT MESSAGE -- */
                    std::unique_lock<std::mutex> lck(rtde_input_mtx_);
                    IO_inputs_ = parser.popInt64();
                    TOOL_IO_inputs_ = parser.popInt64();
                    rtde_input_data_valid_=true;
                });


            // get RPC robot interface handle
            robot_name_ = rpc_cli_->getRobotNames().front();
            robot_interface_ = rpc_cli_->getRobotInterface(robot_name_);
            
            // set speed, because the example did it too.
            robot_interface_->getMotionControl()->setSpeedFraction(0.3);
        
            // ---- SERVOJ mode activation moved to activate() due to timing concerns. ----

            hw_state_ = ST_INACTIVE;
            return 0;
            
        };


        bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list){

			//ONLY START STATE CONTROLLER WITHOUT VALID READ
			for(auto controller : start_list){
				if( (controller.type != "joint_state_controller/JointStateController") && (!data_valid_) ){
					ROS_ERROR("[HW] [prepareSwitch] [START] cannot start controller prior to valid read()");
					ROS_DEBUG("[HW] [prepareSwitch] [START] requested controller:");
					ROS_DEBUG("[HW] [prepareSwitch] [START] --> NAME:  %s", controller.name.c_str());
					ROS_DEBUG("[HW] [prepareSwitch] [START] --> TYPE: %s", controller.type.c_str());
					return false;
				}
			}

            //call parent to do basic stuff
            IROSHardware::prepareSwitch(start_list, stop_list);

            return true;
        }


		void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
					const std::list<hardware_interface::ControllerInfo> &stop_list){
            robot_cmd_mode_ = joint_modes_[0]; // just set the robot control mode to be the mode of the first joint;
			ROS_DEBUG("[HW] [doSwitch]: ROBOT MODE: %d", robot_cmd_mode_);
			ROS_INFO("[HW] [doSwitch]: Switching Controllers . . . ");
		}


        /*
            ACTIVATE() 
                - The main loop must start within 5 sampling cycles after calling activate()
                - activate stalls for ~15ms waiting for the robot to transition to the correct mode
        */
        int activate(){
            // begin servoj mode
            ROS_INFO("ACTIVATING");
            robot_interface_->getMotionControl()->setServoMode(true);
            int i = 0;
            while (!robot_interface_->getMotionControl()->isServoModeEnabled()) {
                if (i++ > 5) {  //failure condition
                    std::cout << "Failed to enable Servo mode! The current servo status is "
                              << rpc_cli_->getRobotInterface(robot_name_)->getMotionControl()->isServoModeEnabled() << std::endl;
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }


            hw_state_ = ST_ACTIVE;
            return 0;};


        int deactivate(){
            if(hw_state_ == ST_UNCONFIGURED){ //cannot transition from UNCONFIGURED --> INACTIVE (by deactivating)
                return 1;
            }
            if(hw_state_ == ST_NOT_READY){ //cannot transition from NOT_READY--> INACTIVE (by deactivating)
                return 1;
            }
            if(hw_state_ == ST_INACTIVE){ //if INACTIVE do nothing
                return 0;
            }
            
            // Turn off servo mode
            robot_interface_->getMotionControl()->setServoMode(false);

            // Wait for end of servo mode
            int i = 0;
            while (robot_interface_->getMotionControl()->isServoModeEnabled()) {
                if (i++ > 5) {
                    std::cout
                        << "Servo Mode disablement failed! The current Servo mode is "
                        << robot_interface_->getMotionControl()->isServoModeEnabled()
                        << std::endl;
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            hw_state_ = ST_INACTIVE; //return ST_INACTIVE on success
            return 0;
        };


        int procError(){
            hw_state_ = ST_FINAL; //can go to ST_INACTIVE if recovery is successful
            return 0;};


        int shutdown(){
            hw_state_ = ST_FINAL;
            return 0;};


        int errorShutdown(){
            hw_state_ = ST_FINAL;
            return 0;};

        
        bool waitForRobotMode(RobotModeType target_mode, double max_time = 10.0)
        {
            RobotModeType current_mode;
            double wait_time=0.0;

            do {
                current_mode = robot_interface_->getRobotState()->getRobotModeType();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                wait_time+=0.100;
                if(wait_time>max_time) 
                {
                    ROS_WARN("[AUBO HW] Robot mode did not achieve target mode before wait expired.");   //TODO: details
                    return(false);
                }
            } while (current_mode != target_mode);
            return(true);
        }


        // broken out into a function for easy transplanting back into aubo_ros_driver
        void init_ancillary_pubs(ros::NodeHandle &node_handle)
        {

            robot_mode_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int64>(node_handle, "robot_mode", 1));
            robot_mode_pub_->msg_.data=-1000;

            safety_mode_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int64>(node_handle, "safety_mode", 1));
            safety_mode_pub_->msg_.data=-1000;

            runtime_state_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Int64>(node_handle, "runtime_mode", 1));
            runtime_state_pub_->msg_.data=-1000;

            estop_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(node_handle, "estop", 1));
            estop_pub_->msg_.data=false;

            IO_inputs_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::UInt64>(node_handle, "io_inputs", 1));
            IO_inputs_pub_->msg_.data=0;

            TOOL_IO_inputs_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::UInt64>(node_handle, "tool_io_inputs", 1));
            TOOL_IO_inputs_pub_->msg_.data=0;

            // init srvs
            go_op_svc_ = node_handle.advertiseService("go_operational", &AuboController::go_op_svc_cb, this);
            set_load_svc_ = node_handle.advertiseService("set_load", &AuboController::set_load_cb, this);
            set_IO_output_svc_ = node_handle.advertiseService("set_io_ouput", &AuboController::set_io_output_cb, this);
        }


        void publish_ancillary_data()
        {
            // new scope for rtde data republishers because we need to lock the mutex
            {
                std::unique_lock<std::mutex> lck(rtde_mtx_);
                if (robot_mode_pub_->trylock())
                {
                    robot_mode_pub_->msg_.data = (int64_t)robot_mode_;
                }
                robot_mode_pub_->unlockAndPublish();

                if (safety_mode_pub_->trylock())
                {
                    safety_mode_pub_->msg_.data = (int64_t)safety_mode_;
                }
                safety_mode_pub_->unlockAndPublish();

                if (runtime_state_pub_->trylock())
                {
                    runtime_state_pub_->msg_.data = (int64_t)runtime_state_;
                }
                runtime_state_pub_->unlockAndPublish();

            }
            {
                //scope for rtde input republisher
                std::unique_lock<std::mutex> lck(rtde_input_mtx_);
                if (IO_inputs_pub_->trylock())
                {
                    IO_inputs_pub_->msg_.data = (int64_t)IO_inputs_;
                }
                IO_inputs_pub_->unlockAndPublish();

                if (TOOL_IO_inputs_pub_->trylock())
                {
                    TOOL_IO_inputs_pub_->msg_.data = (int64_t)TOOL_IO_inputs_;
                }
                TOOL_IO_inputs_pub_->unlockAndPublish();

            }


            if (estop_pub_->trylock())
            {
                estop_pub_->msg_.data = estopped_;
            }
            estop_pub_->unlockAndPublish();

        }


        bool go_op_svc_cb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
        {
            auto robot_mode = robot_interface_->getRobotState()->getRobotModeType();

            if (robot_mode == RobotModeType::Running) {
                ROS_INFO("The robot arm has released the brake and is in running mode");

            } else {
                // Interface call: The robot arm initiates a power-on request
                // can except, not sure if thatll fail the service or take down the driver
                auto pret = robot_interface_->getRobotManage()->poweron();
                if (pret == AUBO_BAD_STATE)
                {
                    ROS_ERROR("[AUBO HW] failed to poweron");
                    return(false);
                }

                // Wait for the robot arm to enter idle mode
                waitForRobotMode(RobotModeType::Idle);

                ROS_INFO("The robotic arm is powered on successfully, current mode: %d", (int) robot_interface_->getRobotState()->getRobotModeType() );

                // Interface call: The robot arm initiates a brake release request
                robot_interface_->getRobotManage()->startup();

                // Wait for the robot arm to enter running mode
                waitForRobotMode(RobotModeType::Running);

                ROS_INFO("The robot arm released the brake successfully, current mode: %d", (int) robot_interface_->getRobotState()->getRobotModeType());
            }
            return(true);
        }


        bool set_load_cb(aubo_msgs::SetPayloadRequest &req, aubo_msgs::SetPayloadResponse &res)
        {
            double mass = 0.0;
            std::vector<double> cog(3, 0.0);
            std::vector<double> aom(3, 0.0);
            std::vector<double> inertia(6, 0.0);
            cog = {req.center_of_gravity.x, req.center_of_gravity.y, req.center_of_gravity.z};
            mass = req.mass;
            int setload_ret;
            setload_ret = robot_interface_->getRobotConfig()->setPayload(mass, cog, aom, inertia);    //TODO: check ret
            if (setload_ret==0)
            {
                res.success = true;
            }
            else
            {
                ROS_ERROR("Could not set payload. Error Code: %d",setload_ret);
                res.success = false;
            }
            
            return(res.success);
        }
        
        bool set_io_output_cb(aubo_msgs::SetIORequest &req, aubo_msgs::SetIOResponse &res)
        {
            if(req.fun==req.FUN_SET_DIGITAL_OUT)
            {
                robot_interface_->getIoControl()->setStandardDigitalOutput(req.pin, req.state);
            }
            return(true);
        }
        
};



int main(int argc, char** argv){
    
    //set logging level to DEBUG
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug)){
        //ros::console::notifyLoggerLevelsChanged();
    //}

    ros::Time last_low_freq_pub_time;

    ros::init(argc, argv, "aubo_hw_interface");


    ROS_DEBUG("[AUBO HW] aubo interface node enter");
    
    ros::NodeHandle nh; // when launching from a launch file
    AuboController hw = AuboController(nh);

    ros::Publisher hb_pub = nh.advertise<std_msgs::UInt64>("/robot_heartbeat", 1);
    std_msgs::UInt64 hb_msg;
    hb_msg.data=0;

    if(hw.getState() == ST_FINAL || hw.getState() == ST_ERROR)
        return -5;

    std::cout<<"[AUBO HW] created STATE: " << hw.getState() << std::endl;
    //init
    if(hw.configure()!=0){
        //TODO: handle if returned false
        return -1; // exit
    }

    if(hw.getState() != ST_INACTIVE){
        ROS_ERROR("[AUBO HW] could not completely configure robot. Terminating");
        return -6;
    }
    std::cout<<"[AUBO HW] configured STATE: " << hw.getState() << std::endl;
    

    //make a controller manager (ONLY CREATE THIS WHEN THE HW IS READY TO GO)
    controller_manager::ControllerManager cm(&hw, nh);
    //start non "RT" thread (cause this one can get locked)
    ros::AsyncSpinner spinner(1);
    spinner.start();


    ros::Rate _rate(hw.getControlHz()); //control_hz_ variable is set by class in init()
    ros::Time now = ros::Time::now();
    ros::Time last;
    ros::Duration dt;

    std::cout<<"[AUBO HW] <frequency> " << hw.getControlHz() <<std::endl;

    //ENABLE TORQUE
    if(hw.activate() != 0){
        //Bail for now. should be able to kick over into inactive mode and recover.
        return -1;
    }
    
    int ret;
    last_low_freq_pub_time = ros::Time::now();
    ROS_INFO("[AUBO HW] ENTERING MAIN CONTROL LOOP");
    while(ros::ok()){
        
        last = now;
        now = ros::Time::now();
        dt = now-last;

        hw.read(dt);
        cm.update(now,dt);
        ret = hw.write(dt);
        // catch the case where servoj commands were interrupted and the robot falls out of servoj mode
        if (ret==-13)
        {
            hw.activate();
        }

        // maintain robot heartbeat
        hb_msg.data++;
        hb_pub.publish(hb_msg);

        // maintain robot ancillary data
        if((now - last_low_freq_pub_time) > ros::Duration((1.0/20.0)))
        {
            hw.publish_ancillary_data();
            last_low_freq_pub_time += ros::Duration((1.0/20.0));
        }

        _rate.sleep();

    }//-----------------------------------------------------------


    //shutdown and clean up

    //can roll this into shutdown() if you want
    if(hw.deactivate() != 0){
        //TODO: handle
        if(hw.procError() != 0){
            hw.errorShutdown();
            return -3;
        }
        if(hw.deactivate() != 0){
            hw.errorShutdown();
            return -2;
        }
    }

    if(!hw.shutdown()){
        //TODO: handle
        hw.errorShutdown();
        return -1;
    }

    ROS_DEBUG("[AUBO HW] exit");

    return 0;




}
