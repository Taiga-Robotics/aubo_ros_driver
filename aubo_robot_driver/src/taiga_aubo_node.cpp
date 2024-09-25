#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <ros_control_hw_interface/IROSHardware.h>

// AUBO SDK
#include "aubo_sdk/rpc.h"
#include "aubo_sdk/rtde.h"
using namespace arcs::aubo_sdk;
using namespace arcs::common_interface;

// my fake class:
class AuboController : public IROSHardware
{
    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > cmd_out_;

    public:
        JointCmdMode robot_cmd_mode_ = MD_NONE;
        
        std::shared_ptr<RpcClient> rpc_cli;
        std::shared_ptr<RtdeClient> rtde_client_;
        std::string robot_ip_ = "192.168.100.3";
        std::string robot_name_;
        RobotInterfacePtr robot_interface_;

        // RTDE subscriber data:
        std::mutex rtde_mtx_;
        std::vector<double> actual_q_{ std::vector<double>(6, 0) };
        std::vector<double> actual_current_{ std::vector<double>(6, 0.1) };
        std::vector<double> actual_TCP_pose_{ std::vector<double>(6, 0.) };
        RobotModeType robot_mode_ = RobotModeType::NoController;
        SafetyModeType safety_mode_ = SafetyModeType::Normal;
        RuntimeState runtime_state_ = RuntimeState::Stopped;
        int line_{ -1 };

        //other
        std::vector<double> actual_qd_{ std::vector<double>(6, 0) };
        std::vector<double> target_q_{ std::vector<double>(6, 0) };
        std::vector<double> target_qd_{ std::vector<double>(6, 0) };
        std::vector<double> actual_current_e{ std::vector<double>(6, 0) };
        std::vector<double> actual_TCP_speed_{ std::vector<double>(6, 0.) };
        std::vector<double> actual_TCP_force_{ std::vector<double>(6, 0.) };
        std::vector<double> target_TCP_pose_{ std::vector<double>(6, 0.) };
        std::vector<double> target_TCP_speed_{ std::vector<double>(6, 0.) };



        AuboController(ros::NodeHandle &node_handle): IROSHardware(node_handle) // example of how to call a specific parent constructor
        {

            //don't waste time if we won't be able to configure this anyways
            //base class errors shoudl be informative enough
            if(hw_state_== ST_ERROR || hw_state_ == ST_FINAL)
                return;

            control_hz_ = 500;

            //make w/e you need to here.
            std::cout<<"Fake Controller Constructor\n";
            std::cout<<"----------------------------\n";
            std::cout<<"num joints: " << num_joints_<<std::endl;
            std::cout<<"frequency: " << control_hz_<<std::endl;
            std::cout<<"----------------------------\n";

            // realtime publisher
            cmd_out_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(node_handle, "fake_hw_cmd_out", 1));
            cmd_out_->msg_.data.resize(num_joints_);

            //not readu at the end, because it needs to init--> recieve some data, and wait for controllers to load.
            hw_state_ = ST_NOT_READY;

        }
        ~AuboController(){};


        /*
            READ()
        */
        int read(ros::Duration &dt){

            {
                std::unique_lock<std::mutex> lck(rtde_mtx_);
                joint_pos_[0] = actual_q_[0];
                joint_pos_[1] = actual_q_[1];
                joint_pos_[2] = actual_q_[2];
                joint_pos_[3] = actual_q_[3];
                joint_pos_[4] = actual_q_[4];
                joint_pos_[5] = actual_q_[5];
            }
			data_valid_ = true;
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
            }
            else
            {
                ROS_DEBUG_THROTTLE(10, "[HW] [write] Unsupported Mode");
            }

            //publish the cmd that we received at full loop rate for now.
            if (cmd_out_->trylock()){
                for(int jid=0; jid<num_joints_; jid++) cmd_out_->msg_.data[jid]=joint_pos_cmd_[jid];
            }
            cmd_out_->unlockAndPublish();

            return ret;
        };


        /*
            CONFIGURE()
        */
        int configure()
        {
            // set up an RPC client
            rpc_cli = std::make_shared<RpcClient>();
            rpc_cli->setRequestTimeout(1000);
            rpc_cli->connect(robot_ip_, 30004);
            rpc_cli->login("aubo", "123456");   //no evidence the u/p combo is sent to the robot at any time

            // set up an RTDE client
            rtde_client_ = std::make_shared<RtdeClient>();
            rtde_client_->connect(robot_ip_, 30010);
            rtde_client_->login("aubo", "123456");
            int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
            if (topic < 0) {
                std::cout << "Set topic fail!" << std::endl;
            }

            //this subscribes to R1_message and then unloads it from the queue with no effect on reception. i dont know if this is required.
/*            rtde_client_->subscribe(topic, [](InputParser &parser) {
                arcs::common_interface::RobotMsgVector msgs;
                msgs = parser.popRobotMsgVector();

                //TODO: nuke below
                for (size_t i = 0; i < msgs.size(); i++) {
                    auto &msg = msgs[i];
                }
                //TODO: nuke above
            });
*/

            //not clear if this is required for joint state subscription, i iwll leave it out for now.
            // setInput(rtde_client_);  

            
            int topic1 = rtde_client_->setTopic(false,
                { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
                "runtime_state", "line_number", "R1_actual_TCP_pose" },
                control_hz_, 0);

            rtde_client_->subscribe(topic1, [this](InputParser &parser) 
                {
                    std::unique_lock<std::mutex> lck(rtde_mtx_);
                    actual_q_ = parser.popVectorDouble();
                    actual_current_ = parser.popVectorDouble();
                    robot_mode_ = parser.popRobotModeType();
                    safety_mode_ = parser.popSafetyModeType();
                    runtime_state_ = parser.popRuntimeState();
                    line_ = parser.popInt32();
                    actual_TCP_pose_ = parser.popVectorDouble();
                });

            // get RPC robot interface handle
            robot_name_ = rpc_cli->getRobotNames().front();
            robot_interface_ = rpc_cli->getRobotInterface(robot_name_);
            
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
            bool result = IROSHardware::prepareSwitch(start_list, stop_list);
            //robot_cmd_mode_ = joint_modes_[0]; // just set the robot control mode to be the mode of the first joint;
            //ROS_DEBUG("[HW] [prepareSwitch] - <ROBOT MODE: %d>", robot_cmd_mode_);

            return true;
        }


		void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
					const std::list<hardware_interface::ControllerInfo> &stop_list){
            robot_cmd_mode_ = joint_modes_[0]; // just set the robot control mode to be the mode of the first joint;
			ROS_DEBUG("[HW] [doSwitch]: ROBOT MODE: %d", robot_cmd_mode_);
			ROS_INFO("[HW] [doSwitch]: Switching Controllers . . . ");
		}


        /*
            ACTIVATE() - The main loop must start within 5 sampling cycles after calling activate()
        */
        int activate(){
            // begin servoj mode
            robot_interface_->getMotionControl()->setServoMode(true);
            int i = 0;
            while (!robot_interface_->getMotionControl()->isServoModeEnabled()) {
                if (i++ > 5) {  //failure condition
                    std::cout << "Failed to enable Servo mode! The current servo status is "
                              << rpc_cli->getRobotInterface(robot_name_)->getMotionControl()->isServoModeEnabled() << std::endl;
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

};


int main(int argc, char** argv){
    
//set logging level to DEBUG
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::Level::Debug)){
        //ros::console::notifyLoggerLevelsChanged();
    //}

    ros::init(argc, argv, "fake_hw_interface");


    ROS_DEBUG("[AUBO HW] fake interface node enter");
    
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
        ROS_ERROR("[AUBO HW] could not completley configure robot. Terminating");
        return -6;
    }
    std::cout<<"[AUBO HW] configured STATE: " << hw.getState() << std::endl;
    

    //make a controller manager (ONLY CREAT THIS WHEN THE HW IS READY TO GO)
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
        //TODO: handle
        return -1;
    }
    

    ROS_INFO("[AUBO HW] ENTERING MAIN CONTROL LOOP");
    while(ros::ok()){
        
        last = now;
        now = ros::Time::now();
        dt = now-last;

        hw.read(dt);
        cm.update(now,dt);
        hw.write(dt);

        hb_msg.data++;
        hb_pub.publish(hb_msg);

        _rate.sleep();

    }//-----------------------------------------------------------


    //shutdown and clean up

    //can roll thuis into shutdown() if you want
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
