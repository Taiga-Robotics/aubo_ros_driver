#include "aubo_hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
namespace aubo_driver {
AuboHardwareInterface::AuboHardwareInterface()
{
}

AuboHardwareInterface::AuboHardwareInterface(ros::NodeHandle &nh) : nh_(nh)
{
    printf("[HW] constructor\n");
}

bool AuboHardwareInterface::init(ros::NodeHandle &root_nh,
                                 ros::NodeHandle &robot_hw_nh)
{
    printf("[HW] init\n");
    if (!nh_.getParam("robot_ip", robot_ip_)) {
        ROS_ERROR_STREAM("Required parameter " << nh_.resolveName("robot_ip")
                                               << " not given." << robot_ip_);
        return false;
    }

    printf("[HW] init - make_shared<RpcClient>\n");
    rpc_client_ = std::make_shared<RpcClient>();

    printf("[HW] init - setRequestTimeout(1000);\n");
    rpc_client_->setRequestTimeout(1000);
    // 接口调用: 连接到 RPC 服务
    printf("[HW] init - connect\n");
    rpc_client_->connect(robot_ip_, 30004);
    // 接口调用: 登录
    printf("[HW] init - login\n");
    rpc_client_->login("aubo", "123456");

    printf("[HW] init - make_shared<RtdeClient>\n");
    rtde_client_ = std::make_shared<RtdeClient>();
    // 接口调用: 连接到 RTDE 服务
    printf("[HW] init - connect\n");
    rtde_client_->connect(robot_ip_, 30010);
    // 接口调用: 登录
    printf("[HW] init - login\n");
    rtde_client_->login("aubo", "123456");
    printf("[HW] init - rtde_client_->setTopic\n");
    int topic = rtde_client_->setTopic(false, { "R1_message" }, 200, 0);
    if (topic < 0) {
        std::cout << "Set topic fail!" << std::endl;
    }
    printf("[HW] init - rtde_client_->subscribe\n");
    rtde_client_->subscribe(topic, [](InputParser &parser) {
        arcs::common_interface::RobotMsgVector msgs;
        msgs = parser.popRobotMsgVector();
        for (size_t i = 0; i < msgs.size(); i++) {
            auto &msg = msgs[i];
        }
    });


    
    // 设置rtde输入
    printf("[HW] init - setInput(rtde_client_)\n");
    setInput(rtde_client_);

    // 配置输出
    printf("[HW] init - configSubscribe(rtde_client_)\n");
    configSubscribe(rtde_client_);
    if (!nh_.getParam("/aubo_hardware_interface/joints", joint_names_)) {
        ROS_ERROR_STREAM("Cannot find required parameter "
                         << nh_.resolveName("joints")
                         << " on the parameter server.");
        throw std::runtime_error(
            "Cannot find required parameter "
            "'controller_joint_names' on the parameter server.");
    }
    // Create ros_control interfaces
    printf("[HW] init - ros control handles\n");
    for (std::size_t i = 0; i < actual_q_.size(); ++i) {
        ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);
        // Create joint state interface for all joints
        js_interface_.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[i], &actual_q_copy_[i], &actual_qd_[i],
            &actual_current_[i]));
        ROS_INFO("joint_names_[i]:%s", joint_names_[i].data());
        // Create joint position control interface
        printf("[HW] init - interface\n");
        pj_interface_.registerHandle(hardware_interface::JointHandle(
            js_interface_.getHandle(joint_names_[i]),
            &joint_position_command_[i]));
        //        spj_interface_.registerHandle(scaled_controllers::ScaledJointHandle(
        //            js_interface_.getHandle(joint_names_[i]),
        //            &joint_position_command_[i], &speed_scaling_combined_));
    }


    printf("[HW] init - register\n");
    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);

    // registerInterface(this);

    return true;
}

void AuboHardwareInterface::read(const ros::Time &time,
                                 const ros::Duration &period)
{
    printf("[HW] read - \n");
    readActualQ();
}

void AuboHardwareInterface::write(const ros::Time &time,
                                  const ros::Duration &period)
{
    printf("[HW] write - \n");
    std::cout<<"[HW] joins as they would be commanded ::: \n[";
    for(int i=0; i< 6; i++){
        std::cout<<joint_position_command_[i]<<", ";
    }
    std::cout<<"]\n";

    // can do speed mode with speedJoint() cmd

    try {
            if(position_controller_running_){
                writeret_ = Servoj(joint_position_command_);
            }
        } catch (const std::exception &e) {
        }
    

}

bool AuboHardwareInterface::isServoModeStart()
{
    printf("[HW] isServoModeStart() - \n");
    return servo_mode_start_;
}

int AuboHardwareInterface::startServoMode()
{
    printf("[HW] startServoMode()- \n");
    if (servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();
    std::cout<<"[HW] startServoMode() - got robot name: " << robot_name <<std::endl;
    rpc_client_->getRobotInterface(robot_name)->getMotionControl()->setSpeedFraction(0.3);

    //开启servo模式
    printf("[HW] startServoMode - getRobotInterface\n");
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(true);
    int i = 0;
    while (!rpc_client_->getRobotInterface(robot_name)
                ->getMotionControl()
                ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode enable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    printf("[HW] startServoMode -servo mode is ENABLED\n");
    servo_mode_start_ = true;
    return 0;
}

int AuboHardwareInterface::stopServoMode()
{
    printf("[HW] stopServoMode -\n");
    if (!servo_mode_start_) {
        return 0;
    }
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();

    while (!rpc_client_->getRobotInterface(robot_name)
                ->getRobotState()
                ->isSteady()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // 关闭servo模式
    int i = 0;
    rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->setServoMode(false);
    while (rpc_client_->getRobotInterface(robot_name)
               ->getMotionControl()
               ->isServoModeEnabled()) {
        if (i++ > 5) {
            std::cout << "Servo Mode disable fail! Servo Mode is "
                      << rpc_client_->getRobotInterface(robot_name)
                             ->getMotionControl()
                             ->isServoModeEnabled()
                      << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    printf("[HW] stopServoMode - servo mode is DISABLED\n");
    std::cout << "Servoj end" << std::endl;
    servo_mode_start_ = false;
    return 0;
}

int AuboHardwareInterface::Servoj(
    const std::array<double, 6> joint_position_command)
{
    printf("[HW] ServoJ - \n");
    int ret;
    // 接口调用 : 获取机器人的名字
    auto robot_name = rpc_client_->getRobotNames().front();
    std::cout<<"[HW] ServoJ - rgetRobotNames().front()"<<robot_name<<std::endl;

    std::cout<<"[HW] ServoJ :::\n[";
    std::vector<double> traj(6, 0);
    for (size_t i = 0; i < traj.size(); i++) {
        traj[i] = joint_position_command[i];
        std::cout<<traj[i]<<", ";
    }
    std::cout<<"]"<<std::endl;

    // 接口调用: 关节运动
    printf("[HW] ServoJ - servoJoint(...)\n");
    ret = rpc_client_->getRobotInterface(robot_name)
        ->getMotionControl()
        ->servoJoint(traj, 0.2, 0.2, 0.005, 0.1, 200);
    // std::this_thread::sleep_for(std::chrono::milliseconds(25));

    //    std::cout << "servoJoint finish!" << std::endl;

    return ret;
}

bool AuboHardwareInterface::prepareSwitch(
    const std::list<hardware_interface::ControllerInfo> &start_list,
    const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    printf("[HW] prepareSwitch() - \n");
    bool ret_val = true;
    // block below prevents controller switching after first pass. I dont know why it would do that.
    // if (controllers_initialized_ && !isRobotProgramRunning() &&
    //     !start_list.empty()) {
    //     for (auto &controller : start_list) {
    //         if (!controller.claimed_resources.empty()) {
    //             ROS_ERROR_STREAM(
    //                 "Robot control is currently inactive. Starting controllers "
    //                 "that claim resources is currently "
    //                 "not possible. Not starting controller '"
    //                 << controller.name << "'");
    //             ret_val = false;
    //         }
    //     }
    // }

    controllers_initialized_ = true;
    return ret_val;
}

void AuboHardwareInterface::doSwitch(
    const std::list<hardware_interface::ControllerInfo> &start_list,
    const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    printf("[HW] doSwitich -\n");
    for (auto &controller_it : stop_list) {
        for (auto &resource_it : controller_it.claimed_resources) {
            if (checkControllerClaims(resource_it.resources)) {
                if (resource_it.hardware_interface ==
                    "scaled_controllers::ScaledPositionJointInterface") {
                    position_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::PositionJointInterface") {
                    position_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "scaled_controllers::ScaledVelocityJointInterface") {
                    velocity_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::VelocityJointInterface") {
                    velocity_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::TrajectoryInterface<control_msgs::"
                    "FollowJointTrajectoryGoal_<std::allocator<void> >, "
                    "control_msgs::FollowJointTrajectoryFeedback_<std::"
                    "allocator<void> > >") {
                    joint_forward_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::TrajectoryInterface<cartesian_control_"
                    "msgs::"
                    "FollowCartesianTrajectoryGoal_<std::allocator<void> >, "
                    "cartesian_control_msgs::FollowCartesianTrajectoryFeedback_"
                    "<std::"
                    "allocator<void> > >") {
                    cartesian_forward_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "ros_controllers_cartesian::TwistCommandInterface") {
                    twist_controller_running_ = false;
                }
                if (resource_it.hardware_interface ==
                    "ros_controllers_cartesian::PoseCommandInterface") {
                    pose_controller_running_ = false;
                }
            }
        }
    }
    for (auto &controller_it : start_list) {
        for (auto &resource_it : controller_it.claimed_resources) {
            if (checkControllerClaims(resource_it.resources)) {
                if (resource_it.hardware_interface ==
                    "scaled_controllers::ScaledPositionJointInterface") {
                    position_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::PositionJointInterface") {
                    position_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "scaled_controllers::ScaledVelocityJointInterface") {
                    velocity_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::VelocityJointInterface") {
                    velocity_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::TrajectoryInterface<control_msgs::"
                    "FollowJointTrajectoryGoal_<std::allocator<void> >, "
                    "control_msgs::FollowJointTrajectoryFeedback_<std::"
                    "allocator<void> > >") {
                    joint_forward_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "hardware_interface::TrajectoryInterface<cartesian_control_"
                    "msgs::"
                    "FollowCartesianTrajectoryGoal_<std::allocator<void> >, "
                    "cartesian_control_msgs::FollowCartesianTrajectoryFeedback_"
                    "<std::"
                    "allocator<void> > >") {
                    cartesian_forward_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "ros_controllers_cartesian::TwistCommandInterface") {
                    twist_controller_running_ = true;
                }
                if (resource_it.hardware_interface ==
                    "ros_controllers_cartesian::PoseCommandInterface") {
                    pose_controller_running_ = true;
                }
            }
        }
    }
}

void AuboHardwareInterface::readActualQ()
{
    printf("[HW] readActualQ -\n");
    // 使用 actual_q_copy_
    // 固定该时间戳下read到的位姿，否则读取到的关节状态不稳定
    // actual_q_copy_必须用 array 否则会 bad_alloc
    {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_copy_[0] = actual_q_[0];
        actual_q_copy_[1] = actual_q_[1];
        actual_q_copy_[2] = actual_q_[2];
        actual_q_copy_[3] = actual_q_[3];
        actual_q_copy_[4] = actual_q_[4];
        actual_q_copy_[5] = actual_q_[5];
    }
}

bool AuboHardwareInterface::isRobotProgramRunning() const
{
    printf("[HW] isRobotProgramRunning -\n");
    return robot_program_running_;
}

bool AuboHardwareInterface::checkControllerClaims(
    const std::set<std::string> &claimed_resources)
{
    for (const std::string &it : joint_names_) {
        for (const std::string &jt : claimed_resources) {
            if (it == jt) {
                return true;
            }
        }
    }
    for (const std::string &jt : claimed_resources) {
        if ("ee_link" == jt) {
            return true;
        }
    }
    return false;
}

bool AuboHardwareInterface::shouldResetControllers()
{
    return false;
}

template <typename T>
void AuboHardwareInterface::printVec(std::vector<T> param, std::string name)
{
    std::cout << "@:" << name << std::endl;
    for (int i = 0; i < param.size(); i++) {
        std::cout << param.at(i) << ",";
    }
    std::cout << std::endl;
}

// 配置输出
void AuboHardwareInterface::configSubscribe(RtdeClientPtr cli)
{
    // 接口调用: 设置 topic1
    int topic1 = cli->setTopic(
        false,
        { "R1_actual_q", "R1_actual_qd", "R1_robot_mode", "R1_safety_mode",
          "runtime_state", "line_number", "R1_actual_TCP_pose" },
        500, 0);
    // 接口调用: 订阅
    cli->subscribe(topic1, [this](InputParser &parser) {
        std::unique_lock<std::mutex> lck(rtde_mtx_);
        actual_q_ = parser.popVectorDouble();
        actual_current_ = parser.popVectorDouble();
        robot_mode_ = parser.popRobotModeType();
        safety_mode_ = parser.popSafetyModeType();
        runtime_state_ = parser.popRuntimeState();
        line_ = parser.popInt32();
        actual_TCP_pose_ = parser.popVectorDouble();
    });
}

// 设置rtde输入
void AuboHardwareInterface::setInput(RtdeClientPtr cli)
{
    // 组合设置输入
    int topic5 = cli->setTopic(
        true,
        { "input_bit_registers0_to_31", "input_bit_registers32_to_63",
          "input_bit_registers64_to_127", "input_int_registers_0" },
        1, 5);

    std::vector<int> value = { 0x00ff, 0x00, 0x00, 44 };
    cli->publish(
        5, [value](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value); });

    int topic6 = cli->setTopic(
        true, { "input_float_registers_0", "input_double_registers_1" }, 1, 6);

    std::vector<double> value2 = { 3.1, 4.1 };
    cli->publish(
        6, [value2](arcs::aubo_sdk::OutputBuilder &ro) { ro.push(value2); });
}

} // namespace aubo_driver
PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboHardwareInterface,
                       hardware_interface::RobotHW)
