#include <z1_ros_control/z1_ros_control.hpp>

void Z1Robot::gripperCB(const control_msgs::GripperCommandGoalConstPtr& msg)
{
    gripper_position_cmd = msg->command.position;
    gripper_effort_cmd = msg->command.max_effort;

    ros::Rate r(5);
    bool success = true;

    while(true){
        if (gripper_as->isPreemptRequested() || !ros::ok())
        {
            gripper_as->setPreempted();
            success = false;
            break;
        }

        double G_Q = arm->lowstate->getGripperQ();
        double G_Qd = arm->lowstate->getGripperQd();
        double G_Tau = arm->lowstate->getGripperTau();

        gripper_feedback.position = G_Q;
        gripper_feedback.effort = G_Tau;

        gripper_feedback.reached_goal = false;
        gripper_feedback.stalled = false;

        if(abs(gripper_position_cmd - G_Q) < 0.01){
            gripper_feedback.reached_goal = true;
        }
        else{
            if(abs(G_Qd) < 0.01){
                gripper_feedback.stalled = true;
            }
        }

        gripper_as->publishFeedback(gripper_feedback);

        if(gripper_feedback.reached_goal){
            gripper_result.position = gripper_feedback.position;
            gripper_result.effort = gripper_feedback.effort;
            gripper_feedback.reached_goal = true;
            gripper_feedback.stalled = false;
            gripper_as->setSucceeded(gripper_result);
            gripper_as_active = false;
            break;
        }

        r.sleep();
    }
}

void Z1Robot::read(const ros::Time& time, const ros::Duration& period)
{
    Vec6 Q = arm->lowstate->getQ();
    Vec6 Qd = arm->lowstate->getQd();
    Vec6 Tau = arm->lowstate->getTau();

    for(int i = 0; i < 6; i++){
        pos[i] = Q[i];
        vel[i] = Qd[i];
        eff[i] = Tau[i];
    }

    if(has_gripper){
        double G_Q = arm->lowstate->getGripperQ();
        double G_Qd = arm->lowstate->getGripperQd();
        double G_Tau = arm->lowstate->getGripperTau();

        pos[6] = G_Q;
        vel[6] = G_Qd;
        eff[6] = G_Tau;
    }
}

void Z1Robot::write(const ros::Time& time, const ros::Duration& period)
{
    for(int i = 0; i < 6; i++){
        arm->q(i) = cmd[i];
    }

    arm->tau = arm->_ctrlComp->armModel->inverseDynamics(arm->q, Vec6::Zero(), Vec6::Zero(), Vec6::Zero());

    if(has_gripper){
        arm->gripperQ = gripper_position_cmd;
        arm->gripperTau = gripper_effort_cmd;
    }

    arm->sendRecv();
}

void Z1Robot::init(){
    ROS_INFO("Initializing...");

    arm->sendRecvThread->start();

    ros::Duration(0.1).sleep();

    ROS_INFO("Checking arm state...");
    if(!arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD)){
        ROS_INFO("Setting State to Passive...");
        arm->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
        ROS_INFO("Setting State to Low-Level Control...");
        arm->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
    }
    ROS_INFO("Arm in Low-Level Control");

    arm->_ctrlComp->lowcmd->setControlGain();
    if(has_gripper){
        arm->_ctrlComp->lowcmd->setGripperGain();
    }

    ROS_INFO("Control Initialized...");
}

void Z1Robot::dinit(){
    arm->sendRecvThread->shutdown();
}

Z1Robot::Z1Robot(ros::NodeHandle& nh)
{
    _nh = &nh;
    nh.param<bool>("UnitreeGripper", has_gripper, true);

    nh.param<std::string>("udp_to_sdk/sdk_ip", hostname, "127.0.0.1");
    nh.param<int>("udp_to_sdk/controller_port", controller_port_sdk, 8072);

    ROS_INFO("%s", hostname.c_str());

    if(has_gripper){
        pos = new double[7];
        vel = new double[7];
        eff = new double[7];
        ROS_INFO("################ 6-Axis Arm with Gripper ################");
    }
    else{
        pos = new double[6];
        vel = new double[6];
        eff = new double[6];
        ROS_INFO("################ 6-Axis Arm ################");
    }   

    hardware_interface::JointStateHandle state_handle_1("joint1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_1);
    hardware_interface::JointStateHandle state_handle_2("joint2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_2);
    hardware_interface::JointStateHandle state_handle_3("joint3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_3);
    hardware_interface::JointStateHandle state_handle_4("joint4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_4);
    hardware_interface::JointStateHandle state_handle_5("joint5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_5);
    hardware_interface::JointStateHandle state_handle_6("joint6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_6);

    if(has_gripper){
        hardware_interface::JointStateHandle state_handle_g("jointGripper", &pos[6], &vel[6], &eff[6]);
        jnt_state_interface.registerHandle(state_handle_g);
    }

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint1"), &cmd[0]);
    arm_pos_interface.registerHandle(pos_handle_1); 
    hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("joint2"), &cmd[1]);
    arm_pos_interface.registerHandle(pos_handle_2); 
    hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("joint3"), &cmd[2]);
    arm_pos_interface.registerHandle(pos_handle_3); 
    hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("joint4"), &cmd[3]);
    arm_pos_interface.registerHandle(pos_handle_4); 
    hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("joint5"), &cmd[4]);
    arm_pos_interface.registerHandle(pos_handle_5); 
    hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("joint6"), &cmd[5]);
    arm_pos_interface.registerHandle(pos_handle_6);

    registerInterface(&arm_pos_interface);

    ctrlComp = new UNITREE_ARM::CtrlComponents();
    ctrlComp->dt = 0.002;//500HZ

    ctrlComp->udp = new UNITREE_ARM::UDPPort(hostname, 8071, controller_port_sdk, UNITREE_ARM::RECVSTATE_LENGTH, UNITREE_ARM::BlockYN::NO, 500000);

    if(has_gripper) {
        ctrlComp->armModel = new UNITREE_ARM::Z1Model(Vec3(0.0382, 0.0, 0.0), 0.80225,
            Vec3(0.0037, 0.0014, -0.0003), Vec3(0.00057593, 0.00099960, 0.00106337).asDiagonal());
    } else {
        ctrlComp->armModel = new UNITREE_ARM::Z1Model();
    }

    ctrlComp->armModel->addLoad(0.03);// add 0.03kg payload to the end joint
    arm = new UNITREE_ARM::unitreeArm(ctrlComp);

    gripper_as = new actionlib::SimpleActionServer<control_msgs::GripperCommandAction>("gripper_controller", boost::bind(&Z1Robot::gripperCB, this, _1), false);
    gripper_as->start();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "z1_ros_control");
    ros::NodeHandle nh("~");

    Z1Robot robot(nh);

    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    robot.init();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(250.0);
    
    while (ros::ok())
    {
        const ros::Time time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        
        robot.read(time, period);
        cm.update(time, period);
        robot.write(time, period);
        
        rate.sleep();
    }

    robot.dinit();
    
    return 0;
}
