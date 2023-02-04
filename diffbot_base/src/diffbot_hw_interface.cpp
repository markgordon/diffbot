#include <diffbot_base/diffbot_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
 
namespace diffbot_base
{
    DiffBotHWInterface::DiffBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh)
    { 
        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/wheel_radius", wheel_radius_);
        error += !rosparam_shortcuts::get(name_, nh_, "mobile_base_controller/linear/x/max_velocity", max_velocity_);
        // Get additional parameters from the diffbot_base/config/base.yaml which is stored on the parameter server
        error += !rosparam_shortcuts::get(name_, nh_, "encoder_resolution", encoder_resolution_);
        error += !rosparam_shortcuts::get(name_, nh_, "gain", gain_);
        error += !rosparam_shortcuts::get(name_, nh_, "trim", trim_);
        error += !rosparam_shortcuts::get(name_, nh_, "motor_constant", motor_constant_);
        error += !rosparam_shortcuts::get(name_, nh_, "pwm_limit", pwm_limit_);
        error += !rosparam_shortcuts::get(name_, nh_, "debug/hardware_interface", debug_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        wheel_diameter_ = 2.0 * wheel_radius_;
        //max_velocity_ = 0.2; // m/s
        // ros_control RobotHW needs velocity in rad/s but in the config its given in m/s
        max_velocity_ = linearToAngular(max_velocity_);


        ROS_INFO_STREAM("mobile_base_controller/wheel_radius: " << wheel_radius_);
        ROS_INFO_STREAM("mobile_base_controller/linear/x/max_velocity: " << max_velocity_);
        ROS_INFO_STREAM("encoder_resolution: " << encoder_resolution_);
        ROS_INFO_STREAM("gain: " << gain_);
        ROS_INFO_STREAM("trim: " << trim_);
        ROS_INFO_STREAM("motor_constant: " << motor_constant_);
        ROS_INFO_STREAM("pwm_limit: " << pwm_limit_);

        // Setup publisher for the motor driver 
       // pub_left_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_left", 10);
       // pub_right_motor_value_ = nh_.advertise<std_msgs::Int32>("motor_right", 10);

        //Setup publisher for angular wheel joint velocity commands
       // pub_wheel_cmd_velocities_ = nh_.advertise<diffbot_msgs::WheelsCmdStamped>("wheel_cmd_velocities", 10);

        // Setup publisher to reset wheel encoders (used during first launch of the hardware interface)
       // pub_reset_encoders_ = nh_.advertise<std_msgs::Empty>("reset", 10);
        // Setup subscriber for the wheel encoders
       // sub_encoder_ticks_ = nh_.subscribe("encoder_ticks", 10, &DiffBotHWInterface::encoderTicksCallback, this);
       // sub_measured_joint_states_ = nh_.subscribe("measured_joint_states", 10, &DiffBotHWInterface::measuredJointStatesCallback, this);

        // Initialize the hardware interface
        init(nh_, nh_);

        // Wait for encoder messages being published
       // isReceivingMeasuredJointStates(ros::Duration(10));
    }

 
    bool DiffBotHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing DiffBot Hardware Interface ...");
        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        std::array<std::string, NUM_JOINTS> motor_names = {"left_motor", "right_motor"};
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointVelocityInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_velocity_commands_[i]);
            velocity_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0; // unused with diff_drive_controller

            joint_velocity_commands_[i] = 0.0;

            // Initialize encoder_ticks_ to zero because receiving meaningful
            // tick values from the microcontroller might take some time
            encoder_ticks_[i] = 0.0;
            measured_joint_states_[i].angular_position_ = 0.0;
            measured_joint_states_[i].angular_velocity_ = 0.0;

            // Initialize the pid controllers for the motors using the robot namespace
            std::string pid_namespace = "pid/" + motor_names[i];
            ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            ros::NodeHandle nh(root_nh, pid_namespace);
            // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            //pids_[i].init(nh, 0.8, 0.35, 0.5, 0.01, 3.5, -3.5, false, max_velocity_, -max_velocity_);
            //pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&velocity_joint_interface_);
    ROS_INFO("open port");

    std::string port1 = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_54D2035530-if00";
    ddms_diff::return_type ret = wheel_command[0].open(port1);
    if(ret != ddms_diff::return_type::SUCCESS)
    {
        ROS_FATAL("DDSM Couldn't open port %s, code %i",port1.c_str(),(int)ret);
        return false;
    }
  
    std::string port2 = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9XOMIL6-if00-port0";
    ret = wheel_command[1].open(port2);

    if(ret != ddms_diff::return_type::SUCCESS)
    {
        ROS_FATAL("DDSM Couldn't open port %s, code %i",port2.c_str(),(int)ret);
        return false;
    }
        ROS_INFO("... Done Initializing DiffBot Hardware Interface");




        return true;
    }

    void DiffBotHWInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
    {

        for(int i=0;i<num_joints_;i++){
            std::vector<double> state;
            //re-use last rpm, since we have to update to get detailed info
            ddms_diff::return_type retval = wheel_command[i].get_wheel_state(i+1,last_hw_commands_[i],state);
            if(retval != ddms_diff::return_type::SUCCESS){
                ROS_FATAL("DDSM Read joint fail");
                return;
            }
            //states may be empty if there was a non-fatal read error
            if(state.size() > 1){
            //if this is first measurement the wheels are at starting position
            if(last_angle_[i] == -1)last_angle_[i] = round(state[1]*1000)/1000.0;
            else{
                if(round(state[0]) !=0){
                    //create absolute encoder from relative
                    state[1] = round(state[1]*1000)/1000.0;
                    double delta=state[1] - last_angle_[i];
                    //rotating backwards over last interval
                    if(state[0] < 0){
                    if (last_angle_[i] - state[1] < -M_PI){
                        delta = -(2*M_PI - state[1] + last_angle_[i]);
                    }
                    }else if(state[0] > 0){
                    if (last_angle_[i] - state[1] > M_PI){
                        delta = round((2*M_PI - last_angle_[i] + state[1])*1000)/1000.0;
                    }
                    }
                // RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "id %d last %f :current %f sp %f",i,last_angle_[i] ,state[1],state[0],delta);

                    current_wheel_position_[i]+=delta;
                }
                last_angle_[i] = state[1];

            }
            //RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "pos %f",current_wheel_position_[i]);

            joint_velocities_[i] = state[0];
            joint_positions_[i]  = current_wheel_position_[i];
            joint_efforts_[i] = 0.0;
            //last_state_[i][0] = state[0];
            //last_state_[i][1] = state[1];
            }else{
                joint_positions_[i] = current_wheel_position_[i];
            }
        }

    }

    void DiffBotHWInterface::write(const ros::Time& time, const ros::Duration& /*period*/)
    {
        for(int i = 0;i<num_joints_;i++){
            last_hw_commands_[i] = joint_velocity_commands_[0];
        }

    }

    bool DiffBotHWInterface::isReceivingMeasuredJointStates(const ros::Duration &timeout)
    {
        ROS_INFO("Get number of measured joint states publishers");

        ros::Time start = ros::Time::now();
        int num_publishers = sub_measured_joint_states_.getNumPublishers();
        ROS_INFO("Waiting for measured joint states being published...");
        while ((num_publishers == 0) && (ros::Time::now() < start + timeout))
        {
            ros::Duration(0.1).sleep();
            num_publishers = sub_measured_joint_states_.getNumPublishers();
        }
        if (num_publishers == 0)
        {
            ROS_ERROR("No measured joint states publishers. Timeout reached.");
        }
        else
        {
            ROS_INFO_STREAM("Number of measured joint states publishers: " << num_publishers);
        }

        ROS_INFO("Publish /reset to encoders");
        std_msgs::Empty msg;
        pub_reset_encoders_.publish(msg);

        return (num_publishers > 0);
    }

    void DiffBotHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

    void DiffBotHWInterface::printState()
    {
        // WARNING: THIS IS NOT REALTIME SAFE
        // FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
        ROS_INFO_STREAM_THROTTLE(1, std::endl << printStateHelper());
    }

    std::string DiffBotHWInterface::printStateHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);

        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << "j" << i << ": " << std::fixed << joint_positions_[i] << "\t ";
            ss << std::fixed << joint_velocities_[i] << "\t ";
            ss << std::fixed << joint_efforts_[i] << std::endl;
        }
        return ss.str();
    }

    std::string DiffBotHWInterface::printCommandHelper()
    {
        std::stringstream ss;
        std::cout.precision(15);
        ss << "    position     velocity         effort  \n";
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            ss << std::fixed << joint_velocity_commands_[i] << "\t ";
        }
        return ss.str();
    }


    double DiffBotHWInterface::normalizeAngle(double &angle) const
    {
        // https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
        angle = fmod(angle, 2.0*M_PI);

        if (angle < 0)
            angle += 2.0*M_PI;

        ROS_DEBUG_STREAM_THROTTLE(1, "Normalized angle: " << angle);
        return angle;
    }


    double DiffBotHWInterface::linearToAngular(const double &distance) const
    {
        return distance / wheel_diameter_ * 2.0;
    }

    double DiffBotHWInterface::angularToLinear(const double &angle) const
    {
        return angle * wheel_diameter_ / 2.0;
    }

};
