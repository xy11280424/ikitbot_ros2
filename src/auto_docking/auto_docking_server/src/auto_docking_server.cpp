#include "auto_docking/auto_docking.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <numeric>
#include <ostream>
#include <stdlib.h>

AutoDockingAction::AutoDockingAction(std::string name, std::shared_ptr<tf2_ros::Buffer> tf)
: rclcpp::Node("auto_docking"),
  tf_buffer_(tf),
  action_name_(std::move(name)),
  virtual_bump_trigger_(false),
  real_bump_trigger_(false),
  bump_trigger_(false)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // 从 ROS2 参数服务器加载（你自己的实现里改用 this->declare/get_parameter）
  LoadParam();

  // ---- 订阅者 ----
  if (use_imu_angle_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 20, std::bind(&AutoDockingAction::Imu_CallBack, this, _1));
  }
  if (use_bms_state_) {
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/base_driver/battery_state", 10, std::bind(&AutoDockingAction::BatteryCallBack, this, _1));
  }
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 100, std::bind(&AutoDockingAction::Odom_CallBack, this, _1));
  robot_state_sub_ = this->create_subscription<robot_msg::msg::BaseInfo>(
      "/base_driver/base_info", 10, std::bind(&AutoDockingAction::RobotState_CallBack, this, _1));
  docker_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "docker_pose", 10, std::bind(&AutoDockingAction::dockerCallBack, this, _1));

  if (enable_sonar_) {
    sonars_sub_.push_back(
      this->create_subscription<sensor_msgs::msg::Range>(
        "/ultrasonic_back_right", 10, std::bind(&AutoDockingAction::SonarCallBacks, this, _1)));
    sonars_sub_.push_back(
      this->create_subscription<sensor_msgs::msg::Range>(
        "/ultrasonic_back_left", 10, std::bind(&AutoDockingAction::SonarCallBacks, this, _1)));
  }

  if (enable_line_laser_) {
    line_laser_sub_.push_back(
      this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_back", 10, std::bind(&AutoDockingAction::LineLaserCallBacks, this, _1)));
  }

  if (enable_real_bumper_) {
    sub_collision_bar_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/collision_bar", 10, std::bind(&AutoDockingAction::collisionBarCB, this, _1));
  } else {
    real_bump_trigger_ = false;
  }

  // ---- 发布者 ----
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // ---- Service Server （注意：ROS2 回调签名需改成(shared_ptr<Request>, shared_ptr<Response>)）----
  config_server_ = this->create_service<robot_msg::srv::SetDevice>(
      "/config/docking_forward",
      std::bind(&AutoDockingAction::configServer, this, _1, _2));

  // ---- 状态机初始化 ----
  //dockingmachine_ = new DOCKINGMACHINE(std::make_unique<InitState>(), tf_buffer_, shared_from_this()); 
  dockingmachine_ = std::make_unique<DOCKINGMACHINE>(
    std::make_unique<InitState>(),  // DOCKINGMACHINE 构造函数需要 std::unique_ptr<State>
    tf_buffer_,
    shared_from_this());
  dockingmachine_->max_back_dis_ = max_docking_distance_;
  dockingmachine_->forward_safety_dis_ = forward_safety_dis_;
  dockingmachine_->docking_forward_ = docking_forward_;
  dockingmachine_->sonar_obstacle_ = 0;
  dockingmachine_->line_laser_obstacle_ = 0;
  robot_vel_.linear.x = 0.0;
  robot_vel_.angular.z = 0.0;

  // ---- 相机控制 Clients ----
  if (docking_success_op_camera_up_) {
    camera_control_client_up_ =
      this->create_client<std_srvs::srv::SetBool>("/ascamera_nuwa_1/is_switch");
    RCLCPP_INFO(this->get_logger(), "Waiting for service /ascamera_nuwa_1/is_switch");
  }
  if (docking_success_op_camera_down_) {
    camera_control_client_down_ =
      this->create_client<std_srvs::srv::SetBool>("/ascamera_nuwa_2/is_switch");
    RCLCPP_INFO(this->get_logger(), "Waiting for service /ascamera_nuwa_2/is_switch");
  }

  // ---- Action Server ----
  action_server_ = rclcpp_action::create_server<AutoDockAction>(
    shared_from_this(),
    action_name_,
    std::bind(&AutoDockingAction::handle_goal, this,
              std::placeholders::_1, std::placeholders::_2),
    std::bind(&AutoDockingAction::handle_cancel, this,
              std::placeholders::_1),
    std::bind(&AutoDockingAction::handle_accepted, this,
              std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Action server [%s] started.", action_name_.c_str());
}

//******************** goal action *******************/
// 接收 goal
rclcpp_action::GoalResponse AutoDockingAction::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const robot_msg::action::AutoDock::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消回调
rclcpp_action::CancelResponse AutoDockingAction::handle_cancel(
    const std::shared_ptr<GoalHandleAutoDock> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 执行回调
void AutoDockingAction::handle_accepted(
    const std::shared_ptr<GoalHandleAutoDock> goal_handle)
{
    // 在新线程中执行，否则会阻塞 ROS2
    std::thread([this, goal_handle]() {
        this->execute(goal_handle);
    }).detach();
}

//***************************************************** */


AutoDockingAction::~AutoDockingAction() {
  RCLCPP_INFO(this->get_logger(), "Action server: %s, shutdown! ", action_name_.c_str());
}

void AutoDockingAction::Init(bool dockCtrl, bool touch) {
  dockingmachine_->docking_forward_ = docking_forward_;
  if (dockCtrl) {
    if (baseChargingState_) // 如果在充电中
    {
      dockingmachine_->setEvent(DOCKING_DONE);
      dockingmachine_->chargeState(std::make_unique<CHARGING_STATE>());
    } else {
      dockingmachine_->touch_ctrl_ = touch;
      dockingmachine_->pose_last_ = robot_pose_;
      dockingmachine_->setEvent(RECV_CHG_CMD);
      dockingmachine_->chargeState(std::make_unique<InitState>());
    }
  } else {
    if (baseChargingState_ == robot_msg::msg::BaseInfo::CHARGING ||
        baseChargingState_ == robot_msg::msg::BaseInfo::DOCK_IN ||
        baseChargingState_ == robot_msg::msg::BaseInfo::CHARGE_OK) {
      // 如果在充电中
      dockingmachine_->pose_last_ = robot_pose_;
      dockingmachine_->setEvent(RECV_SEPARATE_CMD);
      dockingmachine_->chargeState(std::make_unique<SEPARATE_STATE>());
    } else {

      dockingmachine_->pose_last_ = robot_pose_;
      dockingmachine_->setEvent(SEPARATE_STUCK);
      dockingmachine_->chargeState(std::make_unique<GOTO_SEPARATE_RECOVER_POINT_STATE>());
    }
  }
}

void AutoDockingAction::execute(const std::shared_ptr<GoalHandleAutoDock> goal_handle)
{
    auto goal = goal_handle->get_goal();
    geometry_msgs::msg::Twist vel;
    bool current_goal = goal->dock_ctrl;
    bool touch_ = goal->touch;
    rclcpp::Duration task_duration = rclcpp::Duration::from_seconds(300); // 5 min
    rclcpp::Time start_time = this->now();

    Init(current_goal, touch_);

    rclcpp::Rate rate(10); // 10Hz

    while (rclcpp::ok()) {
        // Check if goal was canceled
        if (goal_handle->is_canceling()) {
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            vel_pub_->publish(vel);
            auto feedback = std::make_shared<robot_msg::action::AutoDock::Feedback>();
            feedback->dock_feedback = robot_msg::action::AutoDock::Feedback::CANCLED;
            feedback->dock_feedback_text = "auto docking canceled!";
            goal_handle->publish_feedback(feedback);

            auto result = std::make_shared<robot_msg::action::AutoDock::Result>();
            result->dock_result = true;
            goal_handle->canceled(result);
            return;
        }

        // 状态机执行
        dockingmachine_->getState(feedback_.dock_feedback,
                                 feedback_.dock_feedback_text);

        auto feedback = std::make_shared<robot_msg::action::AutoDock::Feedback>();
        feedback->dock_feedback = feedback_.dock_feedback;
        feedback->dock_feedback_text = feedback_.dock_feedback_text;
        goal_handle->publish_feedback(feedback);

        dockingmachine_->ExeCycle(robot_pose_, vel);
        SafetyControl(vel, goal_handle);

        vel_pub_->publish(vel);

        // 成功判断
        if (feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::DOCKING_SUCCESS) {
            auto result = std::make_shared<robot_msg::action::AutoDock::Result>();
            result->dock_result = true;
            goal_handle->succeed(result);
            handleCameraAfterDocking(false); // 关闭相机
            RCLCPP_INFO(this->get_logger(), "---docking success");
            return;
        }

        // 分离成功
        if (feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::SEPERATING_DONE) {
            auto result = std::make_shared<robot_msg::action::AutoDock::Result>();
            result->dock_result = true;
            goal_handle->succeed(result);
            handleCameraAfterDocking(true); // 打开相机
            RCLCPP_INFO(this->get_logger(), "---separating success");
            return;
        }

        // 失败判断
        if (feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::DOCKING_FAILURE ||
            feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::ERROR_DOCKER_POS ||
            feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::SEARCHING_FAILURE ||
            feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::DOCKER_NO_POWER ||
            feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::OBSTACLE_FAILURE ||
            feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::DOCKER_OUT_OF_DISTANCE ||
            feedback_.dock_feedback == robot_msg::action::AutoDock::Feedback::SEPERAT_FAILURE)
        {
            auto result = std::make_shared<robot_msg::action::AutoDock::Result>();
            result->dock_result = false;
            goal_handle->succeed(result);
            RCLCPP_ERROR(this->get_logger(), "---auto docking failed");
            return;
        }

        // 超时处理
        if ((this->now() - start_time) > task_duration) {
            vel = geometry_msgs::msg::Twist();
            vel_pub_->publish(vel);
            feedback_.dock_feedback = robot_msg::action::AutoDock::Feedback::TIMEOUT;
            feedback_.dock_feedback_text = "auto docking timeout 5min!";

            auto feedback = std::make_shared<robot_msg::action::AutoDock::Feedback>();
            feedback->dock_feedback = feedback_.dock_feedback;
            feedback->dock_feedback_text = feedback_.dock_feedback_text;
            goal_handle->publish_feedback(feedback);
            
            auto result = std::make_shared<robot_msg::action::AutoDock::Result>();
            result->dock_result = false;
            goal_handle->succeed(result);
            RCLCPP_ERROR(this->get_logger(), "---auto docking failed, timeout");
            return;
        }

        rate.sleep();
    }
}

void AutoDockingAction::handleCameraAfterDocking(bool swt)
{
    using SetBool = std_srvs::srv::SetBool;

    if (docking_success_op_camera_up_)
    {
        auto request = std::make_shared<SetBool::Request>();
        request->data = swt;

        auto future = camera_control_client_up_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Upper camera service status: %d", swt);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to operate upper camera service.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call upper camera control service.");
        }
    }

    if (docking_success_op_camera_down_)
    {
        auto request = std::make_shared<SetBool::Request>();
        request->data = swt;

        auto future = camera_control_client_down_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Lower camera service status: %d", swt);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to operate lower camera service.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call lower camera control service.");
        }
    }
}


#if 0
void AutoDockingAction::executeCB(
    const robot_msg::auto_dockGoalConstPtr &goal) {
  geometry_msgs::Twist vel;
  bool current_goal = goal->dock_ctrl;
  bool touch_ = goal->touch;
  ros::Duration task_duration(300); // 任务超时时间

  ros::Time start_time_ = ros::Time::now();
  Init(current_goal, touch_);
  ros::Rate r(10);
  while (ros::ok()) {
    if (as_.isPreemptRequested()) {
      if (as_.isNewGoalAvailable()) {
        robot_msg::auto_dockGoal new_goal = *as_.acceptNewGoal();

        current_goal = new_goal.dock_ctrl;
        touch_ = new_goal.touch;
        Init(current_goal, touch_);
        ROS_INFO_THROTTLE(1, "auto docking, recive a new goal");
      } else {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        feedback_.dock_feedback = robot_msg::auto_dockFeedback::CANCLED;
        feedback_.dock_feedback_text = "auto docking cancled!";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // todo 设置底盘状态
        result_.dock_result = true;
        as_.setPreempted(result_, "AutoDocking Preempted");
        return;
      }
    }
    dockingmachine_->getState(feedback_.dock_feedback,
                             feedback_.dock_feedback_text);
    as_.publishFeedback(feedback_);
    dockingmachine_->ExeCycle(robot_pose_, vel); // 状态机执行
    SafetyControl(vel);
    // ROS_WARN_STREAM_THROTTLE(1, "pub feedback : " <<
    // feedback_.dock_feedback_text); //打印状态机
    if (feedback_.dock_feedback ==
        robot_msg::auto_dockFeedback::DOCKING_SUCCESS) // docking success
    {
      result_.dock_result = true;
      as_.setSucceeded(result_);
      ROS_WARN("---docking success");
      if(docking_success_op_camera_up_)
      {
        // 关闭上面的相机
        std_srvs::SetBool camera_control_msg_up;
        camera_control_msg_up.request.data = false;
        if (camera_control_client_up.call(camera_control_msg_up)) 
        {
            if (camera_control_msg_up.response.success) {
                ROS_INFO("Upper camera service closed.");
            } else {
                ROS_ERROR("Failed to close upper camera service.");
            }
        } else {
            ROS_ERROR("Failed to call upper camera control service.");
        }

      }
      if(docking_success_op_camera_down_)
      {
        std_srvs::SetBool camera_control_msg_down;
        camera_control_msg_down.request.data = false;
        // 关闭下面的相机
        if (camera_control_client_down.call(camera_control_msg_down)) 
        {
            if (camera_control_msg_down.response.success) {
                ROS_INFO("Lower camera service closed.");
            } else {
                ROS_ERROR("Failed to close lower camera service.");
            }
        } else {
            ROS_ERROR("Failed to call lower camera control service.");
        }
      }

      return;
    }
    if (feedback_.dock_feedback ==
        robot_msg::auto_dockFeedback::SEPERATING_DONE) // seperate success
    {
      result_.dock_result = true;
      as_.setSucceeded(result_);
      ROS_WARN("---separating success");
      if(docking_success_op_camera_up_)
      {
        // 开启上面的相机
        std_srvs::SetBool camera_control_msg_up;
        camera_control_msg_up.request.data = true;
        if (camera_control_client_up.call(camera_control_msg_up)) 
        {
            if (camera_control_msg_up.response.success) {
                ROS_INFO("Upper camera service closed.");
            } else {
                ROS_ERROR("Failed to close upper camera service.");
            }
        } else {
            ROS_ERROR("Failed to call upper camera control service.");
        }

      }
      if(docking_success_op_camera_down_)
      {
        std_srvs::SetBool camera_control_msg_down;
        camera_control_msg_down.request.data = true;
        // 开启下面的相机
        if (camera_control_client_down.call(camera_control_msg_down)) 
        {
            if (camera_control_msg_down.response.success) {
                ROS_INFO("Lower camera service closed.");
            } else {
                ROS_ERROR("Failed to close lower camera service.");
            }
        } else {
            ROS_ERROR("Failed to call lower camera control service.");
        }
      }
      return;
    }
    if (feedback_.dock_feedback ==robot_msg::auto_dockFeedback::DOCKING_FAILURE ||feedback_.dock_feedback ==robot_msg::auto_dockFeedback::ERROR_DOCKER_POS ||
        feedback_.dock_feedback ==robot_msg::auto_dockFeedback::SEARCHING_FAILURE ||feedback_.dock_feedback ==robot_msg::auto_dockFeedback::DOCKER_NO_POWER||
        feedback_.dock_feedback ==robot_msg::auto_dockFeedback::OBSTACLE_FAILURE ||feedback_.dock_feedback ==robot_msg::auto_dockFeedback::DOCKER_OUT_OF_DISTANCE||
        feedback_.dock_feedback ==robot_msg::auto_dockFeedback::SEPERAT_FAILURE) // fail
    {
      result_.dock_result = false;
      as_.setSucceeded(result_);
      ROS_ERROR("---auto docking failed");
      return;
    }
    if ((ros::Time::now() - start_time_).toSec() > task_duration.toSec()) {
      vel = geometry_msgs::Twist();
      vel_pub.publish(vel);
      feedback_.dock_feedback =robot_msg::auto_dockFeedback::TIMEOUT ; 
      feedback_.dock_feedback_text = "auto docking ,timeout 5min!";
      as_.publishFeedback(feedback_);
      result_.dock_result = false;
      as_.setSucceeded(result_);
      ROS_ERROR("---auto docking failed, timeout");
      return;
    }
    r.sleep();
  }
}
#endif

void AutoDockingAction::SafetyControl(geometry_msgs::msg::Twist &target_vel, const std::shared_ptr<GoalHandleAutoDock> goal_handle) {
  geometry_msgs::msg::Twist control_vel;
  // 简单的速度平滑控制
  if (target_vel.linear.x > robot_vel_.linear.x)
    control_vel.linear.x =
        std::min(target_vel.linear.x, robot_vel_.linear.x + 0.02);
  else if (target_vel.linear.x < robot_vel_.linear.x) {
    control_vel.linear.x =
        std::max(target_vel.linear.x, robot_vel_.linear.x - 0.02);
  } else {
    control_vel.linear.x = robot_vel_.linear.x;
  }

  if (target_vel.angular.z > robot_vel_.angular.z)
    control_vel.angular.z =
        std::min(target_vel.angular.z, robot_vel_.angular.z + 0.1);
  else if (target_vel.angular.z < robot_vel_.angular.z) {
    control_vel.angular.z =
        std::max(target_vel.angular.z, robot_vel_.angular.z - 0.1);
  } else {
    control_vel.angular.z = robot_vel_.angular.z;
  }

  uint8_t state_now;
  std::string stateStr_now;
  dockingmachine_->getState(state_now, stateStr_now);
  if (stateStr_now == "docking station 6"|| stateStr_now == "docking station 6 OBSTACLE") //最后的对接不考虑避障
  {
    dockingmachine_->obstacle_ = false;
  } 
  else if ((stateStr_now == "separating" && docking_forward_ == true)) //退桩脱离阶段考虑避障
  {
    if (dockingmachine_->sonar_obstacle_ == true ||
        dockingmachine_->line_laser_obstacle_ == true) 
    {
      // 设置 obstacle 标志
      dockingmachine_->obstacle_ = true;

      // 填充 feedback
      auto feedback = std::make_shared<robot_msg::action::AutoDock::Feedback>();
      feedback->dock_feedback = robot_msg::action::AutoDock::Feedback::OBSTACLE;
      feedback->dock_feedback_text = "separating obstacle";

      // 发布 feedback
      goal_handle->publish_feedback(feedback);

      // 节流打印警告
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          5000,  // 5秒
          "separating obstacle -> sonar: %d line_laser: %d",
          dockingmachine_->sonar_obstacle_,
          dockingmachine_->line_laser_obstacle_);
    } else {
      dockingmachine_->obstacle_ = false;
    }
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        10000, "dockingmachine_->obstacle_: %d", dockingmachine_->obstacle_);
  } 
  else if (bump_trigger_) //碰撞触发
  {
    dockingmachine_->obstacle_ = true;
    //feedback_.dock_feedback = robot_msg::auto_dockFeedback::OBSTACLE;
    //feedback_.dock_feedback_text = "bump_trigger_ obstacle";
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),5000, "auto docking, bump trigger");
  } 
  else
    dockingmachine_->obstacle_ = obs_detector_.haveObstacle(target_vel);

  if (dockingmachine_->obstacle_) {
    control_vel.linear.x = 0.0;
    control_vel.angular.z = 0.0;
  }
  robot_vel_ = control_vel;
  vel_pub_->publish(control_vel);
}

void AutoDockingAction::dockerCallBack(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  geometry_msgs::msg::PoseStamped pose = *msg;
  double_t distance = std::hypot(pose.pose.position.x, pose.pose.position.y);
  if (distance < 0.4)
    safety_distance_ = 0.15;
  else if (distance < 0.5 && distance > 0.4) {
    safety_distance_ = 0.25;
  } else {
    safety_distance_ = 0.3;
  }
}

void AutoDockingAction::collisionBarCB(const std_msgs::msg::UInt8::SharedPtr msg)
{
    unsigned char data = msg->data; 
    if(data == 0) 
    {
        //ROS_INFO("recv collision bar no collision!");
        real_bump_trigger_ = 0;
    }
    else if(data == 1)  //1: collision   2: collision keep
    {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),10000, "recv collision bar collision!");
        real_bump_trigger_ = 1;
    }
    else if(data == 2)
    {
        RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),10000, "recv collision bar collision keep!");
        real_bump_trigger_ = 1;
    }
    else
    {
        real_bump_trigger_ = 0;
    }
    bump_trigger_ =(real_bump_trigger_||virtual_bump_trigger_);
}

void AutoDockingAction::RobotState_CallBack(
    const robot_msg::msg::BaseInfo::SharedPtr msg) {
  if (msg->motor_left_err_code == 0 || msg->motor_right_err_code == 0)
    dockingmachine_->motor_error_ = false;
  else {
    if ((msg->base_device_state & 0x10) != 0) // 急停
      dockingmachine_->motor_error_ = true;
  }
  if (enable_virtual_bumper_)
  {
    virtual_bump_trigger_ = static_cast<bool>(msg->virtual_bumper_state);
    //ROS_WARN_STREAM_THROTTLE(10, "enable_virtual_bumper_=:"<< enable_virtual_bumper_ <<", virtual_bump_trigger_:" <<virtual_bump_trigger_ << std::endl);

  }
  else{
    virtual_bump_trigger_ = false;
  }
  if (!enable_real_bumper_)
  {
       real_bump_trigger_ = false;
  }
  
  //ROS_WARN_STREAM_THROTTLE(10, "real_bump_trigger_:"<< real_bump_trigger_ <<", virtual_bump_trigger_:" <<virtual_bump_trigger_ << std::endl);
  bump_trigger_ =(real_bump_trigger_||virtual_bump_trigger_);
  if(bump_trigger_ ==true)
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),10000, 
        "real_bump_trigger_:%d , virtual_bump_trigger_:%d",
         real_bump_trigger_, virtual_bump_trigger_);
  }

  if (use_bms_state_)
    return;
  baseChargingState_ = msg->charging_state;
  if (baseChargingState_ != 0)
    dockingmachine_->chargingState_ = true;
  else
    dockingmachine_->chargingState_ = false;
}

void AutoDockingAction::BatteryCallBack(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  sensor_msgs::msg::BatteryState batteryState;

  batteryState = *msg;
  if (use_bms_state_) {
    if (batteryState.power_supply_status ==
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING ||
        batteryState.power_supply_status ==
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL ||
        batteryState.power_supply_status ==
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN) {
      baseChargingState_ = robot_msg::msg::BaseInfo::CHARGING;
      dockingmachine_->chargingState_ = true;
    }
    if (batteryState.power_supply_status ==
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING ||
        batteryState.power_supply_status ==
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING) {
      baseChargingState_ = robot_msg::msg::BaseInfo::UNCHARGING;
      dockingmachine_->chargingState_ = false;
    }
  }
}

void AutoDockingAction::SonarCallBacks(const sensor_msgs::msg::Range::SharedPtr msg) {
  sensor_msgs::msg::Range range_sensor;
  static bool trigger_front_left = false, trigger_front_right = false,
              trigger_back_left = false, trigger_back_right = false;

  range_sensor = *msg;
  std::string sensor_type = range_sensor.header.frame_id;
  double_t sensor_data = range_sensor.range;
  static int trigger_count_left = 0;
  static int trigger_count_right = 0;

  if (!enable_sonar_) {
    dockingmachine_->sonar_obstacle_ = false;
    return;
  }
  if (sensor_type == "ultrasonic_back_left") {
    if (sensor_data < seperation_dis_ && sensor_data > 0.025) {
      trigger_count_left++;
      if (trigger_count_left > 5) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, "ultrasonic_back_left obstacle : %.3f", sensor_data);
        trigger_back_left = true;
        trigger_count_left = 0;
      }
    } else {
      if(sensor_data >= seperation_dis_) {
        if (trigger_count_left > 0) {
          trigger_count_left--;
        }
      }
      trigger_back_left = false;
    }
  } 
  else if (sensor_type == "ultrasonic_back_right") {
    if (sensor_data < seperation_dis_ && sensor_data > 0.025) {
      trigger_count_right++;
      if (trigger_count_right > 5) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, "ultrasonic_back_right obstacle : %.3f",sensor_data);
        trigger_back_right = true;
        trigger_count_right = 0;
      }
    } else {
      if(sensor_data >= seperation_dis_) {
        if (trigger_count_right > 0) {
          trigger_count_right--;
        }
      }
      trigger_back_right = false;
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),1000, "error unknow sensor type: %s", sensor_type.c_str());
  }

  // ROS_INFO_STREAM_THROTTLE(1, "trigger_back_left:" << trigger_back_left <<
  // "trigger_back_right" << trigger_back_right);
  if (trigger_back_left == true || trigger_back_right == true)
    dockingmachine_->sonar_obstacle_ = true;
  else {
    dockingmachine_->sonar_obstacle_ = false;
  }
}

void AutoDockingAction::LineLaserCallBacks(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  sensor_msgs::msg::LaserScan line_laser_sensor;
  static bool trigger_back = false;

  line_laser_sensor = *msg;
  std::string sensor_type = line_laser_sensor.header.frame_id;
  size_t size = line_laser_sensor.ranges.size();
  std::vector<double_t> sensor_datas;
  const float avoidance_dic = 0.25;
  for (size_t i = 0; i < size; i++) {
    if (line_laser_sensor.intensities[i] > 0 &&
        line_laser_sensor.ranges[i] > line_laser_sensor.range_min &&
        line_laser_sensor.ranges[i] <= line_laser_sensor.range_max) {
      sensor_datas.emplace_back(line_laser_sensor.ranges[i]);
    }
  }
  double sumValue =
      std::accumulate(begin(sensor_datas), end(sensor_datas), 0.0);
  double meanValue = sumValue / sensor_datas.size();
  static int trigger_count_back = 0;
  if (enable_line_laser_ == false) {
    dockingmachine_->line_laser_obstacle_ = false;
    return;
  }

  if (sensor_type == "laser_middle_back") {
    if (meanValue < avoidance_dic) {
      RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(), 5000, "line laser meanValue : %.3f", meanValue);
      trigger_back = true;
      trigger_count_back = 0;
    } else {
      trigger_back = false;
    }

  } else {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "error unknow sensor type: %s", sensor_type.c_str());
  }

  // ROS_WARN_STREAM_THROTTLE(1, "trigger_back:" << trigger_back );
  if (trigger_back == true)
    dockingmachine_->line_laser_obstacle_ = true;
  else {
    dockingmachine_->line_laser_obstacle_ = false;
  }
}

void AutoDockingAction::Odom_CallBack(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_.x = msg->pose.pose.position.x;
    robot_pose_.y = msg->pose.pose.position.y;

    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);

    if (use_imu_angle_) {
        return;
    }

    robot_pose_.theta = yaw;
}


void AutoDockingAction::Imu_CallBack(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    if (use_imu_angle_) {
        robot_pose_.theta = yaw;
    }
}

bool AutoDockingAction::configServer(
    const std::shared_ptr<robot_msg::srv::SetDevice::Request> request,
    std::shared_ptr<robot_msg::srv::SetDevice::Response> response)
{
    if (request->device_id == "auto_docking") {
        if (request->cmd == "docking_forward") {
            if (request->parameter1 == 0) {
                docking_forward_ = true;
                RCLCPP_WARN(this->get_logger(), "set param docking_forward: true");
            } else {
                docking_forward_ = false;
                RCLCPP_WARN(this->get_logger(), "set param docking_forward: false");
            }
        }
    }

    // ROS2 参数设置
    this->set_parameter(rclcpp::Parameter("docking_forward", docking_forward_));

    // 填充响应
    response->result = true;
    response->message_code = 0;
    response->message = "success";
    return true;
}

void AutoDockingAction::LoadParam()
{
    // 声明并获取参数
    this->declare_parameter<double>("robot_radius", 0.23);
    double robot_radius = this->get_parameter("robot_radius").as_double();
    obs_detector_.setRobotRadius(robot_radius);

    this->declare_parameter<double>("seperation_dis", 0.1);
    seperation_dis_ = this->get_parameter("seperation_dis").as_double();

    this->declare_parameter<double>("docking_safety_distance", 0.1);
    safety_distance_ = this->get_parameter("docking_safety_distance").as_double();

    this->declare_parameter<bool>("docking_use_imu_angle", false);
    use_imu_angle_ = this->get_parameter("docking_use_imu_angle").as_bool();

    this->declare_parameter<bool>("docking_use_bms_state", false);
    use_bms_state_ = this->get_parameter("docking_use_bms_state").as_bool();

    this->declare_parameter<bool>("docking_enable_sonar", false);
    enable_sonar_ = this->get_parameter("docking_enable_sonar").as_bool();

    this->declare_parameter<bool>("docking_forward", false);
    docking_forward_ = this->get_parameter("docking_forward").as_bool();

    this->declare_parameter<double>("max_docking_distance", 0.25);
    max_docking_distance_ = this->get_parameter("max_docking_distance").as_double();

    this->declare_parameter<double>("forward_safety_dis_", 0.15);
    forward_safety_dis_ = this->get_parameter("forward_safety_dis_").as_double();

    this->declare_parameter<bool>("docking_enable_line_laser", false);
    enable_line_laser_ = this->get_parameter("docking_enable_line_laser").as_bool();

    this->declare_parameter<bool>("docking_enable_virtual_bumper", false);
    enable_virtual_bumper_ = this->get_parameter("docking_enable_virtual_bumper").as_bool();

    this->declare_parameter<bool>("docking_enable_real_bumper", false);
    enable_real_bumper_ = this->get_parameter("docking_enable_real_bumper").as_bool();

    this->declare_parameter<double>("docking_point_offset", 0.8);
    docking_point_offset_ = this->get_parameter("docking_point_offset").as_double();

    this->declare_parameter<double>("distance_movebase_arrived_tolerance", 0.8);
    distance_movebase_arrived_tolerance_ = this->get_parameter("distance_movebase_arrived_tolerance").as_double();

    this->declare_parameter<double>("scan_docker_distance_tolerance", 1.5);
    scan_docker_distance_tolerance_ = this->get_parameter("scan_docker_distance_tolerance").as_double();

    this->declare_parameter<bool>("docking_success_op_camera_up", false);
    docking_success_op_camera_up_ = this->get_parameter("docking_success_op_camera_up").as_bool();

    this->declare_parameter<bool>("docking_success_op_camera_down", false);
    docking_success_op_camera_down_ = this->get_parameter("docking_success_op_camera_down").as_bool();

    if (bump_trigger_) {
        RCLCPP_INFO(this->get_logger(), "bump trigger");
    }
}
