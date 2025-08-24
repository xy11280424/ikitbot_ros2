#include "auto_docking/dockingmachine.hpp"


//void DOCKINGMACHINE::chargeState(State *pState)
void DOCKINGMACHINE::chargeState(std::unique_ptr<State> pState)
{
    state_ = std::move(pState);  //旧的 state_ 会自动释放，不需要手动 delete
    resetOscillationTime();
}

void InitState::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case NONE:
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            break;
        case RECV_CHG_CMD:
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            machine->retry_num_ = 0;
            RCLCPP_INFO(rclcpp::get_logger("InitState"), "init state, changing state to GOTO_FIRST_POINT_STATE");
            machine->chargeState(std::make_unique<GOTO_FIRST_POINT_STATE>());
            break;
        default:
            break;
    }
}


void SCAN_DOCKER_POSE::Handle(
    DOCKINGMACHINE *machine,
    const geometry_msgs::msg::Pose2D &robot_pose,
    geometry_msgs::msg::Twist &cmd_vel)
{
    std::vector<double_t> yaw_arrs;
    rotated += machine->poseUpdate_.theta;
    static int error_counter = 0;
    bool error_recognized = false;

    try {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        geometry_msgs::msg::TransformStamped t;
        t = machine->tf_buffer_->lookupTransform(
            "odom",
            "docker",
            rclcpp::Time(0)  // 最新 transform
        );
        transformStamped_[counter++] = t;

        machine->resetOscillationTime();

        if (counter >= 20) {
            rclcpp::Time t19(transformStamped_[19].header.stamp);
            rclcpp::Time t10(transformStamped_[10].header.stamp);
            //if ((transformStamped_[19].header.stamp - transformStamped_[10].header.stamp) < rclcpp::Duration::from_seconds(5.0)) 
            if((t19 - t10) < rclcpp::Duration::from_seconds(5.0)) {
                RCLCPP_INFO_THROTTLE(
                    machine->node_->get_logger(),
                    *machine->node_->get_clock(),
                    1.0,
                    "docker trans frame_id %s",
                    transformStamped_[10].header.frame_id.c_str()
                );

                counter = 0;
                machine->got_dock_pose_ = true;
                machine->dock_pose_.x = 0.0;
                machine->dock_pose_.y = 0.0;
                machine->dock_pose_.theta = 0.0;

                for (int i = 10; i < 19; i++) {
                    tf2::Quaternion q(
                        transformStamped_[i].transform.rotation.x,
                        transformStamped_[i].transform.rotation.y,
                        transformStamped_[i].transform.rotation.z,
                        transformStamped_[i].transform.rotation.w
                    );
                    tf2::Matrix3x3 m(q);
                    double_t roll, pitch, yaw;
                    m.getEulerYPR(yaw, pitch, roll);
                    yaw_arrs.push_back(yaw);

                    machine->dock_pose_.x += transformStamped_[i].transform.translation.x / 9;
                    machine->dock_pose_.y += transformStamped_[i].transform.translation.y / 9;
                }

                std::sort(yaw_arrs.begin(), yaw_arrs.end());
                machine->dock_pose_.theta = yaw_arrs[4];

                RCLCPP_INFO(machine->node_->get_logger(),
                    "---docker pose update result : pose:(%.3f, %.3f, %.3f), robot_pose:(%.3f, %.3f, %.3f)",
                    machine->dock_pose_.x, machine->dock_pose_.y, angles::to_degrees(machine->dock_pose_.theta),
                    robot_pose.x, robot_pose.y, robot_pose.theta);

                // -------- ROS2 参数获取 --------
                double distance_tolerance = 1.5;
                if (!machine->node_->has_parameter("auto_docking.scan_docker_distance_tolerance")) {
                    machine->node_->declare_parameter("auto_docking.scan_docker_distance_tolerance", 1.5);
                    RCLCPP_WARN(machine->node_->get_logger(),
                        "scan_docker_distance_tolerance not set, using default value 1.5");
                }
                machine->node_->get_parameter("auto_docking.scan_docker_distance_tolerance", distance_tolerance);

                // -------- 判断是否超出距离 --------
                if (std::abs(robot_pose.x - machine->dock_pose_.x) > distance_tolerance ||
                    std::abs(robot_pose.y - machine->dock_pose_.y) > distance_tolerance)
                {
                    RCLCPP_ERROR_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 5000,
                        "---docker pose out of limits, tolerance=%.2f, dx=%.2f, dy=%.2f",
                        distance_tolerance,
                        std::abs(robot_pose.x - machine->dock_pose_.x),
                        std::abs(robot_pose.y - machine->dock_pose_.y));

                    if (error_counter++ > 5) {
                        machine->got_dock_pose_ = false;
                        error_recognized = true;
                        machine->setEvent(ERROR_DOCK_POSE);
                        machine->chargeState(std::make_unique<FAIL_STATE>());
                        RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000,
                            "REACHED_FIRST_POINT, docker pose out of limits, NO DOCKER FOUND.");
                        error_counter = 0;
                    } else {
                        return;
                    }
                } else {
                    machine->got_dock_pose_ = true;
                    error_recognized = false;
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 5000,
                        "---docker pose in limits, tolerance=%.2f, dx=%.2f, dy=%.2f",
                        distance_tolerance,
                        std::abs(robot_pose.x - machine->dock_pose_.x),
                        std::abs(robot_pose.y - machine->dock_pose_.y));
                }

                switch (machine->getEvent()) {
                    case REACHED_FIRST_POINT:
                        machine->chargeState(std::make_unique<GOTO_FIRST_POINT_STATE>());
                        break;
                    case REACHED_SECOND_POINT:
                        machine->chargeState(std::make_unique<GOTO_SECOND_POINT_STATE>());
                        break;
                    case REACHED_THIRD_POINT:
                        machine->chargeState(std::make_unique<GOTO_THIRD_POINT_STATE>());
                        break;
                    default:
                        break;
                }
            } else {
                counter = 0;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.3;
                //rclcpp::sleep_for(std::chrono::duration<double>(0.5));
                rclcpp::sleep_for(std::chrono::milliseconds(500)); //500ms
            }
        }
    }
    catch (tf2::TransformException &ex) {
        counter = 0;
        // 这里保留原有的状态机逻辑，只是 log 用 ROS2 宏替换
        switch (machine->getEvent()) {
            case REACHED_FIRST_POINT:
                if (machine->valid_sport_check(30)) {
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000,
                        "REACHED_FIRST_POINT, got docker pose failed, waiting...");
                } else {
                    RCLCPP_WARN(machine->node_->get_logger(),
                        "REACHED_FIRST_POINT, timeout 30s, NO DOCKER FOUND.");
                    machine->got_dock_pose_ = false;
                    machine->setEvent(OBSTACLE);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                }
                if (!machine->obstacle_) {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.5;
                    RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000,
                        "REACHED_FIRST_POINT SCANNING DOCKER...");
                } else {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0.0;
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 5000,
                        "REACHED_FIRST_POINT, obstacle... waiting...");
                }
                if (rotated > 2 * 2 * M_PI) {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    machine->got_dock_pose_ = false;
                    machine->setEvent(DOCK_POSE_UNAVAILABLE);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                    RCLCPP_WARN(machine->node_->get_logger(),
                        "REACHED_FIRST_POINT rotated > 2*M_PI NO DOCKER FOUND.");
                }
                break;

            case REACHED_SECOND_POINT:
                if (machine->valid_sport_check(30)) {
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 5000,
                        "REACHED_SECOND_POINT, got docker pose failed, waiting...");
                } else {
                    machine->got_dock_pose_ = false;
                    machine->setEvent(DOCK_POSE_UNAVAILABLE);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                    RCLCPP_WARN(machine->node_->get_logger(),
                        "REACHED_SECOND_POINT timeout 30s, NO DOCKER FOUND.");
                }
                break;

            case REACHED_THIRD_POINT:
                if (machine->valid_sport_check(30)) {
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 5000,
                        "REACHED_THIRD_POINT, got docker pose failed, waiting...");
                } else {
                    machine->got_dock_pose_ = false;
                    machine->setEvent(DOCK_POSE_UNAVAILABLE);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                    RCLCPP_WARN(machine->node_->get_logger(),
                        "REACHED_THIRD_POINT timeout 30s, NO DOCKER FOUND.");
                }
                break;

            default:
                break;
        }
    }
}

void RECOGNIZE_DOCKER::Handle(
    DOCKINGMACHINE *machine,
    const geometry_msgs::msg::Pose2D &robot_pose,
    geometry_msgs::msg::Twist &cmd_vel)
{  
    rotated += machine->poseUpdate_.theta;
    if (rotated_2pi) {
        if (recognized_docker) {
            yaw_err = angles::shortest_angular_distance(robot_pose.theta, min_dist_docker_pose.theta);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = yaw_err * 2.0;
            cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -0.7, 0.7);

            if (std::fabs(angles::to_degrees(yaw_err)) < 20.0) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>());
                RCLCPP_INFO(machine->node_->get_logger(), "RECOGNIZE_DOCKER_STATE, change state to SCAN_DOCKER_POSE");
            }
        } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

            if (machine->getRetryPose()) {
                RCLCPP_WARN(machine->node_->get_logger(), "NO DOCKER RECOGNIZED, REACHED_FIRST_POINT --> GOTO_RETRY_POINT_STATE");
                RCLCPP_INFO(machine->node_->get_logger(), "target pose(%.3f, %.3f)", 
                machine->target_pose_.x, machine->target_pose_.y);
                machine->chargeState(std::make_unique<GOTO_RETRY_POINT_STATE>());
            } else {
                machine->setEvent(DOCK_POSE_UNAVAILABLE);
                machine->chargeState(std::make_unique<FAIL_STATE>());
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000, "NO DOCKER RECOGNIZE.");
            }
        }
    } else {
        if (rotated > M_PI || !machine->valid_sport_check(30)) { // 30s内旋转180度
            rotated_2pi = true;
        }

        try {
            // 获取 docker 相对于 laser 坐标系的 transform
            transformStamped_ = machine->tf_buffer_->lookupTransform(
                "laser", "docker", rclcpp::Time(0)
            );

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.3;
            recognized_docker = true;

            double temp = hypot(transformStamped_.transform.translation.x,
                                transformStamped_.transform.translation.y);

            // 更新距离最近的 docker 坐标
            if (docker_dist > temp) {
                docker_dist = temp;

                // 获取 docker 相对于 odom 的 transform
                transformStamped_ = machine->tf_buffer_->lookupTransform(
                    "odom", "docker", rclcpp::Time(0)
                );

                min_dist_docker_pose.x = transformStamped_.transform.translation.x;
                min_dist_docker_pose.y = transformStamped_.transform.translation.y;
                min_dist_docker_pose.theta = std::atan2(
                    min_dist_docker_pose.y - robot_pose.y,
                    min_dist_docker_pose.x - robot_pose.x
                );

                RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000,
                    "min distance docker yaw: %.2f", angles::to_degrees(min_dist_docker_pose.theta));
            }
        }
        catch (const tf2::TransformException &ex) {
            if (machine->valid_sport_check(20)) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.7;
                RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 3000, "RECOGNIZE DOCKER...");
            } else {
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000,
                    "timeout 20s, NO DOCKER RECOGNIZED.");
                machine->setEvent(DOCK_POSE_UNAVAILABLE);
                machine->chargeState(std::make_unique<FAIL_STATE>());
            }
        }
    }
}



void GOTO_FIRST_POINT_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case RECV_CHG_CMD:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            updateRetryList(machine, robot_pose);
            machine->setEvent(REACHED_FIRST_POINT);
            machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>());
            break;
        case REACHED_FIRST_POINT:
            RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000,
                    "-----REACHED_FIRST_POINT pos: (%.3f,%.3f,%.3f)------", robot_pose.x, robot_pose.y,robot_pose.theta);
            if(getSecondPose(machine, machine->tf_buffer_, machine->target_pose_))
            {
                machine->setEvent(DOCK_POSE_AVAILABLE);
                machine->target_pose_.theta = angles::normalize_angle(M_PI + machine->dock_pose_.theta);    //计算对接点坐标，激光正对充电桩
                RCLCPP_INFO(machine->node_->get_logger(), "second theta %.3f", angles::to_degrees(machine->target_pose_.theta));

                machine->chargeState(std::make_unique<GOTO_SECOND_POINT_STATE>());
                RCLCPP_INFO(machine->node_->get_logger(), "GOTO_FIRST_POINT_STATE, change state to GOTO_SECOND_POINT_STATE");
            }
            else
            {
                machine->setEvent(DOCK_POSE_UNAVAILABLE);
                machine->chargeState(std::make_unique<FAIL_STATE>());
            }
            break;
        default:
            break;
    }
}

void GOTO_SECOND_POINT_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{  
    switch (machine->getEvent())
    {
        case DOCK_POSE_AVAILABLE:
        {
            machine->resetOscillationTime();
            if(reached_xy)  //先运动至目标位置，再调整角度
            {
                yaw_err = angles::shortest_angular_distance(robot_pose.theta, machine->target_pose_.theta);
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = yaw_err * 2.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.3), 0.3);
                if(std::fabs(angles::to_degrees(yaw_err)) < 10)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    updateRetryList(machine, robot_pose);
                    machine->setEvent(REACHED_SECOND_POINT);
                    machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>()); // 到达，再次确认充电桩坐标
                    RCLCPP_INFO(machine->node_->get_logger(), "GOTO_SECOND_POINT_STATE, REACHED_SECOND_POINT, change state to SCAN_DOCKER_POSE");
                }
            }
            else
            {
                dis_err = std::hypot(robot_pose.x - machine->target_pose_.x, robot_pose.y - machine->target_pose_.y);
                heading = std::atan2(machine->target_pose_.y - robot_pose.y, machine->target_pose_.x - robot_pose.x );
                angle_err = angles::shortest_angular_distance(robot_pose.theta, heading);

//                ROS_INFO_THROTTLE(1, "heading: %.3f", heading);
//                ROS_INFO_THROTTLE(1, "angle_err: %.3f", angle_err);
                cmd_vel.linear.x = dis_err * 1.0;
                if(angle_err < 0.2 && angle_err > -0.2)
                    cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, 0.05), 0.1);
                else
                    cmd_vel.linear.x = 0.0;

                cmd_vel.angular.z = angle_err * 3.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.5), 0.5);
                if(dis_err < 0.01)
                {
                    reached_xy = true;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                }
                if(machine->obstacle_ && cmd_vel.linear.x > 0.001){
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    machine->setEvent(OBSTACLE);
                    return;
                }
            }
            break;
        }
        case REACHED_SECOND_POINT:
        {
            RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000, "-----REACHED_SECOND_POINT pos: (%.3f,%.3f,%.3f)", robot_pose.x, robot_pose.y,robot_pose.theta);
            if(getThirdPose(machine, machine->tf_buffer_, machine->target_pose_))
            {
                machine->setEvent(DOCK_POSE_AVAILABLE);
                if(machine->docking_forward_)
                    machine->target_pose_.theta = angles::normalize_angle(machine->dock_pose_.theta + M_PI);
                else
                    machine->target_pose_.theta = angles::normalize_angle(machine->dock_pose_.theta + angles::from_degrees(150));
                RCLCPP_WARN(machine->node_->get_logger(), "third pose theta %.3f", angles::to_degrees(machine->target_pose_.theta));
                machine->chargeState(std::make_unique<GOTO_THIRD_POINT_STATE>());
                RCLCPP_INFO(machine->node_->get_logger(), "GOTO_SECOND_POINT_STATE, REACHED_SECOND_POINT, change state to GOTO_THIRD_POINT_STATE");
            }
            else
            {
                machine->setEvent(DOCK_POSE_UNAVAILABLE);
                machine->chargeState(std::make_unique<FAIL_STATE>());
            }
                
            break;
        }
        case OBSTACLE:
        {
            // cmd_vel.linear.x = 0.0;
            // cmd_vel.angular.z = 0.0;
            if(machine->valid_sport_check(30))
            {
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000,"GOTO_SECOND_POINT_STATE, obstacle...");
            }
            else 
            {
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000, "GOTO_SECOND_POINT_STATE, obstacle,timeout 30s...");
                machine->setEvent(OBSTACLE);
                machine->chargeState(std::make_unique<FAIL_STATE>());
            }
            if(machine->obstacle_ == false)
            {
                machine->setEvent(DOCK_POSE_AVAILABLE);
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000, "GOTO_SECOND_POINT_STATE, machine->obstacle_ == false");   
            }
            else 
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0.0;
                // machine->setEvent(OBSTACLE);
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000, "GOTO_SECOND_POINT_STATE, machine->obstacle_ == true");
            }

            break;
        }
        default:
            break;
    }
}



void GOTO_THIRD_POINT_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case REACHED_RETRY_POINT:
        case DOCK_POSE_AVAILABLE:
        {
            machine->resetOscillationTime();
            if(reached_xy)
            {
                yaw_err = angles::shortest_angular_distance(robot_pose.theta, machine->target_pose_.theta);
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = yaw_err * 2.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.5), 0.5);
                if(std::fabs(angles::to_degrees(yaw_err)) < 3)
                {
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    machine->setEvent(REACHED_THIRD_POINT);
                    machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>());   //距离太近，个别型号激光无法正确反馈充电桩轮廓，不再做检测
                }
            }
            else
            {
                dis_err = std::hypot(robot_pose.x - machine->target_pose_.x, robot_pose.y - machine->target_pose_.y);
                heading = std::atan2(machine->target_pose_.y - robot_pose.y, machine->target_pose_.x - robot_pose.x );
                angle_err = angles::shortest_angular_distance(robot_pose.theta, heading);

//                ROS_INFO_THROTTLE(1, "go to third , heading: %.3f", angles::to_degrees(heading));
//                ROS_INFO_THROTTLE(1, "go to third , angle_err: %.3f", angles::to_degrees(angle_err));
                cmd_vel.linear.x = dis_err * 0.1;
                if(angle_err < 0.1 && angle_err > -0.1) // 大约5°
                    cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, 0.05), 0.1);
                else
                    cmd_vel.linear.x = 0.0;

                cmd_vel.angular.z = angle_err * 3.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.5), 0.5);

                if(dis_err < 0.01)
                {
                    reached_xy = true;
                }
                if(machine->obstacle_ && cmd_vel.linear.x > 0.001){
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    machine->setEvent(OBSTACLE);
                    return;
                }
            }
            break;
        }
        case REACHED_THIRD_POINT:
        {
        RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "-----REACHED_THIRD_POINT pos: (%.3f,%.3f,%.3f)", robot_pose.x, robot_pose.y,robot_pose.theta);
	    if(machine->docking_forward_){
                updateRetryList(machine, machine->dock_pose_);
                machine->target_pose_.theta = angles::normalize_angle(machine->dock_pose_.theta + M_PI);
                heading = std::atan2(machine->dock_pose_.y - robot_pose.y, machine->dock_pose_.x - robot_pose.x );
                yaw_err = angles::shortest_angular_distance(robot_pose.theta, heading);
            }else
            {
                updateRetryList(machine, machine->dock_pose_);
                machine->target_pose_.theta = machine->dock_pose_.theta;
                yaw_err = angles::shortest_angular_distance(robot_pose.theta, machine->target_pose_.theta);
            }
            RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "go to third , yaw_err: %.3f", angles::to_degrees(yaw_err));
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = yaw_err;
            if(cmd_vel.angular.z <= 0.01)
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -0.7), -0.1);
            else {
                cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, 0.7), 0.1);
            }
            if(std::fabs(angles::to_degrees(yaw_err)) < 1)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "go to third , yaw_err: %.3f", angles::to_degrees(yaw_err));
                if(machine->touch_ctrl_){
                    machine->setEvent(REACHED_THIRD_POINT);
                    if(machine->docking_forward_){
                        machine->chargeState(std::make_unique<SLOW_FORWARD_STATE>());
                        RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000, "GOTO_THIRD_POINT_STATE, REACHED_THIRD_POINT, change state to SLOW_FORWARD_STATE");
                    }else {
                        machine->chargeState(std::make_unique<SLOW_BACK_STATE>());
                        RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000, "GOTO_THIRD_POINT_STATE, REACHED_THIRD_POINT, change state to SLOW_BACK_STATE");
                    }
                }else {
                    machine->setEvent(DOCKING_DONE);
                    machine->chargeState(std::make_unique<CHARGING_DONE>());
                    RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000, "GOTO_THIRD_POINT_STATE, REACHED_THIRD_POINT, notouch change state to CHARGING_DONE");
                }
            }
            break;
        }
        case OBSTACLE:
        {

            if(machine->valid_sport_check(30))
            {
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000,"GOTO_THIRD_POINT_STATE, obstacle...");
            }
            else 
            {
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000,"GOTO_THIRD_POINT_STATE, obstacle,timeout 30s...");
                machine->setEvent(OBSTACLE);
                machine->chargeState(std::make_unique<FAIL_STATE>());
            }
            if(machine->obstacle_ == false)
            {
                machine->setEvent(DOCK_POSE_AVAILABLE);
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000,"GOTO_THIRD_POINT_STATE, machine->obstacle_ == false");   
            }
            else 
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0.0;
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 10000,"GOTO_THIRD_POINT_STATE, machine->obstacle_ == true");
            }
            break;

        }
        default:
            break;
    }
}

void SLOW_BACK_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    distance += machine->poseUpdate_.x;
    RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock(), 1000, "slow back docking, distance: %.3f", distance);
    switch (machine->getEvent())
    {
        case REACHED_THIRD_POINT:
            cmd_vel.linear.x = std::min(std::max((distance-0.2), -0.05), -0.02);    
            cmd_vel.linear.x = (distance > 0.17)?-0.02:cmd_vel.linear.x;    // 分段设置电机对接速度，近距离时保持小速度
            cmd_vel.angular.z = 0.0;
            if(machine->motor_error_)    //  如果电机有报错，则清除报警重新使能
                machine->motor_ctrl(true);
            if(machine->chargingState_) 
            {
                cmd_vel.linear.x = -0.01;
                cmd_vel.angular.z = 0;
                machine->resetOscillationTime();
                machine->setEvent(DOCK_IN);
                machine->chargeState(std::make_unique<CHARGING_STATE>());
                break;
            }
            if(distance >= machine->max_back_dis_)   // 最大后退距离限制
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                if(machine->getRetryPose()){
                    RCLCPP_WARN(machine->node_->get_logger(), "SLOW_BACK_STATE, DISTANCE_OUT");
                    RCLCPP_INFO(machine->node_->get_logger(), "target pose(%.3f, %.3f)", 
                    machine->target_pose_.x, machine->target_pose_.y);
                    machine->setEvent(DISTANCE_OUT);
                    machine->chargeState(std::make_unique<GOTO_RETRY_POINT_STATE>());
                }else{
                    machine->setEvent(DISTANCE_OUT);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                }
            }
            break;
        default:
            break;
    }
}

void SLOW_FORWARD_STATE::Handle(DOCKINGMACHINE *machine,
                                const geometry_msgs::msg::Pose2D &robot_pose,
                                geometry_msgs::msg::Twist &cmd_vel)
{
    distance += machine->poseUpdate_.x;
    RCLCPP_INFO(machine->node_->get_logger() , "slow docking, distance: %.3f", distance);
    switch (machine->getEvent())
    {
        case REACHED_THIRD_POINT:
            cmd_vel.linear.x = (distance > 0.17)?0.02:0.05;
            cmd_vel.angular.z = 0.0;
            if(machine->motor_error_)
                machine->motor_ctrl(true);
            if(machine->chargingState_)
            {
                cmd_vel.linear.x = 0.01;
                cmd_vel.angular.z = 0;
                machine->resetOscillationTime();
                machine->setEvent(DOCK_IN);
                machine->chargeState(std::make_unique<CHARGING_STATE>());
                break;
            }

            if(distance >= machine->forward_safety_dis_)
            {
                RCLCPP_INFO(machine->node_->get_logger(), "slow docking, distance: %.3f >= %.3f", distance, machine->forward_safety_dis_);
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                // if(machine->getRetryPose()){
                //     ROS_WARN_STREAM("SLOW_FORWARD_STATE, DISTANCE_OUT");
                //     ROS_INFO_STREAM("target pose " << machine->target_pose_);
                //     machine->setEvent(DISTANCE_OUT);
                //     machine->chargeState(std::make_unique<SEPARATE_STATE>);
                // }else{
                if(!machine->chargingState_){
                    machine->setEvent(DOCKER_NO_POWER);
                }
                else{
                    machine->setEvent(DISTANCE_OUT);
                }
                machine->chargeState(std::make_unique<FAIL_STATE>());
                // }
            }
            break;
        default:
            break;
    }
}

void GOTO_RETRY_POINT_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case REACHED_FIRST_POINT:
        case REACHED_SECOND_POINT:
        {
            if(machine->obstacle_){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                if(machine->valid_sport_check(10)){
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "GOTO_RETRY_POINT_STATE, REACHED_SECOND_POINT,obstacle...");
                    machine->setEvent(OBSTACLE);
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000,"GOTO_RETRY_POINT_STATE, REACHED_SECOND_POINT,obstacle...machine->setEvent(OBSTACLE)");
                }
                else {
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000,"GOTO_RETRY_POINT_STATE,REACHED_SECOND_POINT,obstacle,timeout 10s...");
                    //machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000,"GOTO_RETRY_POINT_STATE,REACHED_SECOND_POINT,obstacle,timeout 10s...machine->chargeState(std::make_unique<FAIL_STATE)");
                    
                }
                return;
            }
            machine->resetOscillationTime(); //todo timeout
            if(reached_xy)
            {
                yaw_err = angles::shortest_angular_distance(robot_pose.theta, machine->target_pose_.theta);
//                    ROS_INFO_THROTTLE(1, "go to the retry point , yaw_err: %.3f", yaw_err);
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = yaw_err * 1.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -1.0), 1.0);
                if(std::fabs(angles::to_degrees(yaw_err)) < 10)
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>());
                    RCLCPP_INFO(machine->node_->get_logger() ,"reached retry point");
                }
            }
            else
            {
                dis_err = std::hypot(robot_pose.x - machine->target_pose_.x, robot_pose.y - machine->target_pose_.y);
                heading = std::atan2(machine->target_pose_.y - robot_pose.y, machine->target_pose_.x - robot_pose.x );
                angle_err = angles::shortest_angular_distance(robot_pose.theta, heading);

                cmd_vel.linear.x = dis_err * 1.0;
                if(angle_err < 0.2 && angle_err > -0.2)
                    cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, 0.1), 0.2);
                else
                    cmd_vel.linear.x = 0.0;

                cmd_vel.angular.z = angle_err * 1.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -1.0), 1.0);

                if(dis_err < 0.05)
                    reached_xy = true;
            }
            break;
        }
        case DOCK_IN:
        case DOCK_STABILIZING:
        case DISTANCE_OUT:
        {
            if(machine->obstacle_){
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                if(machine->valid_sport_check(10)){
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000,"GOTO_RETRY_POINT_STATE, DISTANCE_OUT,obstacle...");
                }
                else {
                    RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000,"GOTO_RETRY_POINT_STATE, DISTANCE_OUT,obstacle,timeout 10s...");
                    machine->setEvent(OBSTACLE);
                    machine->chargeState(std::make_unique<FAIL_STATE>());
                }
                return;
            }
            machine->resetOscillationTime();    // todo timeout
            if(reached_xy)
            {
                yaw_err = angles::shortest_angular_distance(robot_pose.theta, machine->target_pose_.theta);
//                    ROS_INFO_THROTTLE(1, "go to the retry point , yaw_err: %.3f", yaw_err);
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = yaw_err * 1.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -1.0), 1.0);
                if(std::fabs(angles::to_degrees(yaw_err)) < 10)
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    if(machine->retry_num_){
                        machine->chargeState(std::make_unique<FAIL_STATE>());
                    }else{
                        machine->retry_num_++;
                        machine->setEvent(REACHED_SECOND_POINT);
                        machine->chargeState(std::make_unique<SCAN_DOCKER_POSE>());
                        RCLCPP_INFO(machine->node_->get_logger(), "reached retry point");
                    }
                }
            }
            else
            {
                dis_err = std::hypot(robot_pose.x - machine->target_pose_.x, robot_pose.y - machine->target_pose_.y);
                heading = std::atan2(machine->target_pose_.y - robot_pose.y, machine->target_pose_.x - robot_pose.x );
                angle_err = angles::shortest_angular_distance(robot_pose.theta, heading);

                cmd_vel.linear.x = dis_err * 1.0;
                if(angle_err < 0.2 && angle_err > -0.2)
                    cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, 0.1), 0.1);
                else
                    cmd_vel.linear.x = 0.0;

                cmd_vel.angular.z = angle_err * 1.0;
                cmd_vel.angular.z = std::min(std::max(cmd_vel.angular.z, -1.0), 1.0);

                if(dis_err < 0.01)
                    reached_xy = true;
            }
            break;
        }
        default:
            break;
    }
}

void CHARGING_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case DOCK_IN:
            RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1, "DOCK_IN, CHARGING_STATE: %d", machine->chargingState_);
            cmd_vel.linear.x = -0.01;
            cmd_vel.angular.z = 0;
            if(machine->chargingState_)
            {
                machine->resetOscillationTime();
                if(dock_stabilizer++ >= 2){     // 防止抖动
                    dock_stabilizer = 0;
                    cmd_vel.linear.x = 0;
                    machine->setEvent(DOCK_STABILIZING);
                    break;
                }
            }
            if(!machine->valid_sport_check(2)){
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1, "CHARGING_STATE, dock in timeout 2s...");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                machine->chargeState(std::make_unique<SEPARATE_STATE>());
            }
            break;
        case DOCK_STABILIZING:
            RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1, "DOCK_STABILIZING, CHARGING_STATE: %d", machine->chargingState_);
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            if(machine->chargingState_)
            {
                machine->resetOscillationTime();
                if(dock_stabilizer++ >= 10){    //连续检测到充电状态则认为对接成功
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    machine->setEvent(DOCKING_DONE);
                    machine->chargeState(std::make_unique<CHARGING_DONE>());
                    break;
                }
            }else {
                dock_stabilizer = 0;    // continue 10 counts
            }
            if(!machine->valid_sport_check(3)){
                RCLCPP_WARN_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "CHARGING_STATE, dock stabilizing timeout 3s...");
                machine->setEvent(DOCK_IN);
            }
            break;
        default:
            break;
    }
}

void CHARGING_DONE::Handle(DOCKINGMACHINE *machine,
                           const geometry_msgs::msg::Pose2D &robot_poses,
                           geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case DOCKING_DONE:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            break;
        default:
            break;
    }
}
void FAIL_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    switch (machine->getEvent())
    {
        case REACHED_FIRST_POINT:
        case REACHED_SECOND_POINT:
        case REACHED_THIRD_POINT:
        case REACHED_RETRY_POINT:
        case DISTANCE_OUT:
            break;
        case OBSTACLE:
            break;
        case RETRY_FAIL:
            break;
        case DOCK_POSE_UNAVAILABLE:
            break;
        case ERROR_DOCK_POSE:
            break;
        case DOCKER_NO_POWER:
            break;
        case RECV_SEPARATE_CMD:
            break;
        case SEPARATE_STUCK:
            break;
        case REACHED_SEPARATE_RETRY_POINT:
            break;
        default:
            break;
    }
}

void SEPARATE_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    distance += machine->poseUpdate_.x;
    RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "separating, distance: %.3f", distance);
    if(!machine->valid_sport_check(20)){
        RCLCPP_ERROR_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "SEPARATE_STATE FAILED. timeout 20s");
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0.0;
        machine->obstacle_stuck_=1;
        machine->separated_dis_ = distance;
        machine->chargeState(std::make_unique<FAIL_STATE>());
        return;
    }
    if(machine->motor_error_)
        machine->motor_ctrl(true);
    switch (machine->getEvent())
    {
        case RECV_SEPARATE_CMD:
            if(machine->docking_forward_){
                cmd_vel.linear.x = -0.05;
                cmd_vel.angular.z = 0.0;
            }else {
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = 0.0;
            }
            if(distance > 0.25)
            {   
                RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "distance > 0.25 separating done, distance: %.3f", distance);
                cmd_vel.linear.x = 0;
                machine->setEvent(SEPARATE_DONE);
                machine->chargeState(std::make_unique<SEPARATE_DONE_STATE>());
                machine->obstacle_stuck_=0;
            }
            break;
        case DISTANCE_OUT:
            if(machine->docking_forward_){
                cmd_vel.linear.x = -0.05;
                cmd_vel.angular.z = 0.0;
            }else {
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = 0.0;
            }
            if(distance > 0.25)
            {
                cmd_vel.linear.x = 0;
                machine->chargeState(std::make_unique<GOTO_RETRY_POINT_STATE>());
            }
            break;
        default:
            break;
    }
}



void GOTO_SEPARATE_RECOVER_POINT_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{   distance += machine->poseUpdate_.x;
    RCLCPP_INFO_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "go to separate recover, distance: %.3f", distance);
    if(!machine->valid_sport_check(10)){
        RCLCPP_ERROR_THROTTLE(machine->node_->get_logger(), *machine->node_->get_clock() ,1000, "SEPARATE_STUCK. recovery timeout 10s");
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0.0;
        machine->obstacle_stuck_=1;
        machine->chargeState(std::make_unique<FAIL_STATE>());
        return;
    }
    if(machine->motor_error_)
        machine->motor_ctrl(true);
    switch (machine->getEvent())
    {     
        case SEPARATE_STUCK: 
            
            if(machine->docking_forward_){
                cmd_vel.linear.x = -0.05;
                cmd_vel.angular.z = 0.0;
            }else {
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = 0.0;
            }
            if(machine->obstacle_)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                machine->separated_dis_+=distance;
                //ROS_WARN("separated_dis_:%.3f", machine->separated_dis_);

            }
            if(machine->separated_dis_>0.45)
            {  
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                //ROS_WARN("reach saparte recover pos"); 
                machine->setEvent(SEPARATE_DONE);
                machine->chargeState(std::make_unique<SEPARATE_DONE_STATE>());
                machine->obstacle_stuck_=0;    
            }
            break;
        default:
            break;
    }
}



void SEPARATE_DONE_STATE::Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel)
{
    switch (machine->getEvent())
    {
        case SEPARATE_DONE:
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            break;
        default:
            break;
    }
}

void OBSTACLE_STATE::Handle(DOCKINGMACHINE *machine,
                            const geometry_msgs::msg::Pose2D &robot_pose,
                            geometry_msgs::msg::Twist &cmd_vel)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
}


