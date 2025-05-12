#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include "crazyflie_interfaces/msg/position.hpp"

using namespace std::chrono_literals;

class Cmd_Position_Publisher : public rclcpp::Node
{
public:
    Cmd_Position_Publisher() : Node("follower_node"), time_cnt(0.0), time_real(0.0), step_idx(0), triggered(false)
    {
        cmd_position_publisher_follower = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf02/cmd_position", 10);
        pose_heading_publisher_follower = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cf02/pose_heading", 10);
        trigger_publisher_follower = this->create_publisher<std_msgs::msg::Int32>("/csv_trigger", 10);
        goal_subscriber_follower = this->create_subscription<geometry_msgs::msg::Pose>(
            "/cf02/goal", 10, std::bind(&Cmd_Position_Publisher::goal_callback, this, std::placeholders::_1));
        state_subscriber_follower = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cf02/pose", 10, std::bind(&Cmd_Position_Publisher::state_callback, this, std::placeholders::_1));
        command_loop_timer_follower = this->create_wall_timer(10ms, std::bind(&Cmd_Position_Publisher::command_loop_callback, this));
        global_xyz_cmd = Eigen::VectorXd::Zero(4);
        actual_pose = geometry_msgs::msg::Pose();
    }

private:
    void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (triggered == true)
        {
            //global_xyz_cmd[0] = msg->position.x+1.0; //for flow-deck
            global_xyz_cmd[0] = msg->position.x; //for optitrack
            global_xyz_cmd[1] = msg->position.y;
            global_xyz_cmd[2] = msg->position.z;
            global_xyz_cmd[3] = msg->orientation.z;
        }
    }

    void state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        actual_pose = msg->pose;
    }

    void command_loop_callback()
    {
        time_cnt++;
        time_real = time_cnt * 0.01;

        if (time_real < 1.0) global_xyz_cmd[2] = -0.05;
        else if (time_real > 1.2)
        {
            if(global_xyz_cmd[2] < 0.6)
            {
                global_xyz_cmd[2] += 0.01;
                global_xyz_cmd[0] = -1.0;
                global_xyz_cmd[1] = 0.0;
            }
            else global_xyz_cmd[2] = 0.6;
        }

        /*if (time_real>7 && step_idx <= 400) {  //for flow-deck
            global_xyz_cmd[0] = step_idx * 0.005;
            global_xyz_cmd[2] = 0.6;  
            step_idx++;
        }*/
        
        if (time_real>7 && step_idx <= 400) {//for optitarck
            global_xyz_cmd[0] = -1.0+step_idx * 0.005;
            global_xyz_cmd[2] = 0.6;
            step_idx++;
        }
        else if (triggered == false && std::abs(actual_pose.position.x - 1.0) < 0.05 && std::abs(actual_pose.position.y) < 0.01) {
            std_msgs::msg::Int32 msg;
            msg.data = 1100;
            triggered = true;
            trigger_publisher_follower->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Follower reached 1.0 (actual), sent trigger 1100");
        }

        crazyflie_interfaces::msg::Position global_xyz_cmd_msg;
        global_xyz_cmd_msg.x = global_xyz_cmd[0];
        global_xyz_cmd_msg.y = global_xyz_cmd[1];
        global_xyz_cmd_msg.z = global_xyz_cmd[2];
        global_xyz_cmd_msg.yaw = global_xyz_cmd[3];
        cmd_position_publisher_follower->publish(global_xyz_cmd_msg);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "CF02";
        pose_msg.pose = actual_pose;
        pose_heading_publisher_follower->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "%lf, %lf %lf %lf", time_real, actual_pose.position.x, actual_pose.position.y, actual_pose.position.z);
    }

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cmd_position_publisher_follower;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_heading_publisher_follower;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr trigger_publisher_follower;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscriber_follower;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_subscriber_follower;
    rclcpp::TimerBase::SharedPtr command_loop_timer_follower;

    Eigen::VectorXd global_xyz_cmd;
    geometry_msgs::msg::Pose actual_pose;
    double time_cnt;
    double time_real;
    int step_idx;
    bool triggered;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cmd_Position_Publisher>());
    rclcpp::shutdown();
    return 0;
}

