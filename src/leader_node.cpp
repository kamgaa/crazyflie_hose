#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "crazyflie_interfaces/msg/position.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class Cmd_Position_Publisher : public rclcpp::Node
{
public:
    Cmd_Position_Publisher() : Node("leader_node"), time_cnt(0.0), time_real(0.0), step_idx(0), triggered(false)
    {
        cmd_position_publisher_leader = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf01/cmd_position", 10);
        pose_heading_publisher_leader = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cf01/pose_heading", 10);
        trigger_publisher_leader = this->create_publisher<std_msgs::msg::Int32>("/csv_trigger", 10);
        
        goal_subscriber_leader = this->create_subscription<geometry_msgs::msg::Pose>(
            "/cf01/goal", 10, std::bind(&Cmd_Position_Publisher::goal_callback, this, std::placeholders::_1));
	state_subscriber_leader = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cf01/pose", 10, std::bind(&Cmd_Position_Publisher::state_callback, this, std::placeholders::_1));
        command_loop_timer_leader = this->create_wall_timer(10ms, std::bind(&Cmd_Position_Publisher::command_loop_callback, this));
        global_xyz_cmd = Eigen::VectorXd::Zero(4);
        actual_pose = geometry_msgs::msg::Pose();

    }

private:
    void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (triggered)
        {
            global_xyz_cmd[0] = msg->position.x;
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

        // 호버링 로직 (5초간)
        if (time_real < 1.0) global_xyz_cmd[2] = -0.05;
        else if (time_real > 1.5)
        {
            if(global_xyz_cmd[2]<0.6)
            {
                global_xyz_cmd[2]=global_xyz_cmd[2]+0.01;
                global_xyz_cmd[0]=0;
                global_xyz_cmd[1]=0;
            }
            else global_xyz_cmd[2]=0.6;
        }
        
        if (time_real>7 && step_idx <= 205) {
            global_xyz_cmd[0] = step_idx * 0.005;
            global_xyz_cmd[2] = 0.6;
            step_idx++;
        }
        /*else if (time_real < 1.1) global_xyz_cmd[2] = 0.05;
        else if (time_real < 1.2) global_xyz_cmd[2] = 0.1;
        else if (time_real < 1.3) global_xyz_cmd[2] = 0.125;
        else if (time_real < 1.4) global_xyz_cmd[2] = 0.15;
        else if (time_real < 1.5) global_xyz_cmd[2] = 0.2;
        else if (time_real < 1.6) global_xyz_cmd[2] = 0.25;

        else if (time_real < 1.7) global_xyz_cmd[2] = 0.3;
        else if (time_real < 1.8) global_xyz_cmd[2] = 0.35;
        else if (time_real < 1.9) global_xyz_cmd[2] = 0.4;
        else if (time_real < 2.0) global_xyz_cmd[2] = 0.5;
        else if (time_real < 2.1) global_xyz_cmd[2] = 0.6;
        
        else if (time_real >= 2.1 && time_real < 3.1) {
        	global_xyz_cmd[2] = 0.6;
        	global_xyz_cmd[0] = 0.0;  // x 고정
    	}*/

    // 고도 유지 후 x 방향으로 1미터 전진
    	/*if (step_idx <= 100&&time_real > 7.0) {
        	global_xyz_cmd[2] = 0.6;
        	
        	if (global_xyz_cmd[0] < 1.0){
        	global_xyz_cmd[0]=global_xyz_cmd[0]+0.01;
        	
        	}
        	else if (global_xyz_cmd[0] == 1.0)  global_xyz_cmd[0]=1.0;
            global_xyz_cmd[0] = step_idx * 0.01;
            global_xyz_cmd[2] = 0.6;
            step_idx++;
   	}*/
        
        //else if (time_real > 7.5) global_xyz_cmd[2] = 0.6; // 고도 유지 (hover)
	// 착륙은???????????코드 반영할것, follower node 에도 똑같이 반영할것!!!
        // 5초 이후부터 goal 명령 반영
        else if (triggered == false && std::abs(actual_pose.position.x - 1.0) < 0.05 && std::abs(actual_pose.position.y) < 0.01) {
            std_msgs::msg::Int32 msg;
            msg.data = 1111;
            triggered = true;
            trigger_publisher_leader->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Leader reached 1.0 (actual), sent trigger 1111");
        }
        
        // 목표 명령 퍼블리시
        crazyflie_interfaces::msg::Position global_xyz_cmd_msg;
        global_xyz_cmd_msg.x = global_xyz_cmd[0];
        global_xyz_cmd_msg.y = global_xyz_cmd[1];
        global_xyz_cmd_msg.z = global_xyz_cmd[2];
        global_xyz_cmd_msg.yaw = global_xyz_cmd[3];
        cmd_position_publisher_leader->publish(global_xyz_cmd_msg);

        // 현재 위치 + heading 퍼블리시 (현재는 목표 = 현재라고 가정)
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "CF01";
        pose_msg.pose.position.x = global_xyz_cmd[0];
        pose_msg.pose.position.y = global_xyz_cmd[1];
        pose_msg.pose.position.z = global_xyz_cmd[2];
        pose_msg.pose.orientation.z = global_xyz_cmd[3];  // yaw 단일 저장
        pose_heading_publisher_leader->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "%lf, %lf %lf %lf triggered: %d", time_real, actual_pose.position.x,actual_pose.position.y,actual_pose.position.z,triggered ? 1 : 0);

        
    }

    /*void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        global_xyz_cmd[0] = msg->position.x;
        global_xyz_cmd[1] = msg->position.y;
        global_xyz_cmd[2] = msg->position.z;
        global_xyz_cmd[3] = msg->orientation.z; // yaw (단일값으로 전달 중)
    }*/

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cmd_position_publisher_leader;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_heading_publisher_leader;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr trigger_publisher_leader;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscriber_leader;
    rclcpp::TimerBase::SharedPtr command_loop_timer_leader;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_subscriber_leader;

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

