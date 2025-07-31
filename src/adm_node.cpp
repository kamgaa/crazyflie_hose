#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "crazyflie_interfaces/msg/position.hpp"
#include "std_msgs/msg/int32.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"


using namespace std::chrono_literals;

class Cmd_Position_Publisher : public rclcpp::Node
{
public:
    Cmd_Position_Publisher() : Node("leader_node"), time_cnt(0.0), time_real(0.0), step_idx(0), triggered(false),M_(0.2),D_(0.1),K_(0.0),x_adm(0.0),x_cmd(0.0)
    {
        arm_client_ = this->create_client<crazyflie_interfaces::srv::Arm>("/cf01/arm");
        if (!arm_client_->wait_for_service(3s)) {
            //RCLCPP_ERROR(this->get_logger(), "Arm 서비스가 준비되지 않았습니다!");
            rclcpp::shutdown();
            return;
        }
        sendArmRequest(true);                // 즉시 Arm
        arm_called_time_ = this->now();      // Arm 호출 시각 저장
        
        
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
    void sendArmRequest(bool arm)
    {
        auto req = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
        req->arm = arm;
        auto fut = arm_client_->async_send_request(req);
        if (fut.wait_for(2s) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), arm ? "→ Drone armed." : "→ Drone disarmed.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arm 서비스 호출 실패");
        }
    }

    
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
        
        const double arm_delay = (this->now() - arm_called_time_).seconds();

        if (time_real < 1.0)global_xyz_cmd[2] = 0.0;
        
        /*else if (time_real > 2.0)
        {
            if(global_xyz_cmd[2] < 0.6)
            {
                
                global_xyz_cmd[0] = 0.0; // x command
                global_xyz_cmd[1] = 0.0; // y command
                global_xyz_cmd[2] += 0.01; // z command (up)
            }
            else global_xyz_cmd[2] = 0.6;
        }*/
        
        else if (time_real > 2.0 && time_real < 5)
        {
            if(global_xyz_cmd[2] < 0.9)
            {
                
                global_xyz_cmd[0] = 0.0; // x command
                global_xyz_cmd[1] = 0.0; // y command
                global_xyz_cmd[2] += 0.01; // z command (up)
            }
            else global_xyz_cmd[2] = 0.9;
        }
        
        else if (time_real <=30 && time_real >15)
        {
            double F_ext = -20.0;      // 외력
            double dt    = 0.01;       // time_real 증가 단위와 일치
        	// x_2는 admittance 변위, x_2_dot는 admittance 속도
            x_1_dot = F_ext - D_/M_ * x_1 - K_/M_ * x_2 ; //y축에 적용함.
	    x_2_dot = x_1;
	    x_1 += x_1_dot*dt;
            x_2 += x_2_dot*dt;
            x_adm = -1.0/M_*x_2;
        	// 보정 목표 위치
            double x_cmd = x_des + x_adm;
            
            global_xyz_cmd[0] = 0.0; // x command
            global_xyz_cmd[1] = x_cmd; // y command
            global_xyz_cmd[2] = 0.9; // z command (up)
        }
        else if (time_real >20)
        {
            global_xyz_cmd[2] -= 0.01;
            if(global_xyz_cmd[2]<0.02)
            {
                global_xyz_cmd[2] =-0.3;
            }
        }
        
        /*if (triggered == false && time_real>7.0)
        {
            std_msgs::msg::Int32 msg;
            msg.data = 1111;
            triggered = true;
            trigger_publisher_leader->publish(msg);

            RCLCPP_INFO(this->get_logger(), "cf01 GO");
        
        
        
        }*/
        
        
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

        RCLCPP_INFO(this->get_logger(), "%f, %f %f triggered: %d", actual_pose.position.x,actual_pose.position.y,actual_pose.position.z, triggered ? 1:0);
	
	
        
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
    rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr arm_client_;
    rclcpp::Time arm_called_time_;
    Eigen::VectorXd 		global_xyz_cmd;
    geometry_msgs::msg::Pose 	actual_pose;
    double 			time_cnt;
    double 			time_real;
    int 			step_idx;
    bool 			triggered;
    
    	double			x_1 = 0.0;
	double			x_2 = 0.0;
	double			x_1_dot = 0.0;
	double			x_2_dot = 0.0;
	double			x_des = 0.0;
	double			F_x_ext = -20.0; //-20N
        double			M_, D_, K_;
	double			x_adm, x_cmd;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cmd_Position_Publisher>());
    rclcpp::shutdown();
    return 0;
}

