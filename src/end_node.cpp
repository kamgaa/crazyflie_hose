#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"

#include "std_msgs/msg/bool.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class Cmd_Position_Publisher : public rclcpp::Node
{
public:
    Cmd_Position_Publisher() : Node("end_node"), time_cnt(0.0), time_real(0.0), step_idx(0), triggered(false)
    {
        /*arm_client_f = this->create_client<crazyflie_interfaces::srv::Arm>("/cf02/arm");
        if (!arm_client_f->wait_for_service(10s)) {
            //RCLCPP_ERROR(this->get_logger(), "Arm 서비스가 준비되지 않았습니다!");
            rclcpp::shutdown();
            return;
        }
        arm_timer_ = this->create_wall_timer(100ms, [this]{
  	    arm_timer_->cancel();
  	    sendArmRequest(true);
	});*/
        //sendArmRequest(true);                // 즉시 Arm
        //arm_called_time_f = this->now();      // Arm 호출 시각 저장
        
        //TODO -> 3 agent flight : node name change 


        cmd_position_publisher_end = this->create_publisher<crazyflie_interfaces::msg::Position>("/cf04/cmd_position", 10);
        pose_heading_publisher_end = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cf03/pose_heading", 10);
        trigger_publisher_end = this->create_publisher<std_msgs::msg::Int32>("/csv_trigger", 10);
        goal_subscriber_end = this->create_subscription<geometry_msgs::msg::Pose>(
            "/cf02/goal", 10, std::bind(&Cmd_Position_Publisher::goal_callback, this, std::placeholders::_1));
        state_subscriber_end= this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cf02/pose", 10, std::bind(&Cmd_Position_Publisher::state_callback, this, std::placeholders::_1));
        command_loop_timer_end= this->create_wall_timer(10ms, std::bind(&Cmd_Position_Publisher::command_loop_callback, this));
        global_xyz_cmd = Eigen::VectorXd::Zero(4);
        actual_pose = geometry_msgs::msg::Pose();
        
        auto armed_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
	armed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cf03/armed", armed_qos);
        arm_client_ = this->create_client<crazyflie_interfaces::srv::Arm>("/cf03/arm"); // 팔로워는 "/cf02/arm"
	arm_timer_ = this->create_wall_timer(100ms, std::bind(&Cmd_Position_Publisher::arm_tick, this));
	auto start_qos = rclcpp::QoS(1).reliable().transient_local();
    	hover_start_sub_ = this->create_subscription<builtin_interfaces::msg::Time>(
    "/hover_start_time", start_qos,
    [this](const builtin_interfaces::msg::Time::SharedPtr t){
        hover_start_time_ = rclcpp::Time(t->sec, t->nanosec, RCL_ROS_TIME);
        //RCLCPP_INFO(this->get_logger(), "Hover start time received: %d.%09u", t->sec, t->nanosec);
    });
    }

private:
    /*void sendArmRequest(bool arm)
    {
        auto req = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
        req->arm = arm;
        auto fut = arm_client_f->async_send_request(req);
        if (fut.wait_for(2s) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), arm ? "→ Drone armed." : "→ Drone disarmed.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arm 서비스 호출 실패");
        }
    }*/
   /*void sendArmRequest(bool arm)
	{
	    auto req = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
	    req->arm = arm;
	    auto fut = arm_client_f->async_send_request(req);

	    auto rc = rclcpp::spin_until_future_complete(
		this->get_node_base_interface(), fut, 5s);

	    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_INFO(this->get_logger(), arm ? "→ Drone armed." : "→ Drone disarmed.");
		std_msgs::msg::Bool msg;
		msg.data = arm;              // true = armed, false = disarmed
		armed_pub_->publish(msg);
	    }
	    else if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
	//	RCLCPP_WARN(this->get_logger(),
	//	    "Arm 서비스 응답 지연(타임아웃). 실제 기체는 ARM/비행했을 수 있음. 서버/상태를 확인하세요.");
		std_msgs::msg::Bool msg; msg.data = false; armed_pub_->publish(msg);
	    }
	    else { // INTERRUPTED 등
		RCLCPP_ERROR(this->get_logger(), "Arm 서비스 호출 실패.");
		std_msgs::msg::Bool msg; msg.data = false; armed_pub_->publish(msg);
	    }
	}*/


    
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

    void arm_tick() {
  // 서비스 준비될 때까지 짧게 폴링 (블록 금지)
  	if (!arm_req_sent_) {
	    if (!arm_client_->wait_for_service(0s)) {  // 즉시 리턴
	      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[ARM] waiting for service...");
	      return;
	    }
	    auto req = std::make_shared<crazyflie_interfaces::srv::Arm::Request>();
	    req->arm = true;
	    arm_client_->async_send_request(
	      req,
	      [this](rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedFuture) {
		// 응답 도착(성공 가정). 필요하면 resp 필드 확인.
		std_msgs::msg::Bool m; m.data = true;
		armed_pub_->publish(m);
		arm_ok_ = true;
		RCLCPP_INFO(this->get_logger(), "→ Drone armed (async).");
	      });
	    arm_req_sent_ = true;
	    RCLCPP_INFO(this->get_logger(), "[ARM] request sent.");
	    return;
	  }
	  if (arm_ok_) {
	    arm_timer_->cancel(); // 한 번만
	  }
    }
    void command_loop_callback()
    {
        if (!arm_ok_) return;
        if (hover_start_time_.nanoseconds() == 0) return;
        
        if (!hover_started_) 
        {
	    if (this->now() < hover_start_time_) return;
	    hover_started_ = true;
	    time_cnt = 0.0;       // 두 노드 타이머 동기화(선택)
	    time_real = 0.0;
	    //RCLCPP_INFO(this->get_logger(), "Hover START!");
	}

        
        time_cnt++;
        time_real = time_cnt * 0.01;
        //const double arm_delay = (this->now() - arm_called_time_f).seconds();

        if (time_real < 1.0)
        {
            global_xyz_cmd[2] = 0.0;
            global_xyz_cmd[0] = -0.23; 
        }
        else if (time_real > 1.5)
        {
            if(global_xyz_cmd[2] < 1.0)
            {
                
                global_xyz_cmd[0] = -0.23; // x command
                global_xyz_cmd[1] = 0.0; // y command
                global_xyz_cmd[2] += 0.005; // z command (up)
            }
            else global_xyz_cmd[2] = 1.0;
        }
        if (triggered == false && time_real>10 )
        {
            std_msgs::msg::Int32 msg;
            msg.data = 1110;
            triggered = true;
            trigger_publisher_follower->publish(msg);

            RCLCPP_INFO(this->get_logger(), "cf03 GO");
        
        
        
        }

        /*if (time_real>7 && step_idx <= 400) {  //for flow-deck
            global_xyz_cmd[0] = step_idx * 0.005;
            global_xyz_cmd[2] = 0.6;  
            step_idx++;
        }*/
        /*
        if (time_real>7 && step_idx <= 400) {//for optitarck
            global_xyz_cmd[0] = -1.0+step_idx * 0.005;
            global_xyz_cmd[2] = 0.6;
            step_idx++;
        }*/
        /*else if (triggered == false && std::abs(actual_pose.position.x - .0) < 0.05 && std::abs(actual_pose.position.y) < 0.01) {
            std_msgs::msg::Int32 msg;
            msg.data = 1100;
            triggered = true;
            trigger_publisher_follower->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Follower reached 1.0 (actual), sent trigger 1100");
        }*/

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

    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cmd_position_publisher_end;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_heading_publisher_end;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr trigger_publisher_end;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscriber_end;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr state_subscriber_end;
    rclcpp::TimerBase::SharedPtr command_loop_timer_end;
    
    rclcpp::Client<crazyflie_interfaces::srv::Arm>::SharedPtr arm_client_;
    //rclcpp::Time arm_called_time_f;
 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
    rclcpp::TimerBase::SharedPtr arm_timer_;


    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr hover_start_sub_;


    bool hover_started_ = false;
    bool arm_req_sent_ = false;
    bool arm_ok_ = false;   
    rclcpp::Time hover_start_time_; // 수신된 시작 시각

   
   // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
    Eigen::VectorXd 		global_xyz_cmd;
    geometry_msgs::msg::Pose 	actual_pose;
    double 			time_cnt;
    double 			time_real;
    int 			step_idx;
    bool 			triggered;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cmd_Position_Publisher>());
    rclcpp::shutdown();
    return 0;
}

