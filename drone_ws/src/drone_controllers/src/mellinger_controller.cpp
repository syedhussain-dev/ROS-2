#include "controllers/mellinger_controller.hpp"
#include <Eigen/Geometry>
using std::placeholders::_1;

MellingerController::MellingerController(const std::string& name): 
    Node(name)

{
    declare_parameter("mass",0.138);
    declare_parameter("g",9.81);
    declare_parameter("L",0.12);
    declare_parameter("kF",3.0e-5);
    declare_parameter("kM",1.1e-6);
    declare_parameter("Kp",2.0);
    declare_parameter("Kv",1.0);
    declare_parameter("Kr",0.1);
    declare_parameter("Kw",0.05);
    declare_parameter("show_logs",false);

    mass = get_parameter("mass").as_double();
    g = get_parameter("g").as_double();
    L = get_parameter("L").as_double();
    kF = get_parameter("kF").as_double();
    kM = get_parameter("kM").as_double();
    Kp = get_parameter("Kp").as_double() * Eigen::Matrix3d::Identity();
    Kv = get_parameter("Kv").as_double() * Eigen::Matrix3d::Identity();
    Kr = get_parameter("Kr").as_double() * Eigen::Matrix3d::Identity();
    Kw = get_parameter("Kw").as_double() * Eigen::Matrix3d::Identity();
    show_logs = get_parameter("show_logs").as_boolean();

    motor_pub_ = create_publisher<actuator_msgs::msg::Actuators>("/simple_velocity_controller/commands",10);
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("/mellinger_controller/odom",10,std::bind(&MellingerController::odomCallback,this,_1));
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/mellinger_controller/cmd_vel",10,std::bind(&MellingerController::velCallback,this,_1));
    
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "root";
    int log_iterator = 0;

    M_ << kF << kF << kF << kF << 0 << -(kF*L) << 0 << (kF*L) << -(kF*L) << 0 << (kF*L) << 0 << -kM << kM << -kM << kM
    inv_M_ = M_.inverse();

    RCLCPP_INTO_STREAM(get_logger(), "C++ Implemenation of Mellinger Controller Running...")
    auto timer = rclcpp::create_timer(this, this->get_clock(), 0.01, std::bind(&MellingerController::controlLoop, this, _1));
}

void MellingerController::controlLoop(){
    log_iterator += 1;
    yaw_T_ += yaw_rate_T_ * 0.01;
    Eigen::Vector3d i_B = R_.col(0);
    Eigen::Vector3d h{i_B[0], i_B[1], 0};
    h /= h.norm();
    Eigen::RowVector3d h_T{std::cos(yaw_T_), std::sin(yaw_T_), 0.0};

    Eigen::RowVector3d weight{0.0, 0.0, mass * g};
    auto Fdes = -Kp * (r_ - r_T_) -Kv * (vel_ - vel_T_) + weight;
    auto Pdes = Fdes.dot(R_.col(2));
    Eigen::Matrix3d Rdes = Eigen::Matrix3d::Zero();
    Eigen::RowVector3d k{0.0, 0.0, 1.0};
    Rdes.col(2) = Fdes / Fdes.norm();
    Rdes.col(0) = Rdes.col(2).cross(k.cross(h_T));
    Rdes.col(1) = Rdes.col(2).cross(Rdes.col(0));

    Eigen::Matrix3d temp_arr = Rdes.transpose() * R_ - R_.transpose() * Rdes;
    Eigen::RowVector3d eR{temp_arr(1,0),temp_arr(0,2),temp_arr(2,1)};
    eR *= 0.5;
    Eigen::RowVector3d eW = w_;

    auto Tdes = -Kr * eR - Kw * eW;
    auto tau_body = R_.transpose() * Tdes;
    Eigen::Vector4d temp_arr_2{Pdes, tau_body.row(0), tau_body.row(1), tau_body.row(2)};
    auto rotor_speed_sq_ = inv_M_ * temp_arr_2;
    double rs[4]{std::max(0.0,rotor_speed_sq_.row(0)),std::max(0.0,rotor_speed_sq_.row(1)),std::max(0.0,rotor_speed_sq_.row(2),
    ),std::max(0.0,rotor_speed_sq_.row(3))};

    if(log_iterator % 20 == 0 && show_logs){
        RCLCPP_INTO_STREAM(get_logger(),"R_ = "<< R_);
        RCLCPP_INTO_STREAM(get_logger(),"eR = "<< eR);
        RCLCPP_INTO_STREAM(get_logger(),"eW = "<< eW);
        RCLCPP_INTO_STREAM(get_logger(),"Fdes = "<< Fdes);
        RCLCPP_INTO_STREAM(get_logger(),"Pdes = "<< Pdes);
        RCLCPP_INTO_STREAM(get_logger(),"Tdes = "<< Tdes);
        RCLCPP_INTO_STREAM(get_logger(),"tau_body = "<< tau_body);
        RCLCPP_INTO_STREAM(get_logger(),"rotor_speed_sq_ = "<< rotor_speed_sq_);
        RCLCPP_INTO_STREAM(get_logger(),"rotor_speeds = "<< rs);
    }

    actuator_msgs::msg::Actuators msg;
    msg.velocity(rs[0],rs[1],rs[2],rs[3]);
    motor_pub_->publish(msg);

}

void MellingerController::velCallback(const geometry_msgs::msg::TwistStamped& msg){
    r_T_ << msg.twist.linear.x << msg.twist.linear.y << msg.twist.linear.z
    yaw_T_ << msg.twist.angular.z
}

void MellingerController::odomCallback(const nav_msgs::msg::Odometry& msg){
    r_ << msg.pose.pose.position.x << msg.pose.pose.position.y << msg.pose.pose.position.z
    vel_ << msg.twist.twist.linear.x << msg.twist.twist.linear.y << msg.twist.twist.linear.z
    w_ << msg.twist.twist.angular.x << msg.twist.twist.angular.y << msg.twist.twist.angular.z
    Eigen::Quaterniond q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w);
    R_ = q.toRotationMatrix();

    transform_stamped_.transform.translation.x = r_[0];
    transform_stamped_.transform.translation.y = r_[1];
    transform_stamped_.transform.translation.z = r_[2];
    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    broadcaster_->sendTransform(transform_stamped_);

}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MellingerController>("mellinger_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}