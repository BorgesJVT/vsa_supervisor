#include "../../include/guidance/math_operations.h"

double vsa_guidance::unwrap_angle(double value)
{
    double result = std::atan2(std::sin(value), std::cos(value));
    return result;
}

vsa_guidance::orientation_t vsa_guidance::get_euler_angles(geometry_msgs::msg::Quaternion *quaternion)
{
    vsa_guidance::orientation_t result;

    tf2::Quaternion _quaternion(quaternion->x,
                                quaternion->y,
                                quaternion->z,
                                quaternion->w);
    
    tf2::Matrix3x3 m_quaternion;
    m_quaternion.setRotation(_quaternion);

    m_quaternion.getRPY(result.roll, result.pitch, result.yaw);

    return result;
}

vsa_guidance::odometry_t vsa_guidance::msg_odometry_to_odometry_t(nav_msgs::msg::Odometry msg)
{
    vsa_guidance::odometry_t result;

    result.pose.x = msg.pose.pose.position.x;
    result.pose.y = msg.pose.pose.position.y;
    result.pose.z = msg.pose.pose.position.z;

    result.linear_velocity.x = msg.twist.twist.linear.x;
    result.linear_velocity.y = msg.twist.twist.linear.y;
    result.linear_velocity.z = msg.twist.twist.linear.z;

    result.orientation = vsa_guidance::get_euler_angles(&msg.pose.pose.orientation);

    result.angular_velocity.x = msg.twist.twist.angular.x;
    result.angular_velocity.y = msg.twist.twist.angular.y;
    result.angular_velocity.z = msg.twist.twist.angular.z;

    result.xy_abs_velocity = std::sqrt((msg.twist.twist.linear.x * msg.twist.twist.linear.x) +
                                        (msg.twist.twist.linear.y) * msg.twist.twist.linear.y);

    return result;
}

double vsa_guidance::map(double value, double from_lower, double from_upper, double to_low, double to_upper)
{
    double result = (value - from_lower) * (to_upper - to_low) / (from_upper - from_lower) + to_low;
}