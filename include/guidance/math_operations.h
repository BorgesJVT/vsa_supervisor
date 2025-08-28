#ifndef __GUIDANCE_MATH_OPERATIONS__
#define __GUIDANCE_MATH_OPERATIONS__

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "types.h"
#include "../plan_db/maneuver.h"

namespace vsa_guidance
{
    double unwrap_angle(double value);

    orientation_t get_euler_angles(geometry_msgs::msg::Quaternion *quaternion);

    odometry_t msg_odometry_to_odometry_t(nav_msgs::msg::Odometry msg);

    double map(double value, double from_lower, double from_upper, double to_low, double to_upper);
}

#endif
