#include "../../include/guidance/guidance.h"

using namespace vsa_guidance;


Guidance::Guidance()
{
    thruster_controller_ = new PID();
    pitch_controller_ = new PID();
    yaw_controller_ = new PID();
}

Guidance::Guidance(guidance_pid_params_t thruster_controller_parameters,
                    guidance_pid_params_t pitch_controller_parameters,
                    guidance_pid_params_t yaw_controller_parameters,
                    double abs_xy_tolerance_error, double dt)
{
    thruster_controller_ = new PID(thruster_controller_parameters, dt);
    pitch_controller_ = new PID(pitch_controller_parameters, dt);
    yaw_controller_ = new PID(yaw_controller_parameters, dt);

    set_abs_xy_tolerance_error(abs_xy_tolerance_error);
}

Guidance::~Guidance()
{
    delete thruster_controller_;
    delete pitch_controller_;
    delete yaw_controller_;
}

guidance_setpoint Guidance::get_setpoint()
{
    return setpoint_;
}

guidance_state_e Guidance::get_state()
{
    return state_;
}

double Guidance::get_abs_xy_tolerance_error(void)
{
    return abs_xy_tolerance_error_;
}

PID* Guidance::get_thruster_controller()
{
    return thruster_controller_;
}

PID* Guidance::get_pitch_controller()
{
    return pitch_controller_;
}

PID* Guidance::get_yaw_controller()
{
    return yaw_controller_;
}

void Guidance::set_setpoint(guidance_setpoint setpoint)
{
    setpoint_ = setpoint;
    set_state(guidance_state_e::guidance_status_IN_PROGRESS);
}

void Guidance::set_abs_xy_tolerance_error(double value)
{
    abs_xy_tolerance_error_ = value;
}

void Guidance::set_state(guidance_state_e state)
{
    state_ = state;
}

guidance_output_t Guidance::execute(odometry_t current_odometry, double dt)
{
    guidance_output_t result;

    double error_pos_x = setpoint_.x - current_odometry.pose.x;
    double error_pos_y = setpoint_.y - current_odometry.pose.y;
    double error_pos_z = setpoint_.z - current_odometry.pose.z;

    double abs_xy_error = std::sqrt((error_pos_x * error_pos_x) + 
                                    (error_pos_y * error_pos_y));

    double desire_pitch = -std::atan2(error_pos_z, -std::abs(error_pos_x));
    double desire_yaw = std::atan2(error_pos_y, error_pos_x);

    if(abs_xy_error < abs_xy_tolerance_error_)
    {
        result.thruster = 0.0;
        result.horizontal_rudders = 0.0;
        result.vertical_rudders = 0.0;

        set_state(guidance_state_e::guidance_status_TARGET_REACHED);
    }
    else
    {
        result.thruster = thruster_controller_->compute(current_odometry.xy_abs_velocity,
                                                        setpoint_.speed,
                                                        dt);

        result.horizontal_rudders = pitch_controller_->compute(current_odometry.orientation.pitch,
                                                                desire_pitch,
                                                                dt);

        result.vertical_rudders = yaw_controller_->compute(current_odometry.orientation.yaw,
                                                            desire_yaw,
                                                            dt);

        set_state(guidance_state_e::guidance_status_IN_PROGRESS);
    }

    return result;
}

void Guidance::finish()
{
    set_state(guidance_state_e::guidance_status_IDLE);
}
