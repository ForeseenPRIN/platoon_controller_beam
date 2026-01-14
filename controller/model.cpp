
#include "fmu4cpp/fmu_variable.hpp"
#include "pid.hpp"
#include <algorithm>
#include <cmath>
#include <fmu4cpp/fmu_base.hpp>
#include <numbers>
#include <array>
#include <numeric>

using namespace fmu4cpp;


class Controller : public fmu_base {

public:
    static constexpr double DT = 5e-3;
    static constexpr size_t N_SAMPLES = 100;

    FMU4CPP_CTOR(Controller) {

        // Input variables
        register_variable(
                real("a_des", &a_des)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("posX", &posX_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("posY", &posY_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("posZ", &posZ_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("velX", &velX_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("velY", &velY_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("velZ", &velZ_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("accX", &accX_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("accY", &accY_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("accZ", &accZ_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("roll", &roll_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("pitch", &pitch_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("yaw", &yaw_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        register_variable(
                real("groundSpeed", &groundSpeed_)
                        .setCausality(causality_t::INPUT)
                        .setVariability(variability_t::CONTINUOUS));

        // Attack parameters
        register_variable(
                real("attack_time", &attack_time_)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                integer("attack", &attack_)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("attack_amplitude", &attack_amplitude_)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        // Output variables
        register_variable(
                real("throttle", &throttle_)
                        .setCausality(causality_t::OUTPUT)
                        .setVariability(variability_t::CONTINUOUS)
                        .setInitial(initial_t::EXACT));

        register_variable(
                real("brake", &brake_)
                        .setCausality(causality_t::OUTPUT)
                        .setVariability(variability_t::CONTINUOUS)
                        .setInitial(initial_t::EXACT));

        register_variable(
                real("steering", &steering_)
                        .setCausality(causality_t::OUTPUT)
                        .setVariability(variability_t::CONTINUOUS)
                        .setInitial(initial_t::EXACT));

        register_variable(
                real("acc_filtered", &acc_filtered)
                        .setCausality(causality_t::OUTPUT)
                        .setVariability(variability_t::CONTINUOUS)
                        .setInitial(initial_t::EXACT));

        register_variable(
                real("v_des", &v_des)
                        .setCausality(causality_t::OUTPUT)
                        .setVariability(variability_t::CONTINUOUS)
                        .setInitial(initial_t::EXACT));

        Controller::reset();
    }

    bool do_step(double dt) override {
        if (std::abs(dt - DT) > 1e-5)
            return false;

        // Apply actuator attack if conditions are met
        if (currentTime() >= attack_time_)
                apply_attack();

        // Get actual acceleration (due to bug, it's -accY)
        double acc_actual = -accY_;

        // Apply moving average 
        accX_history_[accX_history_index_] = acc_actual;
        accX_history_index_ = (accX_history_index_ + 1) % N_SAMPLES;
        acc_filtered = std::accumulate(accX_history_.begin(), accX_history_.end(), 0.0) / N_SAMPLES;

        // Rate-limit the desired acceleration to smooth step inputs
        const double max_accel_rate = 3.0;  // m/s² per second
        double a_des_change = a_des - a_des_prev;
        double max_change = max_accel_rate * dt;
        
        if (a_des_change > max_change) {
            a_des_filtered = a_des_prev + max_change;
        } else if (a_des_change < -max_change) {
            a_des_filtered = a_des_prev - max_change;
        } else {
            a_des_filtered = a_des;
        }
        a_des_prev = a_des_filtered;

        // Numerical integration with filtered acceleration
        v_des += dt * a_des_filtered;

        // Velocity error with saturation
        double v_error = v_des - velX_;
        const double max_v_error = 5.0;  // Limit velocity error
        v_error = std::clamp(v_error, -max_v_error, max_v_error);

        // Acceleration error
        double a_error = a_des_filtered - acc_filtered;

        // Controller with separate tuning for velocity and acceleration
        const double kv = 0.61;  // Velocity gain (reduced from 0.9)
        const double ka = 0.12;  // Acceleration gain (reduced from 0.166)
        
        double u = kv * v_error + ka * a_error;

        // Add derivative term on velocity error for damping
        double v_error_derivative = (v_error - prev_v_error_) / dt;
        prev_v_error_ = v_error;
        const double kd_v = 0.15;  // Damping on velocity error
        u += kd_v * v_error_derivative;

        // Smooth rate limiting on control output
        const double max_u_rate = 1.5;  // Maximum control change per second
        double u_change = u - prev_u_;
        double max_u_change = max_u_rate * dt;
        
        if (u_change > max_u_change) {
            u = prev_u_ + max_u_change;
        } else if (u_change < -max_u_change) {
            u = prev_u_ - max_u_change;
        }
        prev_u_ = u;

        // Split into throttle and brake with deadzone
        const double deadzone = 0.033;
        if (std::abs(u) < deadzone)
        {
            throttle_ = 0.0;
            brake_ = 0.0;
        }
        else
        {
            throttle_ = std::clamp(u, 0.0, 1.0);
            brake_ = std::clamp(-0.666 * u, 0.0, 1.0);
        }
                
        // Keep steering control as is
        steering_ = -steerPID.calculate(std::numbers::pi / 2.0, yaw_);
        
        prev_velX_ = velX_;

        return true;
    }

    void apply_attack() {

        switch (attack_)
        {
        case 4:
                a_des = a_des * (1.0 + attack_amplitude_);
                break;
        case 5:
                a_des = a_des + attack_amplitude_;
                break;
        }
    }

    void reset() override {
        steerPID.reset();

        prev_velX_ = 0.0;
        accX_history_.fill(0.0);
        accX_history_index_ = 0;

        v_des = 0;
        a_des_prev = 0;
        a_des_filtered = 0;
        prev_v_error_ = 0;
        prev_u_ = 0;

        // Attack parameters keep their values (set by user)

        // Reset inputs
        a_des = 0.0;
        posX_ = 0.0;
        posY_ = 0.0;
        posZ_ = 0.0;
        velX_ = 0.0;
        velY_ = 0.0;
        velZ_ = 0.0;
        accX_ = 0.0;
        accY_ = 0.0;
        accZ_ = 0.0;
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        groundSpeed_ = 0.0;

        // Reset outputs
        throttle_ = 0.0;
        brake_ = 0.0;
        steering_ = 0.0;
        acc_filtered = 0.0;
    }

private:
    // stuff
    PID steerPID = PID(DT, 1, -1, 0.04, 0.07, 0.01);

    // State
    double prev_velX_ = 0;
    std::array<double, N_SAMPLES> accX_history_{};
    size_t accX_history_index_ = 0;

    double v_des = 0;
    double a_des_prev = 0;
    double a_des_filtered = 0;
    double prev_v_error_ = 0;
    double prev_u_ = 0;

    // Attack parameters
    double attack_time_ = 1e9;      // Default: attack never occurs
    int attack_ = 0.0;            // Default: no attack
    double attack_amplitude_ = 0.0;  // Default: zero amplitude

    // Input variables
    double a_des  = 0;
    double posX_  = 0;
    double posY_  = 0;
    double posZ_  = 0;
    double velX_  = 0;
    double velY_  = 0;
    double velZ_  = 0;
    double accX_  = 0;
    double accY_  = 0;
    double accZ_  = 0;
    double roll_  = 0;
    double pitch_ = 0;
    double yaw_   = 0;
    double groundSpeed_ = 0;

    // Output variables
    double throttle_ = 0; // [0, 1]
    double brake_    = 0; // [0, 1]
    double steering_ = 0; // [-1, 1]
    double acc_filtered = 0;
};

model_info fmu4cpp::get_model_info() {
    model_info info;
    info.modelName = "Controller";
    info.description = "Vehicle controller FMU";
    return info;
}

FMU4CPP_INSTANTIATE(Controller);
