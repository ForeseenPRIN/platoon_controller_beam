
#include <fmu4cpp/fmu_base.hpp>
#include <cmath>

using namespace fmu4cpp;


class Driver : public fmu_base {
private:
    // Output
    double u;

    // Parameters for fun1 (sine wave)
    double fun1_amplitude;
    double fun1_offset;
    double fun1_frequency;  // rad/s
    double fun1_phase;      // rad

    // Parameters for fun2 (ramp)
    double fun2_slope;

    // Parameters for final_steps (sine wave)
    double final_steps_amplitude;
    double final_steps_bias;
    double final_steps_frequency;  // rad/s
    double final_steps_phase;      // rad

    // Control parameters
    int operational_mode;
    double sprint_period;
    double acceleration_value;
    double deceleration_value;

    // Internal state
    double clock;

public:
    static constexpr double DT = 5e-3;

    FMU4CPP_CTOR(Driver) {

        // Output variable
        register_variable(
                real("u", &u)
                        .setCausality(causality_t::OUTPUT)
                        .setVariability(variability_t::CONTINUOUS)
                        .setInitial(initial_t::EXACT));

        // Parameters for fun1 (sine wave)
        register_variable(
                real("fun1_amplitude", &fun1_amplitude)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("fun1_offset", &fun1_offset)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("fun1_frequency", &fun1_frequency)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("fun1_phase", &fun1_phase)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        // Parameters for fun2 (ramp)
        register_variable(
                real("fun2_slope", &fun2_slope)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        // Parameters for final_steps (sine wave)
        register_variable(
                real("final_steps_amplitude", &final_steps_amplitude)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("final_steps_bias", &final_steps_bias)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("final_steps_frequency", &final_steps_frequency)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("final_steps_phase", &final_steps_phase)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        // Control parameters
        register_variable(
                integer("operational_mode", &operational_mode)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("sprint_period", &sprint_period)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("acceleration_value", &acceleration_value)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        register_variable(
                real("deceleration_value", &deceleration_value)
                        .setCausality(causality_t::PARAMETER)
                        .setVariability(variability_t::FIXED));

        Driver::reset();
    }

    bool do_step(double dt) override {
        // Compute function generators
        double fun1 = fun1_amplitude * std::sin(fun1_frequency * clock + fun1_phase) + fun1_offset;
        double fun2 = fun2_slope * clock;  // Ramp starting at t=0 with initial output 0
        double final_steps = final_steps_amplitude * std::sin(final_steps_frequency * clock + final_steps_phase) + final_steps_bias;

        // Compute acc_des (u) based on operational mode
        if (operational_mode == 0) {
            if (clock < sprint_period) {
                u = acceleration_value;
            } else {
                u = fun1;
            }
        } else if (operational_mode == 1) {
            u = fun2;
            if (clock > 8) {
                u = final_steps;
            }
        } else {
            double helper = 60 + sprint_period * 2;
            if (clock < sprint_period) {
                u = acceleration_value;
            } else if (clock < 30) {
                u = final_steps;
            } else if (clock < 30 + sprint_period) {
                u = deceleration_value;
            } else if (clock < 60 + sprint_period * 2) {
                u = final_steps;
            } else {
                double mod_time = std::fmod(clock, helper);
                if (mod_time < sprint_period) {
                    u = acceleration_value;
                } else if (mod_time < 30) {
                    u = final_steps;
                } else if (mod_time < 30 + sprint_period) {
                    u = deceleration_value;
                } else {
                    u = final_steps;
                }
            }
        }

        // Update clock
        clock += dt;

        return true;
    }

    void reset() override {
        u = 0.0;
        clock = 0.0;

        // Default parameters for fun1
        fun1_amplitude = 1.0;
        fun1_offset = 0.0;
        fun1_frequency = 1.0;  // rad/s
        fun1_phase = 0.0;      // rad

        // Default parameters for fun2
        fun2_slope = 1.0;

        // Default parameters for final_steps (from MATLAB code)
        final_steps_amplitude = 0.03;
        final_steps_bias = 0.005;
        final_steps_frequency = 1.5;  // rad/s
        final_steps_phase = 0.0;      // rad

        // Default control parameters
        operational_mode = 0;
        sprint_period = 5.0;
        acceleration_value = 1.0;
        deceleration_value = -1.0;
    }
};

model_info fmu4cpp::get_model_info() {
    model_info info;
    info.modelName = "Driver";
    info.description = "The driver of the leader car";
    return info;
}

FMU4CPP_INSTANTIATE(Driver);
