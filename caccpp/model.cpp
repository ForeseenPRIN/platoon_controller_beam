//
// FMU4cpp Hello World example
//

#include "fmu4cpp/fmu_variable.hpp"
#include "fmu4cpp/status.hpp"
#include <cassert>
#include <cmath>
#include <fmu4cpp/fmu_base.hpp>


using namespace fmu4cpp;

class Model : public fmu_base {
public:
private:
    double target_distance;

    // @TODO make this stuff paramters from FMU (or not idc)
    static constexpr double C1 = 0.5;
    static constexpr double xi = 1;
    static constexpr double omega = 0.2;
    static constexpr double vehicle_len = 4;
    static constexpr double T = 0.01;

    // inputs
    double acc_leader;
    double acc_prec;
    double in_x;
    double in_x_leader;
    double in_x_prec;
    double speed_leader;
    double speed_me;
    double speed_prec;

    // output
    double accdes;
    double speeddes;

    double last_trigger = 0;
public:
    explicit Model(const fmu_data &data) : fmu_base(data) {
        // Paramters
        register_variable(real(
                                  "targetDistance", &target_distance)
                                  .setCausality(causality_t::PARAMETER)
                                  .setVariability(variability_t::FIXED));

        // input vars
        register_variable(real("acc_leader", &acc_leader)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("acc_prec", &acc_prec)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("in_x", &in_x)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("in_x_leader", &in_x_leader)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("in_x_prec", &in_x_prec)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("speed_leader", &speed_leader)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("speed_me", &speed_me)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        register_variable(real("speed_prec", &speed_prec)
                                  .setCausality(causality_t::INPUT)
                                  .setVariability(variability_t::DISCRETE));

        // output
        // Output variable
        register_variable(real("accdes", &accdes)
                                  .setCausality(causality_t::OUTPUT)
                                  .setVariability(variability_t::DISCRETE)
                                  .setDependencies({"acc_leader",
                                                    "acc_prec",
                                                    "in_x",
                                                    "in_x_leader",
                                                    "in_x_prec",
                                                    "speed_leader",
                                                    "speed_me",
                                                    "speed_prec"}));

        register_variable(real("speeddes", &speeddes)
                                  .setCausality(causality_t::OUTPUT)
                                  .setVariability(variability_t::DISCRETE)
                                  .setDependencies({"acc_leader",
                                                    "acc_prec",
                                                    "in_x",
                                                    "in_x_leader",
                                                    "in_x_prec",
                                                    "speed_leader",
                                                    "speed_me",
                                                    "speed_prec"}));

        // Init vars
        Model::reset();
    }

    bool do_step(double dt) override {
        if(currentTime() != 0 and last_trigger + T < currentTime())
            return true;

        accdes = C1 * acc_prec + (1 - C1) * acc_leader 
                - 0.3 * (speed_me - speed_prec)
                - 0.1 * (speed_me - speed_leader)
                - 0.04 * (target_distance - (in_x_prec - in_x - vehicle_len));
        
        if((in_x_prec - in_x - vehicle_len) >= 1.5*target_distance)
            accdes += 3.33;
        

        speeddes = 0; // @todo integral
        //speeddes = C1 * speed_prec + (1 - C1) * speed_leader 
        //        - 0.3 * (in_x - in_x_prec)
        //        - 0.1 * (in_x - in_x_leader)
        //        - 0.04 * (target_distance - (in_x_prec - in_x - vehicle_len));

        last_trigger = currentTime();
        return true;
    }

    void reset() override {
        last_trigger = 0;
        accdes = 0;
        target_distance = 11.0;
    }
};

model_info fmu4cpp::get_model_info() {
    return {
        .modelName = "caccpp",
        .description = "CACC alg for FORESEEN"
    };
}

FMU4CPP_INSTANTIATE(Model);// Entry point for FMI instantiate function.
