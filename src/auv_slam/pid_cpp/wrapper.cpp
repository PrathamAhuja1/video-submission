#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

#include "heave_pid_rt.h"

namespace py = pybind11;

PYBIND11_MODULE(heave_pid_rt_py, m)
{
    m.doc() = "Simulink-generated PID controller for AUV heave control";
    
    py::class_<heave_pid_rt>(m, "HeavePID")
        .def(py::init<>(), "Initialize the PID controller")

        // Initialize model
        .def("initialize", &heave_pid_rt::initialize,
             "Initialize the PID controller with default states")

        // Set model inputs
        .def(
            "set_inputs",
            [](heave_pid_rt &self,
               double depth_measured,
               double depth_goal)
            {
                self.rtU.depth_measured = depth_measured;
                self.rtU.depth_goal     = depth_goal;
            },
            py::arg("depth_measured"),
            py::arg("depth_goal"),
            "Set PID inputs: current depth and target depth (meters)"
        )

        // Step model (one control cycle)
        .def("step", &heave_pid_rt::step,
             "Execute one PID control step at the configured sample rate")

        // Get thruster outputs as list
        .def(
            "get_outputs",
            [](heave_pid_rt &self)
            {
                // Return as Python list [FL, FR, RL, RR]
                std::vector<double> outputs = {
                    self.rtY.FL,
                    self.rtY.FR,
                    self.rtY.RL,
                    self.rtY.RR
                };
                return outputs;
            },
            "Get thruster outputs: [FL, FR, RL, RR] in range -100 to +100"
        )
        
        // Alternative: Get outputs as dictionary
        .def(
            "get_outputs_dict",
            [](heave_pid_rt &self)
            {
                py::dict outputs;
                outputs["FL"] = self.rtY.FL;  // Front Left
                outputs["FR"] = self.rtY.FR;  // Front Right
                outputs["RL"] = self.rtY.RL;  // Rear Left
                outputs["RR"] = self.rtY.RR;  // Rear Right
                return outputs;
            },
            "Get thruster outputs as dictionary with keys: FL, FR, RL, RR"
        )

        // Get simulation time
        .def(
            "get_time",
            [](heave_pid_rt &self)
            {
                return self.getRTM()->Timing.t[0];
            },
            "Get current simulation time (seconds)"
        )

        // Error handling
        .def(
            "has_error",
            [](heave_pid_rt &self)
            {
                return self.getRTM()->getErrorStatus() != nullptr;
            },
            "Check if the PID controller has encountered an error"
        )

        .def(
            "get_error_status",
            [](heave_pid_rt &self)
            {
                const char *err = self.getRTM()->getErrorStatus();
                return err ? std::string(err) : std::string();
            },
            "Get error status message (empty string if no error)"
        )
        
        // Get individual outputs (for debugging)
        .def(
            "get_output_FL",
            [](heave_pid_rt &self) { return self.rtY.FL; },
            "Get Front-Left thruster output"
        )
        
        .def(
            "get_output_FR",
            [](heave_pid_rt &self) { return self.rtY.FR; },
            "Get Front-Right thruster output"
        )
        
        .def(
            "get_output_RL",
            [](heave_pid_rt &self) { return self.rtY.RL; },
            "Get Rear-Left thruster output"
        )
        
        .def(
            "get_output_RR",
            [](heave_pid_rt &self) { return self.rtY.RR; },
            "Get Rear-Right thruster output"
        )
        
        // Get current inputs (for verification)
        .def(
            "get_current_depth",
            [](heave_pid_rt &self) { return self.rtU.depth_measured; },
            "Get currently set measured depth"
        )
        
        .def(
            "get_target_depth",
            [](heave_pid_rt &self) { return self.rtU.depth_goal; },
            "Get currently set target depth"
        );
    
    // Module-level info
    m.attr("__version__") = "1.0.0";
    m.attr("sample_time") = 0.2;  // 200ms sample time from Simulink
    m.attr("control_rate") = 5.0;  // 5 Hz
}