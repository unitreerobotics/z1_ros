#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "unitree_arm_sdk/unitree_arm.h"

using namespace UNITREE_ARM_SDK;

namespace py = pybind11;
PYBIND11_MODULE(unitree_arm_interface, m) {
  py::enum_<Error>(m, "Error")
    .value("joint_position_limits_violation", Error::joint_position_limits_violation)
    .value("joint_velocity_violation", Error::joint_velocity_violation)
    .value("colision_detected", Error::colision_detected)
    .value("repeat_file_invalid", Error::repeat_file_invalid)
    .export_values()
    ;

  py::enum_<ArmMode>(m, "ArmMode")
    .value("Invalid", ArmMode::Invalid)
    .value("Passive", ArmMode::Passive)
    .value("LowCmd", ArmMode::LowCmd)
    .value("JointSpeedCtrl", ArmMode::JointSpeedCtrl)
    .value("JointPositionCtrl", ArmMode::JointPositionCtrl)
    .value("Teach", ArmMode::Teach)
    .value("TeachRepeat", ArmMode::TeachRepeat)
    .value("Calibration", ArmMode::Calibration)
    .value("ClearError", ArmMode::ClearError)
    .export_values()
    ;

  py::class_<GripperState>(m, "GripperState")
    .def(py::init<>())
    .def_readwrite("angle", &GripperState::angle)
    .def_readwrite("speed", &GripperState::speed)
    .def_readwrite("tau", &GripperState::tau)
    .def_readwrite("reached_goal", &GripperState::reached_goal)
    .def_readwrite("stalled", &GripperState::stalled)
    .def_readwrite("exist", &GripperState::exist)
    ;

  py::class_<GripperCmd>(m, "GripperCmd")
    .def(py::init<>())
    .def_readwrite("angle", &GripperCmd::angle)
    .def_readwrite("speed", &GripperCmd::speed)
    .def_readwrite("maxTau", &GripperCmd::maxTau)
    .def_readwrite("epsilon_inner", &GripperCmd::epsilon_inner)
    .def_readwrite("epsilon_outer", &GripperCmd::epsilon_outer)
    ;

  py::class_<ArmCmd>(m, "ArmCmd")
    .def(py::init<>())
    .def_readwrite("mode", &ArmCmd::mode)
    .def_readwrite("mass_ee", &ArmCmd::mass_ee)
    .def_readwrite("com_ee", &ArmCmd::com_ee)
    .def_readwrite("inertia_ee", &ArmCmd::inertia_ee)
    .def_readwrite("F_T_EE", &ArmCmd::F_T_EE)
    .def_readwrite("mass_load", &ArmCmd::mass_load)
    .def_readwrite("com_load", &ArmCmd::com_load)
    .def_readwrite("inertia_load", &ArmCmd::inertia_load)
    .def_readwrite("gripperCmd", &ArmCmd::gripperCmd)
    .def_readwrite("Kp", &ArmCmd::Kp)
    .def_readwrite("Kd", &ArmCmd::Kd)
    .def_readwrite("q_d", &ArmCmd::q_d)
    .def_readwrite("dq_d", &ArmCmd::dq_d)
    .def_readwrite("tau_d", &ArmCmd::tau_d)
    .def_readwrite("label", &ArmCmd::label)
    .def("setLabel", [](ArmCmd& cmd, std::string str){
      size_t maxSize = std::min(str.size(), cmd.label.size()-1);
      std::copy(str.begin(), str.begin()+maxSize, cmd.label.begin());
      cmd.label[maxSize] = '\0';
    })
    .def("setF_T_EE", [](ArmCmd& cmd, const Mat4& T) { cmd.setF_T_EE(T); })
    .def("set_inertia_ee", [](ArmCmd& cmd, const Mat3& I) { cmd.set_inertia_ee(I); })
    ;

  py::class_<ArmState>(m, "ArmState")
    .def(py::init<>())
    .def_readwrite("mode", &ArmState::mode)
    .def_readwrite("gripperState", &ArmState::gripperState)
    .def_readwrite("gripperCmd", &ArmState::gripperCmd)
    .def_readwrite("q", &ArmState::q)
    .def_readwrite("q_d", &ArmState::q_d)
    .def_readwrite("dq", &ArmState::dq)
    .def_readwrite("dq_d", &ArmState::dq_d)
    .def_readwrite("tau", &ArmState::tau)
    .def_readwrite("tau_d", &ArmState::tau_d)
    .def_readwrite("motor_temperature", &ArmState::motor_temperature)
    .def_readwrite("F_T_EE", &ArmState::F_T_EE)
    .def_readwrite("errors", &ArmState::errors)
    .def_readwrite("motor_errors", &ArmState::motor_errors)
    .def("getF_T_EE", &ArmState::getF_T_EE)
    .def("hasError", &ArmState::hasError)
    .def("hasMotorError", &ArmState::hasMotorError)
    ;

  py::class_<UnitreeArm>(m, "UnitreeArm")
    .def(py::init<std::string, uint>())
    .def("init", &UnitreeArm::init)
    .def("sendRecv", &UnitreeArm::sendRecv)
    .def_readwrite("armCmd", &UnitreeArm::armCmd)
    .def_readwrite("armState", &UnitreeArm::armState)
    .def_property_readonly("dt", [](const UnitreeArm& self) { return self.dt; })
    ;
}