/**
 * validator.cc
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: March 7, 2020
 * Authors: Toki Migimatsu
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include "logic_opt/planning/validator.h"

namespace logic_opt {

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(logicopt, m) {

  // Articulated body
  py::class_<Validator>(m, "Validator")
      .def(py::init<const std::string&, const std::string&>(), "domain"_a, "problem"_a)
      .def_property_readonly("initial_state", &Validator::initial_state)
      .def("next_state", &Validator::NextState)
      .def("is_valid_action", &Validator::IsValidAction)
      .def("is_valid_tuple", &Validator::IsValidTuple)
      .def("is_goal_satisfied", &Validator::IsGoalSatisfied)
      .def("is_valid_plan", &Validator::IsValidPlan);

}

}  // namespace spatial_dyn
