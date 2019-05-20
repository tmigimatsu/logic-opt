/**
 * logic_opt_mex_function.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: April 25, 2019
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_MATLAB_LOGIC_OPT_MEX_FUNCTION_H_
#define LOGIC_OPT_MATLAB_LOGIC_OPT_MEX_FUNCTION_H_

#include <algorithm>   // std::copy
#include <functional>  // std::multiplies
#include <memory>      // std::shared_ptr
#include <map>         // std::map
#include <numeric>     // std::accumulate
#include <string>      // std::string
#include <utility>     // std::move
#include <vector>      // std::vector;

#include "mex.hpp"
#include "mexAdapter.hpp"

#include <logic_opt/world.h>

namespace logic_opt {

template<int Dim>
class LogicOptMexFunction : public matlab::mex::Function {

 public:

  LogicOptMexFunction(std::vector<size_t>&& dim_output,
                      const std::shared_ptr<const std::map<std::string, logic_opt::Object<Dim>>>& objects,
                      size_t T = 1)
      : dim_output_(std::move(dim_output)),
        world_(objects, T) {}

  virtual ~LogicOptMexFunction() = default;

  void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) override {
    CheckArguments(outputs, inputs);

    // Compute dimensions
    matlab::data::TypedArray<double> arr_X = inputs[0];
    const matlab::data::ArrayDimensions dim_input = arr_X.getDimensions();
    const size_t num_evaluations = std::accumulate(dim_input.begin() + 2, dim_input.end(),
                                                   1, std::multiplies<size_t>());

    matlab::data::ArrayDimensions dim_output(dim_output_.size() + dim_input.size() - 2);
    std::copy(dim_output_.begin(), dim_output_.end(), dim_output.begin());
    std::copy(dim_input.begin() + 2, dim_input.end(), dim_output.begin() + dim_output_.size());
    if (dim_output.empty()) {
      dim_output.push_back(1);
    }
    const size_t size_outputs = std::accumulate(dim_output_.begin(), dim_output_.end(),
                                                1, std::multiplies<size_t>());

    // Get data buffer (creates copy)
    const double* data = &*arr_X.begin();

    // Create output array
    matlab::data::TypedArray<double> output = factory_.createArray<double>(dim_output);
    double* data_output = &*output.begin();

    // Evaluate objective
    for (size_t i = 0; i < num_evaluations; i++) {
      const double* data_i = &data[(world_.kDof * world_.num_timesteps()) * i];
      const Eigen::Map<const Eigen::MatrixXd> X(data_i, world_.kDof, world_.num_timesteps());
      Eigen::Map<Eigen::VectorXd> O(data_output + size_outputs * i, size_outputs);
      Evaluate(X, O);
    }

    // Return
    outputs[0] = std::move(output);
  }

 protected:

  using Args = std::vector<matlab::data::Array>;

  void CheckArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
    if (inputs.size() < 1) {
      Error("Not enough input arguments");
    }

    if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
        inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
      Error("input must be a double array");
    }

    const matlab::data::TypedArray<double> arr_X = inputs[0];
    const matlab::data::ArrayDimensions dim = arr_X.getDimensions();
    if (dim.size() < 2) {
      Error("Input array must have at least two dimensions");
    }
    if (dim[0] != world_.kDof || dim[1] != world_.num_timesteps()) {
      Error("First two dimensions of input array must be [" + std::to_string(world_.kDof) + " x T]");
    }
  }

  void Print(const std::string& str) {
    matlab_->feval(u"fprintf", 0, Args({ factory_.createScalar(str) }));
  }

  void Println(const std::string& str) {
    matlab_->feval(u"fprintf", 0, Args({ factory_.createScalar(str + "\n") }));
  }

  void Error(const std::string& str) {
    matlab_->feval(u"error", 0, Args({ factory_.createScalar(str) }));
  }

  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> X,
                        Eigen::Ref<Eigen::VectorXd> output) = 0;

  // Pointer to MATLAB engine
  std::shared_ptr<matlab::engine::MATLABEngine> matlab_ = getEngine();

  // Factory to create MATLAB data arrays
  matlab::data::ArrayFactory factory_;

  logic_opt::World<Dim> world_;

  const std::vector<size_t> dim_output_;

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_MATLAB_LOGIC_OPT_MEX_FUNCTION_H_
