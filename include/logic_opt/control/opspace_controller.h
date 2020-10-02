/**
 * opspace_controller.h
 *
 * Copyright 2019. All Rights Reserved.
 *
 * Created: June 4, 2019
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_CONTROL_OPSPACE_CONTROLLER_H_
#define LOGIC_OPT_CONTROL_OPSPACE_CONTROLLER_H_

#include <spatial_dyn/spatial_dyn.h>
#include <spatial_opt/spatial_opt.h>

#include <chrono>              // std::chrono
#include <condition_variable>  // std::condition_variable
#include <csignal>             // std::sig_atomic_t
#include <functional>          // std::function
#include <map>                 // std::map
#include <memory>              // std::shared_ptr
#include <mutex>               // std::mutex, std::lock_guard, std::unique_lock
#include <optional>            // std::optional

#include "logic_opt/world.h"

namespace logic_opt {

class PlannerControllerInterface {
 public:
  struct OptimizationResult {
    Eigen::MatrixXd X_optimal;
    World world;
    size_t t_action;
  };

  struct ExecutionUpdate {
    std::map<std::string, Object> world_poses;
    ctrl_utils::Tree<std::string, Frame> tree;
    size_t t_action;
  };

  PlannerControllerInterface(volatile std::sig_atomic_t& g_runloop)
      : g_runloop(g_runloop) {}

  void SetOptimizationResult(Eigen::MatrixXd&& X_optimal, World&& world,
                             size_t t_action) {
    std::lock_guard<std::mutex> lock(mtx_optimization_result_);
    optimization_result_ = {std::move(X_optimal), std::move(world),
                            std::move(t_action)};
    is_optimization_result_set_ = true;
    cv_optimization_result_.notify_one();
  }

  std::optional<OptimizationResult> TryGetOptimizationResult() {
    std::lock_guard<std::mutex> lock(mtx_optimization_result_);
    if (!is_optimization_result_set_) return {};

    is_optimization_result_set_ = false;
    return {std::move(optimization_result_)};
  }

  bool WaitForOptimizationResult() const {
    using namespace std::chrono_literals;

    // Wait for result
    std::unique_lock<std::mutex> lock(mtx_optimization_result_);
    while (g_runloop && !is_optimization_result_set_) {
      const auto now = std::chrono::system_clock::now();
      cv_optimization_result_.wait_until(lock, now + 1ms);
    }
    return g_runloop;
  }

  /**
   * Update step for controller -> trajectory optimizer.
   *
   * @param world_poses Map of world object poses according to the kinematic
   * tree.
   * @param tree Kinematic tree for world object poses.
   * @param t_action Current timestep in the original plan.
   */
  void SetExecutionUpdate(const std::map<std::string, Object>& world_poses,
                          const ctrl_utils::Tree<std::string, Frame>& tree,
                          size_t t_action) {
    std::lock_guard<std::mutex> lock(mtx_execution_update_);
    execution_update_ = {world_poses, tree, t_action};
    is_execution_update_set_ = true;
    cv_execution_update_.notify_one();
  }

  ExecutionUpdate GetExecutionUpdate() {
    using namespace std::chrono_literals;

    // Wait for update
    std::unique_lock<std::mutex> lock(mtx_execution_update_);
    while (g_runloop && !is_execution_update_set_) {
      const auto now = std::chrono::system_clock::now();
      cv_execution_update_.wait_until(lock, now + 1ms);
    }

    if (!g_runloop) {
      throw std::runtime_error(
          "PlannerControllerInterface::GetExecutionUpdate(): g_runloop set to "
          "false.");
    }
    is_execution_update_set_ = false;
    return std::move(execution_update_);
  }

  volatile std::sig_atomic_t& g_runloop;

 private:
  mutable std::mutex mtx_optimization_result_;
  OptimizationResult optimization_result_;
  mutable std::condition_variable cv_optimization_result_;
  bool is_optimization_result_set_ = false;

  mutable std::mutex mtx_execution_update_;
  ExecutionUpdate execution_update_;
  mutable std::condition_variable cv_execution_update_;
  bool is_execution_update_set_ = false;
};

void OpspaceController(
    const std::map<std::string, Object>& world_poses_0,
    const spatial_dyn::ArticulatedBody& ab_0,
    std::shared_ptr<PlannerControllerInterface> shared_memory);

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONTROL_OPSPACE_CONTROLLER_H_
