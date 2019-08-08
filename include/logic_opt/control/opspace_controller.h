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

#include <csignal>  // std::sig_atomic_t

#include <spatial_dyn/spatial_dyn.h>

#include "logic_opt/world.h"

namespace logic_opt {

void ExecuteOpspaceController(spatial_dyn::ArticulatedBody& ab, const World3& world,
                              const std::shared_ptr<const std::map<std::string, Object3>>& world_objects,
                              const Eigen::MatrixXd& X_optimal, volatile std::sig_atomic_t& g_runloop);

}  // namespace logic_opt

#endif  // LOGIC_OPT_CONTROL_OPSPACE_CONTROLLER_H_
