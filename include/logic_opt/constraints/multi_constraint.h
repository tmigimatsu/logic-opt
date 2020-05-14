/**
 * multi_constraint.h
 *
 * Copyright 2018. All Rights Reserved.
 *
 * Created: November 18, 2018
 * Authors: Toki Migimatsu
 */

#ifndef LOGIC_OPT_MULTI_CONSTRAINT_H_
#define LOGIC_OPT_MULTI_CONSTRAINT_H_

#include <memory>  // std::unique_ptr
#include <vector>  // std::vector

#include "logic_opt/constraints/constraint.h"

namespace logic_opt {

/**
 * Concatenation of multiple constraints.
 */
class MultiConstraint : public Constraint {

 public:

  MultiConstraint(std::vector<std::unique_ptr<Constraint>>&& constraints,
                  const std::string& name_constraint);

  virtual ~MultiConstraint() = default;

  // Optimization methods
  virtual void Evaluate(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> constraints) override;

  virtual void Jacobian(Eigen::Ref<const Eigen::MatrixXd> Q,
                        Eigen::Ref<Eigen::VectorXd> Jacobian) override;

  virtual void JacobianIndices(Eigen::Ref<Eigen::ArrayXi> idx_i,
                               Eigen::Ref<Eigen::ArrayXi> idx_j) override;

  virtual void Hessian(Eigen::Ref<const Eigen::MatrixXd> Q,
                       Eigen::Ref<const Eigen::VectorXd> lambda,
                       Eigen::Ref<Eigen::SparseMatrix<double>> Hessian) override;

  virtual void HessianStructure(Eigen::SparseMatrix<bool>& Hessian) override;

  // Constraint properties
  virtual Type constraint_type(size_t idx_constraint) const override;

  virtual void OpenConstraintLog(const std::string& filepath) override;
  virtual void OpenJacobianLog(const std::string& filepath) override;

  virtual void CloseConstraintLog() override;
  virtual void CloseJacobianLog() override;

  // Iterator
  class const_iterator {

   public:

    using iterator_category = std::random_access_iterator_tag;
    using value_type = std::unique_ptr<Constraint>;
    using difference_type = ptrdiff_t;
    using pointer = const value_type*;
    using reference = const value_type&;

    const_iterator(const std::unique_ptr<Constraint>* ptr) : ptr_(ptr) {}

    const_iterator& operator++() { ++ptr_; return *this; }
    const_iterator& operator+=(size_t n) { ptr_ += n; return *this; }
    const_iterator operator+(size_t n) const { return const_iterator(ptr_ + n); }
    const_iterator& operator--() { --ptr_; return *this; }
    const_iterator& operator-=(size_t n) { ptr_ -= n; return *this; }
    const_iterator operator-(size_t n) const { return const_iterator(ptr_ - n); }
    bool operator==(const const_iterator& other) const { return ptr_ == other.ptr_; }
    bool operator!=(const const_iterator& other) const { return ptr_ != other.ptr_; }
    reference operator*() const { return *ptr_; }
    pointer operator->() const { return ptr_; }

   private:

    const std::unique_ptr<Constraint>* ptr_;

  };

  const_iterator begin() const { return const_iterator(constraints_.data()); }
  const_iterator end() const { return const_iterator(constraints_.data() + constraints_.size()); }

 protected:

  static size_t NumConstraints(const std::vector<std::unique_ptr<Constraint>>& constraints);

  static size_t LenJacobian(const std::vector<std::unique_ptr<Constraint>>& constraints);

  static size_t TStart(const std::vector<std::unique_ptr<Constraint>>& constraints);

  static size_t NumTimesteps(const std::vector<std::unique_ptr<Constraint>>& constraints);

  std::vector<std::unique_ptr<Constraint>> constraints_;  // Vector of constraints

};

}  // namespace logic_opt

#endif  // LOGIC_OPT_MULTI_CONSTRAINT_H_
