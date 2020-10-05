/**
 * jacobian_tests.cc
 *
 * Copyright 2020. All Rights Reserved.
 *
 * Created: June 9, 2020
 * Authors: Marion Lepert
 */

#include <ctrl_utils/eigen.h>
#include <logic_opt/objectives/angular_distance_objective.h>

#include <catch2/catch.hpp>
#include <cstdlib>   // std::srand
#include <iostream>  // std::std::cout

using std::cout;
using std::endl;

using ::logic_opt::AngularDistanceObjective;

Eigen::Vector3d get_random_unit_vector() {
  Eigen::Vector3d random_vec = Eigen::Vector3d::Random();
  random_vec.normalize();
  return random_vec;
}

Eigen::Quaterniond get_random_quaternion() {
  Eigen::Vector4d coeffs = Eigen::Vector4d::Random();
  coeffs.normalize();
  Eigen::Map<Eigen::Quaterniond> q((double*)coeffs.data());
  return q;
}

/*********************************** POSITION JACOBIAN
 * **************************************/

/* Eq 21: Jacobian matrix of a vector f, corresponding to the rotation of a
   vector x by quaternion q, with respect to the quaternion coordinates */
Eigen::Matrix<double, 3, 4> get_df_dq(const Eigen::Quaterniond& q,
                                      const Eigen::Vector3d& x) {
  Eigen::Vector3d df_dwq = 2 * q.w() * x + 2 * q.vec().cross(x);

  Eigen::Matrix3d df_dvq =
      -2 * q.w() * ctrl_utils::CrossMatrix(x) - 2 * x * q.vec().transpose() +
      2 * q.vec().transpose().dot(x) * Eigen::Matrix3d::Identity() +
      2 * q.vec() * x.transpose();

  Eigen::Matrix<double, 3, 4> df_dq;
  df_dq.block<3, 1>(0, 0) = df_dwq;
  df_dq.block<3, 3>(0, 1) = df_dvq;

  return df_dq;
}

/* Returns the rotation of a vector x by a quaternion q */
Eigen::Vector3d get_f(const Eigen::Quaterniond& q, const Eigen::Vector3d& x) {
  Eigen::Quaterniond xQuat;
  xQuat.w() = 0;
  xQuat.vec() = x;
  Eigen::Quaterniond f = q * xQuat * q.conjugate();
  return f.vec();
}

/* Returns the 3x1 vector that corresponds to the numerical derivative
   of the unit vector x with respect to the specified quaternion parameter */
Eigen::Vector3d get_df_dq_numerical(const Eigen::Vector3d& x,
                                    const Eigen::Quaterniond& q,
                                    const char param) {
  double eps = 1e-6;

  Eigen::Quaterniond qd = q;

  switch (param) {
    case 'w':
      qd.w() += eps;
      break;
    case 'x':
      qd.x() += eps;
      break;
    case 'y':
      qd.y() += eps;
      break;
    case 'z':
      qd.z() += eps;
      break;
  }

  Eigen::Vector3d f = get_f(q, x);
  Eigen::Vector3d fd = get_f(qd, x);

  return (fd - f) / eps;
}

bool test_position_jacobian() {
  // Get random quaternion and unit vector
  Eigen::Quaterniond q = get_random_quaternion();
  Eigen::Vector3d x = get_random_unit_vector();  // Vector to rotate

  // Get analytical gradient
  Eigen::Matrix<double, 3, 4> df_dq_analytical = get_df_dq(q, x);

  // Get numerical gradient
  Eigen::Matrix<double, 3, 4> df_dq_numerical;
  df_dq_numerical.col(0) = get_df_dq_numerical(x, q, 'w');
  df_dq_numerical.col(1) = get_df_dq_numerical(x, q, 'x');
  df_dq_numerical.col(2) = get_df_dq_numerical(x, q, 'y');
  df_dq_numerical.col(3) = get_df_dq_numerical(x, q, 'z');

  double norm_diff = (df_dq_numerical - df_dq_analytical).norm();
  bool is_correct = norm_diff < 1e-5;

  if (!is_correct) {
    cout << "---------- TEST POSITION JACOBIAN -----------" << endl;
    cout << "Analytical gradient: " << endl << df_dq_analytical << endl << endl;
    cout << "Numerical gradient: " << endl << df_dq_numerical << endl << endl;
    cout << "Difference: " << norm_diff << endl;
    cout << "----------------------------------------------" << endl;
  }

  return is_correct;
}

/******************************** ORIENTATION JACOBIAN
 * **************************************/

double get_dbqc_dwq(const Eigen::Quaterniond& q, const Eigen::Quaterniond& b,
                    const Eigen::Quaterniond& c) {
  double dbqc_dwq = b.w() * c.w() - b.vec().transpose().dot(c.vec());
  return dbqc_dwq;
}

Eigen::Vector3d get_dbqc_dvq(const Eigen::Quaterniond& q,
                             const Eigen::Quaterniond& b,
                             const Eigen::Quaterniond& c) {
  Eigen::Vector3d dbqc_dvq =
      -b.w() * c.vec().transpose() - c.w() * b.vec().transpose() +
      b.vec().transpose() * ctrl_utils::CrossMatrix(c.vec());
  return dbqc_dvq;
}

double get_daqinvb_dwq(const Eigen::Quaterniond& q, const Eigen::Quaterniond& a,
                       const Eigen::Quaterniond& b) {
  double daqinvb_dwq = a.w() * b.w() - a.vec().transpose().dot(b.vec());
  return daqinvb_dwq;
}

Eigen::Vector3d get_daqinvb_dvq(const Eigen::Quaterniond& q,
                                const Eigen::Quaterniond& a,
                                const Eigen::Quaterniond& b) {
  Eigen::Vector3d daqinvb_dvq =
      a.w() * b.vec().transpose() + b.w() * a.vec().transpose() -
      a.vec().transpose() * ctrl_utils::CrossMatrix(b.vec());
  return daqinvb_dvq;
}

double get_daqinvbqc_dwq(const Eigen::Quaterniond& q,
                         const Eigen::Quaterniond& a,
                         const Eigen::Quaterniond& b,
                         const Eigen::Quaterniond& c) {
  double daqinvbqc_dwq =
      2 * a.w() * b.w() * c.w() * q.w()

      - 2 * a.w() * q.w() * b.vec().transpose().dot(c.vec()) -
      2 * b.w() * q.w() * a.vec().transpose().dot(c.vec()) -
      2 * c.w() * q.w() * a.vec().transpose().dot(b.vec())

      - 2 * q.w() * a.vec().transpose().dot(b.vec().cross(c.vec())) +
      2 * a.w() * b.vec().transpose().dot(c.vec().cross(q.vec())) -
      2 * c.w() * a.vec().transpose().dot(b.vec().cross(q.vec()))

      +
      2 * a.vec().transpose().dot(b.vec()) * c.vec().transpose().dot(q.vec()) -
      2 * a.vec().transpose().dot(q.vec()) * b.vec().transpose().dot(c.vec());

  return daqinvbqc_dwq;
}

Eigen::Vector3d get_daqinvbqc_dvq(const Eigen::Quaterniond& q,
                                  const Eigen::Quaterniond& a,
                                  const Eigen::Quaterniond& b,
                                  const Eigen::Quaterniond& c) {
  Eigen::Vector3d daqinvbqc_dvq =
      2 * a.w() * b.w() * c.w() * q.vec().transpose() +
      2 * a.w() * q.w() * b.vec().transpose() *
          ctrl_utils::CrossMatrix(c.vec()) -
      2 * c.w() * q.w() * a.vec().transpose() *
          ctrl_utils::CrossMatrix(b.vec()) +
      2 * a.w() * b.vec().transpose().dot(c.vec()) * q.vec().transpose() -
      2 * b.w() * a.vec().transpose().dot(c.vec()) * q.vec().transpose() +
      2 * c.w() * a.vec().transpose().dot(b.vec()) * q.vec().transpose() -
      2 * a.w() * b.vec().transpose().dot(q.vec()) * c.vec().transpose() -
      2 * a.w() * c.vec().transpose().dot(q.vec()) * b.vec().transpose() -
      2 * c.w() * a.vec().transpose().dot(q.vec()) * b.vec().transpose() -
      2 * c.w() * b.vec().transpose().dot(q.vec()) * a.vec().transpose() +
      2 * q.w() * a.vec().transpose().dot(b.vec()) * c.vec().transpose() -
      2 * q.w() * b.vec().transpose().dot(c.vec()) * a.vec().transpose() +
      2 * a.vec().transpose() * (b.vec().cross(c.vec())) * q.vec().transpose() +
      2 * b.vec().transpose() * q.vec() * a.vec().transpose() *
          ctrl_utils::CrossMatrix(c.vec()) +
      2 * a.vec().transpose() * (c.vec().cross(q.vec())) * b.vec().transpose();
  return daqinvbqc_dvq;
}

/* Define enum to switch btw kinematic tree configurations for orientation
 * jacobian */
enum tree { bqc = 0, aqb = 1, aqbqc = 2 };

/* Given quaternions q, a, b, c, and a specified derivative (for the orientation
   jacobian), returns the double that corresponds to the numerical derivative of
   the w parameter of the desired angle, with respect to the specified
   quaternion q parameter */
double get_dw_dq_numerical(const Eigen::Quaterniond& q,
                           const Eigen::Quaterniond& a,
                           const Eigen::Quaterniond& b,
                           const Eigen::Quaterniond& c, const char param,
                           int kinematic_tree) {
  double eps = 1e-6;

  Eigen::Quaterniond qd = q;

  switch (param) {
    case 'w':
      qd.w() += eps;
      break;
    case 'x':
      qd.x() += eps;
      break;
    case 'y':
      qd.y() += eps;
      break;
    case 'z':
      qd.z() += eps;
      break;
  }

  Eigen::Quaterniond quatW;
  Eigen::Quaterniond quatWd;

  switch (kinematic_tree) {
    case bqc:
      quatW = b * q * c;
      quatWd = b * qd * c;
      break;
    case aqb:
      quatW = a * q.conjugate() * b;
      quatWd = a * qd.conjugate() * b;
      break;
    case aqbqc:
      quatW = a * q.conjugate() * b * q * c;
      quatWd = a * qd.conjugate() * b * qd * c;
      break;
  }

  return (quatWd.w() - quatW.w()) / eps;
}

/* Given quaternions q, a, b, c, returns a 4x1 vector corresponding
   to the desired analytical derivative (for the orientation jacobian) */
Eigen::Vector4d get_dw_dq_analytical(const Eigen::Quaterniond& q,
                                     const Eigen::Quaterniond& a,
                                     const Eigen::Quaterniond& b,
                                     const Eigen::Quaterniond& c,
                                     int kinematic_tree) {
  Eigen::Vector4d dw_dq;

  switch (kinematic_tree) {
    case bqc:
      dw_dq[3] = get_dbqc_dwq(q, b, c);
      dw_dq.head(3) = get_dbqc_dvq(q, b, c);
      break;
    case aqb:
      dw_dq[3] = get_daqinvb_dwq(q, a, b);
      dw_dq.head(3) = get_daqinvb_dvq(q, a, b);
      break;
    case aqbqc:
      dw_dq[3] = get_daqinvbqc_dwq(q, a, b, c);
      dw_dq.head(3) = get_daqinvbqc_dvq(q, a, b, c);
      break;
  }

  return dw_dq;
}

/* Given quaternions q, a, b, c, and the desired derivative to compute,
   returns true if the analytical and numerical derivatives match */
bool compare_dw_dq_num_analytical_gradient(const Eigen::Quaterniond& q,
                                           const Eigen::Quaterniond& a,
                                           const Eigen::Quaterniond& b,
                                           const Eigen::Quaterniond& c,
                                           int kinematic_tree) {
  Eigen::Vector4d dw_dq_analytical =
      get_dw_dq_analytical(q, a, b, c, kinematic_tree);

  Eigen::Vector4d dw_dq_numerical;
  dw_dq_numerical[3] = get_dw_dq_numerical(q, a, b, c, 'w', kinematic_tree);
  dw_dq_numerical[0] = get_dw_dq_numerical(q, a, b, c, 'x', kinematic_tree);
  dw_dq_numerical[1] = get_dw_dq_numerical(q, a, b, c, 'y', kinematic_tree);
  dw_dq_numerical[2] = get_dw_dq_numerical(q, a, b, c, 'z', kinematic_tree);

  double norm_diff = (dw_dq_numerical - dw_dq_analytical).norm();
  bool is_correct = norm_diff < 5e-5;

  if (!is_correct) {
    std::string tree_name;
    switch (kinematic_tree) {
      case bqc:
        tree_name = "bqc";
        break;
      case aqb:
        tree_name = "aq_invb";
        break;
      case aqbqc:
        tree_name = "aq_invbqc";
        break;
    }
    cout << "------------- TEST ORIENTATION JACOBIAN: " << tree_name
         << " ---------------" << endl;
    cout << "Analytical gradient: " << dw_dq_analytical.transpose() << endl
         << endl;
    cout << "Numerical gradient:  " << dw_dq_numerical.transpose() << endl
         << endl;
    cout << "Difference: " << norm_diff << endl;
    cout << "-------------------------------------------------------------"
         << endl
         << endl;
  }

  return is_correct;
}

/* Gets random q, a, b, and c quaternions and returns true if all three
   numerical and analytical derivatives are the same */
bool test_orientation_jacobian() {
  Eigen::Quaterniond q = get_random_quaternion();
  Eigen::Quaterniond a = get_random_quaternion();
  Eigen::Quaterniond b = get_random_quaternion();
  Eigen::Quaterniond c = get_random_quaternion();

  bool bqc_match = compare_dw_dq_num_analytical_gradient(q, a, b, c, bqc);
  bool aqb_match = compare_dw_dq_num_analytical_gradient(q, a, b, c, aqb);
  bool aqbqc_match = compare_dw_dq_num_analytical_gradient(q, a, b, c, aqbqc);
  bool is_correct = bqc_match & aqb_match & aqbqc_match;

  if (!is_correct) {
    cout << "-------------------------------------------------------------"
         << endl;
    cout << "-------------------------------------------------------------"
         << endl;
  }

  return is_correct;
}

/***************************************************************************************/

Eigen::Matrix4d ComputeNumericalJacobian(
    const Eigen::Quaterniond& q,
    const std::function<Eigen::Quaterniond(const Eigen::Quaterniond&)>& f,
    double h = 1e-3) {
  Eigen::Matrix4d dq_dq;
  Eigen::Quaterniond q_h = q;
  for (size_t i = 0; i < 4; i++) {
    double& q_i = q_h.coeffs()(i);
    const double q_i_0 = q_i;
    q_i = q_i_0 + h;
    const Eigen::Quaterniond F_hp = f(q_h);
    q_i = q_i_0 - h;
    const Eigen::Quaterniond F_hn = f(q_h);
    q_i = q_i_0;

    dq_dq.col(i) = (F_hp.coeffs() - F_hn.coeffs()) / (2. * h);
  }
  return dq_dq;
}

Eigen::MatrixXd ComputeNumericalJacobian(
    const Eigen::VectorXd& x,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
    double h = 1e-3) {
  Eigen::MatrixXd df_dx(f(x).size(), x.size());
  Eigen::VectorXd x_h = x;
  for (size_t i = 0; i < x.size(); i++) {
    double& x_i = x_h(i);
    const double x_i_0 = x_i;
    x_i = x_i_0 + h;
    const Eigen::VectorXd f_hp = f(x_h);
    x_i = x_i_0 - h;
    const Eigen::VectorXd f_hn = f(x_h);
    x_i = x_i_0;

    df_dx.col(i) = (f_hp - f_hn) / (2. * h);
  }
  return df_dx;
}

template <typename Derived1, typename Derived2>
void PrintMatrices(const std::string& name,
                   const Eigen::MatrixBase<Derived1>& analytical,
                   const Eigen::MatrixBase<Derived2>& numerical) {
  std::cout << name << std::endl
            << std::endl
            << "Analytical:" << std::endl
            << analytical << std::endl
            << std::endl
            << "Numerical:" << std::endl
            << numerical << std::endl
            << std::endl;
}

template <typename Derived1, typename Derived2>
bool MatricesMatch(const std::string& name,
                   const Eigen::MatrixBase<Derived1>& analytical,
                   const Eigen::MatrixBase<Derived2>& numerical,
                   bool tol = 1e-10) {
  const bool match = ((analytical - numerical).array().abs() < tol).all();
  if (!match) PrintMatrices(name, analytical, numerical);
  return match;
}

void TestOrientationJacobian() {
  const Eigen::Quaterniond q = get_random_quaternion();
  const Eigen::Quaterniond a = get_random_quaternion();
  const Eigen::Quaterniond b = get_random_quaternion();
  const Eigen::Quaterniond c = get_random_quaternion();

  // Test aqb
  const Eigen::Quaterniond aqb =
      AngularDistanceObjective::aqinvb(q, a, b);
  const Eigen::Quaterniond aqb_num = a * q.conjugate() * b;
  REQUIRE(MatricesMatch("aqb", aqb.coeffs(), aqb_num.coeffs()));

  // Test aqb Jacobian
  Eigen::Matrix4d daqinvb_dq;
  daqinvb_dq.topRows<3>() =
      AngularDistanceObjective::daqinvbv_dq(q, a, b);
  daqinvb_dq.bottomRows<1>() =
      AngularDistanceObjective::daqinvbw_dq(q, a, b);
  const Eigen::Matrix4d daqinvb_dq_num = ComputeNumericalJacobian(
      q,
      [&a, &b](const Eigen::Quaterniond& q) { return a * q.conjugate() * b; });
  REQUIRE(MatricesMatch("daqb_dq", daqinvb_dq, daqinvb_dq_num));

  const Eigen::Matrix4d daqinvb_dq_test =
      AngularDistanceObjective::QuaternionJacobian(
          {a, b, std::optional<Eigen::Quaterniond>()}, q);
  REQUIRE(MatricesMatch("daqb_dq", daqinvb_dq, daqinvb_dq_test));

  // Test bqc
  const Eigen::Quaterniond bqc =
      AngularDistanceObjective::bqc(q, b, c);
  const Eigen::Quaterniond bqc_num = b * q * c;
  REQUIRE(MatricesMatch("bqc", bqc.coeffs(), bqc_num.coeffs()));

  // Test bqc Jacobian
  Eigen::Matrix4d dbqc_dq;
  dbqc_dq.topRows<3>() =
      AngularDistanceObjective::dbqcv_dq(q, b, c);
  dbqc_dq.bottomRows<1>() =
      AngularDistanceObjective::dbqcw_dq(q, b, c);
  const Eigen::Matrix4d dbqc_dq_num = ComputeNumericalJacobian(
      q, [&b, &c](const Eigen::Quaterniond& q) { return b * q * c; });
  REQUIRE(MatricesMatch("dbqc_dq", dbqc_dq, dbqc_dq_num));

  const Eigen::Matrix4d dbqc_dq_test =
      AngularDistanceObjective::QuaternionJacobian(
          {std::optional<Eigen::Quaterniond>(), b, c}, q);
  REQUIRE(MatricesMatch("dbqc_dq", dbqc_dq, dbqc_dq_test));

  // Test aqbqc
  const Eigen::Quaterniond aqbqc =
      AngularDistanceObjective::aqinvbqc(q, a, b, c);
  const Eigen::Quaterniond aqbqc_num = a * q.conjugate() * b * q * c;
  REQUIRE(MatricesMatch("aqbqc", aqbqc.coeffs(), aqbqc_num.coeffs()));

  // Test aqbqc Jacobian
  Eigen::Matrix4d daqbqc_dq;
  daqbqc_dq.topRows<3>() =
      AngularDistanceObjective::daqinvbqcv_dq(q, a, b, c);
  daqbqc_dq.bottomRows<1>() =
      AngularDistanceObjective::daqinvbqcw_dq(q, a, b, c);
  const Eigen::Matrix4d daqbqc_dq_num =
      ComputeNumericalJacobian(q, [&a, &b, &c](const Eigen::Quaterniond& q) {
        return a * q.conjugate() * b * q * c;
      });
  REQUIRE(MatricesMatch("daqbqc_dq", daqbqc_dq, daqbqc_dq_num));

  const Eigen::Matrix4d daqbqc_dq_test =
      AngularDistanceObjective::QuaternionJacobian({a, b, c}, q);
  REQUIRE(MatricesMatch("daqbqc_dq", daqbqc_dq, daqbqc_dq_test));

  // Test gradient
  const Eigen::Quaterniond qq(q.coeffs() * 1.1);
  const Eigen::Matrix4d dqhat_dq =
      AngularDistanceObjective::dqhat_dq(qq);
  const Eigen::Matrix4d dqhat_dq_num = ComputeNumericalJacobian(
      qq, [](const Eigen::Quaterniond& qq) { return qq.normalized(); });
  REQUIRE(MatricesMatch("dqhat_dq", dqhat_dq, dqhat_dq_num));

  const Eigen::Matrix<double, 1, 4> dtheta_dq =
      -M_PI * aqbqc.conjugate().coeffs().transpose() * daqbqc_dq * dqhat_dq;
  const Eigen::Matrix<double, 1, 4> dtheta_dq_num = ComputeNumericalJacobian(
      qq.coeffs(), [&a, &b, &c](const Eigen::VectorXd& x) -> Eigen::VectorXd {
        const Eigen::Map<const Eigen::Quaterniond> qq(x.data());
        const Eigen::Quaterniond q = qq.normalized();
        const Eigen::Quaterniond aqbqc = a * q.conjugate() * b * q * c;
        Eigen::VectorXd g(1);
        g << M_PI / 2. * (1. - aqbqc.conjugate().coeffs().dot(aqbqc.coeffs()));
        return g;
      });
  REQUIRE(MatricesMatch("dtheta_dq", dtheta_dq, dtheta_dq_num));
}

TEST_CASE("quaternion jacobians", "[jacobians]") {
  // std::srand((unsigned int)time(0));
  std::srand(0);

  int num_tests = 20;

  // SECTION("position jacobian") {
  //   for (int i = 0; i < num_tests; i++) {
  //     REQUIRE(test_position_jacobian());
  //   }
  // }

  SECTION("orientation jacobian") {
    for (int i = 0; i < num_tests; i++) {
      // REQUIRE(test_orientation_jacobian());
      TestOrientationJacobian();
      // REQUIRE(test_orientation_jacobian());
    }
  }
}
