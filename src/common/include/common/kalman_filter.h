#ifndef KALMANFILTER_COMMON_H
#define KALMANFILTER_COMMON_H

#include <common/macros.h>

THIRD_PARTY_HEADERS_BEGIN
#include <eigen3/Eigen/Dense>
THIRD_PARTY_HEADERS_END

namespace common {

/*!
 * \brief The KalmanFilter class template for linear (time variant or time
 * invariant) kalman filter using Eigen matrices and vectors as interface. The
 * kalman filter is implemented as square root kalman filter, improving on the
 * numerical condition compared to a standard kalman filter implementation
 * (which can often be numerically unstable  depending on the system matrix and
 * the noise matrices).
 * The system model (system matrix, control vector and system noise matrix) and
 * the measurement model (measurement matrix and measurement noise matrix) can
 * be set once in the constructor (for time invariant kalman filter) or before
 * every predict/innovate call (for time variant kalman filter).
 * predict() and innovate(measurements) are usually called in alternation, with
 * one kalman iteration being one call to predict() followed by one call to
 * innovate(measurements) and the state estimation being the result of the
 * prediction. It is however valid and sometimes sensible to call predict()
 * multiple times consecutively without calls to innovate(measurements) in
 * between (e.g. if no measurements are available) and to use the result of
 * predict() as state estimation. Changing state and measurement vector
 * dimensions and partial innovations are currently not supported.
 * Even with the square root implementation, care should still be taken in
 * regard to the numerical condition. For this reason, there shouldn't be too
 * many orders of magnitude between the different values of the system and
 * measurement noise covariances.
 *
 * state_dim: dimension of state vector
 * measurement_dim: dimension of measurement vector
 */
template <unsigned int state_dim, unsigned int measurement_dim>
class KalmanFilter {
 public:
  typedef Eigen::Matrix<double, state_dim, state_dim> SystemNoiseMatrix;
  typedef Eigen::Matrix<double, state_dim, state_dim> SystemMatrix;
  typedef Eigen::Matrix<double, measurement_dim, measurement_dim> MeasurementNoiseMatrix;
  typedef Eigen::Matrix<double, measurement_dim, state_dim> MeasurementMatrix;
  typedef Eigen::Matrix<double, state_dim, measurement_dim> KalmanGainMatrix;
  typedef Eigen::Matrix<double, state_dim, state_dim> StateCovarianceMatrix;
  typedef Eigen::Matrix<double, state_dim, 1> StateVector;
  typedef Eigen::Matrix<double, state_dim, 1> ControlVector;
  typedef Eigen::Matrix<double, measurement_dim, 1> MeasurementVector;

  /*!
   * \brief KalmanFilter constructor for the kalman filter. This constructor is
   * especially useful for time invariant kalman filter, for which all
   * parameters are known at time of the kalman filter initialization.
   * \param system_matrix the system matrix of the system model
   * \param system_noise_matrix covariance matrix of the (assumed) additive zero
   * mean gaussian system noise
   * \param measurement_matrix the measurement matrix of the measurement model
   * \param measurement_noise_matrix covariance matrix of the (assumed) additive
   * zero mean gaussian system noise
   * \param control_vector control vector of the system model
   * \param initial_state state initialization
   * \param initial_state_covariance state covariance initialization. Usually a
   * high value is used to indicate a high uncertainty of the initial state, but
   * too high values should be avoided since very large initial covariance
   * values impair the numerical condition of the kalman filter.
   */
  KalmanFilter(const SystemMatrix& system_matrix,
               const SystemNoiseMatrix& system_noise_matrix,
               const MeasurementMatrix& measurement_matrix,
               const MeasurementNoiseMatrix& measurement_noise_matrix,
               const ControlVector& control_vector = ControlVector::Zero(),
               const StateVector& initial_state = StateVector::Zero(),
               double initial_state_covariance = 1000.)
      : system_matrix(system_matrix),
        control_vector(control_vector),
        measurement_matrix(measurement_matrix),
        system_noise_matrix(system_noise_matrix),
        measurement_noise_matrix(measurement_noise_matrix),
        state(initial_state) {
    resetStateAndStateCovariance(initial_state_covariance, initial_state);
  }

  /*!
   * \brief KalmanFilter default constructor. Useful for time variant kalman
   * filter if the parameters have to be updated before every kalman filter
   * iteration anyway.
   */
  KalmanFilter()
      : KalmanFilter(SystemMatrix::Identity(),
                     SystemNoiseMatrix::Identity(),
                     MeasurementMatrix::Identity(),
                     MeasurementNoiseMatrix::Identity()) {}

  /*!
   * \brief kalman filter prediction step
   * \return prediction state
   */
  StateVector predict() {
    system_noise_matrix_L = system_noise_matrix.llt().matrixL();

    state = system_matrix * state + control_vector;

    // formulas and notation from "Discrete Square Root Filtering: A Survey of
    // Current Techniques" by Kaminski et al.

    // define an intermediate matrix
    Eigen::Matrix<double, state_dim, state_dim> upper_part_of_intermediate_matrix =
        state_covariance_matrix_root.transpose() * system_matrix.transpose();
    Eigen::Matrix<double, state_dim, state_dim> lower_part_of_intermediate_matrix =
        system_noise_matrix_L.transpose();
    Eigen::Matrix<double, 2 * state_dim, state_dim> intermediate_matrix;
    intermediate_matrix.template block<state_dim, state_dim>(0, 0)
        << upper_part_of_intermediate_matrix;
    intermediate_matrix.template block<state_dim, state_dim>(state_dim, 0)
        << lower_part_of_intermediate_matrix;

    // prediction update
    typedef Eigen::Matrix<double, 2 * state_dim, state_dim> QRtype;
    Eigen::HouseholderQR<QRtype> qr_decomposition = intermediate_matrix.householderQr();
    QRtype qr_r_matrix =
        qr_decomposition.matrixQR().template triangularView<Eigen::Upper>();
    state_covariance_matrix_root =
        qr_r_matrix.template block<state_dim, state_dim>(0, 0).transpose();

    return state;
  }

  /*!
   * \brief kalman filter innovation step
   * \param measurement_vector sensor measurements
   * \return state estimation
   */
  StateVector innovate(const MeasurementVector& measurement_vector) {
    measurement_noise_matrix_L = measurement_noise_matrix.llt().matrixL();

    // covariance update and kalman gain calculation
    // define intermediate variable F
    Eigen::Matrix<double, state_dim, measurement_dim> F =
        state_covariance_matrix_root.transpose() * measurement_matrix.transpose();

    innovation_signal_covariance = measurement_noise_matrix + F.transpose() * F;

    // define intermediate variable G
    Eigen::Matrix<double, measurement_dim, measurement_dim> G =
        (innovation_signal_covariance).llt().matrixL();

    // solve inverts *this and multiplies *this with its argument (in this case
    // the identity matrix, so equivalent to a pure inversion of *this) (uses
    // backsubstitution for inversion, so more efficient and numerical stable
    // than
    // inversion by other means (but only works for triangular matrices))
    Eigen::Matrix<double, measurement_dim, measurement_dim> G_inverted =
        G.template triangularView<Eigen::Lower>().solve(
            Eigen::Matrix<double, measurement_dim, measurement_dim>::Identity());

    kalman_gain = state_covariance_matrix_root * F * G_inverted.transpose() * G_inverted;

    // calculate the cholesky decompotion of next_state_covariance S+
    // clang-format off
    state_covariance_matrix_root =
        state_covariance_matrix_root -
        state_covariance_matrix_root * F * G_inverted.transpose() *
            (G + measurement_noise_matrix_L).template triangularView<Eigen::Lower>().solve(F.transpose());
    // clang-format on


    // state update
    state = state + kalman_gain * (measurement_vector - measurement_matrix * state);

    return state;
  }

  /*!
   * \brief resetStateAndStateCovariance resets the state to initial_state and
   * the state covariance to initial_state_covariance times identity matrix.
   * Too high values for initial_state_covariance can impair the numerical
   * condition of the kalman filter
   * \param initial_state_covariance the value for the diagonal elements of the
   * new state covariance matrix
   * \param initial_state the new state vector value
   */
  void resetStateAndStateCovariance(double initial_state_covariance = 1000.,
                                    const StateVector& initial_state = StateVector::Zero()) {
    state = initial_state;

    const StateCovarianceMatrix state_covariance_matrix =
        initial_state_covariance * StateCovarianceMatrix::Identity();

    // cholesky decomposition of state_covariance s+
    state_covariance_matrix_root = state_covariance_matrix.llt().matrixL();
  }

  /*!
   * \brief the system matrix
   */
  SystemMatrix system_matrix;

  /*!
   * \brief control vector of the system
   */
  ControlVector control_vector;

  /*!
   * \brief measurement matrix
   */
  MeasurementMatrix measurement_matrix;

  /*!
   * \brief system noise matrix (additive covariance matrix of the system)
   */
  SystemNoiseMatrix system_noise_matrix;

  /*!
   * \brief the measurement noise matrix (additive covariance matrix of the
   * measurement process)
   */
  MeasurementNoiseMatrix measurement_noise_matrix;

  /*!
   * \brief getState returns current state vector
   * \return the state
   */
  StateVector getState() const { return state; }

  /*!
   * \brief getKalmanGain return current Kalman gain matrix
   * \return the kalman gain
   */
  KalmanGainMatrix getKalmanGain() const { return kalman_gain; }

  /*!
   * \brief getStateCovarianceMatrix return the current state covariance matrix.
   * Internally the matrix root of the state covariance matrix is used.
   * \return state covariance
   */
  StateCovarianceMatrix getStateCovarianceMatrix() const {
    return state_covariance_matrix_root * state_covariance_matrix_root.transpose();
  }

  /*!
   * \brief getStateCovarianceMatrixRoot return the matrix root (in terms of the
   * cholesky decomposition) of the state covariance matrix
   * \return the root of the state covariance matrix
   */
  StateCovarianceMatrix getStateCovarianceMatrixRoot() const {
    return state_covariance_matrix_root;
  }

  /*!
   * \brief getInnovationSignalCovariance returns the innovation signal
   * covariance. This matrix gets calculated during the calculation of the
   * kalman gain. It's condition number is a good measure of the numerical
   * condition of the kalman filter. \return the innovation signal covariance
   */
  MeasurementNoiseMatrix getInnovationSignalCovariance() const {
    return innovation_signal_covariance;
  }

 protected:
  /*!
   * \brief the initial state
   */
  const StateVector initial_state;

  /*!
   * \brief the estimated state
   */
  StateVector state;

  /*!
   * \brief kalman gain
   */
  KalmanGainMatrix kalman_gain = KalmanGainMatrix::Zero();

  /*!
   * \brief the matrix root of the estimated covariance matrix of the state
   * estimation
   */
  StateCovarianceMatrix state_covariance_matrix_root;

  /*!
   * \brief root of the measurement noise matrix (additive covariance matrix of
   * the measurement process)
   */
  MeasurementNoiseMatrix measurement_noise_matrix_L;

  /*!
   * \brief root of the system noise matrix (additive covariance matrix of the
   * system)
   */
  SystemNoiseMatrix system_noise_matrix_L;

  /*!
   * \brief this matrix gets calculated during the calculation of the kalman
   * gain. It's condition number is a good measure of the numerical condition of
   * the kalman filter.
   */
  MeasurementNoiseMatrix innovation_signal_covariance = MeasurementNoiseMatrix::Zero();
};

}  // namespace common

#endif  // KALMAN_FILTER_COMMON
