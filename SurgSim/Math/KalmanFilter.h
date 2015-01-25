// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_MATH_KALMANFILTER_H
#define SURGSIM_MATH_KALMANFILTER_H

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Math
{

/// Implements a linear Kalman filter, a recursive estimator.
/// Does not support control inputs.
class KalmanFilter
{
public:
	/// Set the initial state vector, x(0), length m.
	/// \param x The initial state.
	void setInitialState(const Vector& x);

	/// Set the initial covariance of the state, P(0), size m x m.
	/// \param p The initial covariance.
	void setInitialStateCovariance(const Matrix& p);

	/// Set the state transition, A, such that x(t+1) = A.x(t), size m x m.
	/// \param a The state transition matrix.
	void setStateTransition(const Matrix& a);

	/// Set the observation matrix, H, such that z(t) = H.x(t), size n x m.
	/// \param h The observation matrix.
	void setObservationMatrix(const Matrix& h);

	/// Set the process noise covariance, size m x m.
	/// \param q The process noise covariance.
	void setProcessNoiseCovariance(const Matrix& q);

	/// Set the measurement noise covariance, size n x n.
	/// \param r The measurement noise covariance.
	void setMeasurementNoiseCovariance(const Matrix& r);

	/// Advance one step without a measurement.
	/// \return The estimate for the new state, x(t+1), length m.
	const Vector& update();

	/// Advance one step with measurement.
	/// \return The estimate for the new state, x(t+1), length m.
	const Vector& update(const Vector& measurement);

	/// Get the current state.  Does not advance the state.
	/// \return The estimate for the current state, x(t), length m.
	const Vector& getState() const;

private:
	/// Use the current estimated state, x(t), and matrices to predict the new state, x(t+1), and
	/// state covariance, P(t+1).
	void updatePrediction();

	/// Correct the current estimated state, x(t), and state covariance, P(t), based on a measurement.
	/// \param measurement The measurement, length n.
	void updateMeasurement(const Vector& measurement);

	/// The state transition matrix.
	Matrix m_stateTransition;

	/// The observation matrix.
	Matrix m_observationMatrix;

	/// The process noise covariance.
	Matrix m_processNoiseCovariance;

	/// The measurement noise covariance.
	Matrix m_measurementNoiseCovariance;

	/// The state.
	Vector m_state;

	/// The covariance of the state.
	Matrix m_stateCovariance;
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_KALMANFILTER_H
