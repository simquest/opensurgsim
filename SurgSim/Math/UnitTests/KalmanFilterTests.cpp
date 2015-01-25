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

/// \file
/// Tests for the KalmanFilter.cpp functions.

#include <gtest/gtest.h>

#include "SurgSim/Math/KalmanFilter.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

typedef Eigen::Matrix<double, 1, 1> Vector1d;
typedef Eigen::Matrix<double, 1, 1, Eigen::RowMajor> Matrix11d;

namespace SurgSim
{

namespace Math
{
TEST(KalmanFilterTests, 1DConstant)
{
	auto kalman = std::make_shared<KalmanFilter>();
	kalman->setInitialState(Vector1d::Constant(0.0)); // the state is a scalar, and we have an initial guess for it
	kalman->setInitialStateCovariance(Matrix11d::Constant(1000.0)); // the uncertainty about initial guess is high
	kalman->setStateTransition(Matrix11d::Constant(1.0)); // we predict the true state will stay constant
	kalman->setObservationMatrix(Matrix11d::Constant(1.0)); // we observe the actual state plus measurement error
	kalman->setProcessNoiseCovariance(Matrix11d::Constant(0.0001)); // the noise in the prediction action is low
	kalman->setMeasurementNoiseCovariance(Matrix11d::Constant(0.1)); // the assumed measurement noise

	const double measurements[] = {0.9, 0.8, 1.1, 1, 0.95, 1.05, 1.2, 0.9, 0.85, 1.15};
	for (double measurement : measurements)
	{
		kalman->update(Vector1d::Constant(measurement));
	}

	const double result = 0.99046032523740679;
	EXPECT_NEAR(result , kalman->getState()[0], 1e-9);
	// The state doesn't change unless there's an update.
	EXPECT_NEAR(result, kalman->getState()[0], 1e-9);
	// Updating a constant model without a measurement shouldn't change the state.
	EXPECT_NEAR(result, kalman->update()[0], 1e-9);
}

TEST(KalmanFilterTests, 1DVelocity)
{
	auto kalman = std::make_shared<KalmanFilter>();
	// The state is the position and velocity, and here's our initial guess.
	const Vector2d initialState = Vector2d::Zero();
	kalman->setInitialState(initialState);
	const Matrix22d initialStateCovariance = 1000.0 * Matrix22d::Identity();
	kalman->setInitialStateCovariance(initialStateCovariance);
	// The new position is the old position plus velocity times dt.  The velocity is constant.
	const double dt = 0.1;
	Matrix22d stateTransition;
	stateTransition << 1.0, dt,
						0.0, 1.0;
	kalman->setStateTransition(stateTransition);
	// The measurements are of the position, plus measurement error.
	Eigen::Matrix<double, 1, 2, Eigen::RowMajor> observationMatrix;
	observationMatrix.setIdentity();
	kalman->setObservationMatrix(observationMatrix);
	const double velocityNoise = 0.001;
	// Assume the noise is only in the velocity, so a continuous noise model of [0, 0; 0, velocityNoise], then
	// approximate that by a time-discrete process to get...
	Matrix22d processNoise;
	processNoise << dt * dt * dt * velocityNoise / 3.0, dt * dt * velocityNoise / 2.0,
					dt * dt * velocityNoise / 3.0,			dt * velocityNoise;
	kalman->setProcessNoiseCovariance(processNoise);
	kalman->setMeasurementNoiseCovariance(Matrix11d::Constant(0.1));

	const double measurements[] = {0.11, 0.29, 0.32, 0.5, 0.58, 0.54};
	for (double measurement : measurements)
	{
		kalman->update(Vector1d::Constant(measurement));
	}

	// check the state
	const double position = 0.61844193701221828;
	EXPECT_NEAR(position, kalman->getState()[0], 1e-9);
	const double velocity = 0.91376229444025137;
	EXPECT_NEAR(velocity, kalman->getState()[1], 1e-9);

	// The state doesn't change unless there's an update.
	EXPECT_NEAR(position, kalman->getState()[0], 1e-9);
	// Updating without a measurement will predict ahead.
	EXPECT_NEAR(position + velocity * dt, kalman->update()[0], 1e-9);
	EXPECT_NEAR(velocity, kalman->getState()[1], 1e-9);
}
}; // namespace Math

}; // namespace SurgSim
