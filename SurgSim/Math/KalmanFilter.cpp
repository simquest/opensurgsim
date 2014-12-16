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

#include "SurgSim/Math/KalmanFilter.h"

namespace SurgSim
{
namespace Math
{

void KalmanFilter::setInitialState(const Vector& x)
{
	m_state = x;
}

void KalmanFilter::setInitialStateCovariance(const Matrix& p)
{
	m_stateCovariance = p;
}

void KalmanFilter::setStateTransition(const Matrix& a)
{
	m_stateTransition = a;
}

void KalmanFilter::setObservationMatrix(const Matrix& h)
{
	m_observationMatrix = h;
}

void KalmanFilter::setProcessNoiseCovariance(const Matrix& q)
{
	m_processNoiseCovariance = q;
}

void KalmanFilter::setMeasurementNoiseCovariance(const Matrix& r)
{
	m_measurementNoiseCovariance = r;
}

const Vector& KalmanFilter::update()
{
	updatePrediction();
	return m_state;
}

const Vector& KalmanFilter::update(const Vector& measurement)
{
	updatePrediction();
	updateMeasurement(measurement);
	return m_state;
}

const Vector& KalmanFilter::getState() const
{
	return m_state;
}

void KalmanFilter::updatePrediction()
{
	m_state = m_stateTransition * m_state;
	m_stateCovariance = m_stateTransition * m_stateCovariance * m_stateTransition.transpose() +
		m_processNoiseCovariance;
}

void KalmanFilter::updateMeasurement(const Vector& measurement)
{
	const Matrix gain = m_stateCovariance * m_observationMatrix.transpose() *
		(m_observationMatrix * m_stateCovariance * m_observationMatrix.transpose() +
		m_measurementNoiseCovariance).inverse();
	m_state += gain * (measurement - m_observationMatrix * m_state);
	m_stateCovariance -= gain * m_observationMatrix * m_stateCovariance;
}

}; // namespace Math
}; // namespace SurgSim
