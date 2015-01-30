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

#ifndef SURGSIM_MATH_KALMANFILTER_INL_H
#define SURGSIM_MATH_KALMANFILTER_INL_H

namespace SurgSim
{
namespace Math
{

template <size_t M, size_t N>
KalmanFilter<M, N>::KalmanFilter() :
	m_stateTransition(Eigen::Matrix<double, M, M, Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN())),
	m_observationMatrix(Eigen::Matrix<double, N, M, Eigen::RowMajor>::Constant(
	std::numeric_limits<double>::quiet_NaN())),
	m_processNoiseCovariance(Eigen::Matrix<double, M, M, Eigen::RowMajor>::Constant(
		std::numeric_limits<double>::quiet_NaN())),
	m_measurementNoiseCovariance(Eigen::Matrix<double, N, N, Eigen::RowMajor>::Constant(
		std::numeric_limits<double>::quiet_NaN())),
	m_state(Eigen::Matrix<double, M, 1>::Constant(std::numeric_limits<double>::quiet_NaN())),
	m_stateCovariance(Eigen::Matrix<double, M, M, Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()))
{
}

template <size_t M, size_t N>
KalmanFilter<M, N>::~KalmanFilter()
{
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::setInitialState(const Eigen::Ref<const Eigen::Matrix<double, M, 1>>& x)
{
	m_state = x;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::setInitialStateCovariance(const Eigen::Ref<const Eigen::Matrix<double, M, M>>& p)
{
	m_stateCovariance = p;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::setStateTransition(const Eigen::Ref<const Eigen::Matrix<double, M, M>>& a)
{
	m_stateTransition = a;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::setObservationMatrix(const Eigen::Ref<const Eigen::Matrix<double, N, M>>& h)
{
	m_observationMatrix = h;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::setProcessNoiseCovariance(const Eigen::Ref<const Eigen::Matrix<double, M, M>>& q)
{
	m_processNoiseCovariance = q;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::setMeasurementNoiseCovariance(const Eigen::Ref<const Eigen::Matrix<double, N, N>>& r)
{
	m_measurementNoiseCovariance = r;
}

template <size_t M, size_t N>
const Eigen::Matrix<double, M, 1>& KalmanFilter<M, N>::update()
{
	updatePrediction();
	return m_state;
}

template <size_t M, size_t N>
const Eigen::Matrix<double, M, 1>&
	KalmanFilter<M, N>::update(const Eigen::Ref<const Eigen::Matrix<double, N, 1>>& measurement)
{
	updatePrediction();
	updateMeasurement(measurement);
	return m_state;
}

template <size_t M, size_t N>
const Eigen::Matrix<double, M, 1>& KalmanFilter<M, N>::getState() const
{
	return m_state;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::updatePrediction()
{
	m_state = m_stateTransition * m_state;
	m_stateCovariance = m_stateTransition * m_stateCovariance * m_stateTransition.transpose() +
		m_processNoiseCovariance;
}

template <size_t M, size_t N>
void KalmanFilter<M, N>::updateMeasurement(const Eigen::Ref<const Eigen::Matrix<double, N, 1>>& measurement)
{
	const Matrix gain = m_stateCovariance * m_observationMatrix.transpose() *
		(m_observationMatrix * m_stateCovariance * m_observationMatrix.transpose() +
		m_measurementNoiseCovariance).inverse();
	m_state += gain * (measurement - m_observationMatrix * m_state);
	m_stateCovariance -= gain * m_observationMatrix * m_stateCovariance;
}

}; // namespace Math
}; // namespace SurgSim

#endif // SURGSIM_MATH_KALMANFILTER_INL_H