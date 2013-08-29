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
/// Definition of the load (force + torque) computational type.

#ifndef SURGSIM_MATH_LOAD_H
#define SURGSIM_MATH_LOAD_H

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Math
{


template <typename T>
class Load
{
public:
	typedef T Scalar;
	typedef typename Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vector3;

	Load()
	{
		SURGSIM_FAILURE() << "unimplemented";
	}

	Load(const Load& other)
	{
		SURGSIM_FAILURE() << "unimplemented";
	}

	Load& operator=(const Load& other);

	Load& setZero();

	static Load Zero();

	void addForceAtPoint(const Vector3& force, const Vector3& point);

	void addForceWithoutMoment(const Vector3& force);

	void addPureMoment(const Vector3& moment);

	Vector3 getForce() const;

	Vector3 getMomentAroundPoint(const Vector3& point) const;

	Load& operator+=(const Load& other);

	Load& operator-=(const Load& other);

	Load operator+(const Load& other) const;

	Load operator-(const Load& other) const;

	Load operator-() const;

	template <typename V>
	Load& applyTranslation(const V& translation);

	template <typename M>
	Load& applyRotation(const M& rotation);

	template <int Flags>
	Load& applyRigidTransform(const Eigen::Transform<T, 3, Eigen::Isometry, Flags>& transform);

	template <typename U>
	Load<U> cast() const;
};


template <typename T, int Flags>
Load<T> operator*(const Eigen::Transform<T, 3, Eigen::Isometry, Flags>& transform, const Load<T>& load)
{
	return Load<T>(load).applyRigidTransform(transform);
}


/// A load data type using floats.
typedef Load<float> Loadf;

/// A load data type using doubles.
typedef Load<double> Loadd;


};  // namespace Math
};  // namespace SurgSim

#endif  // SURGSIM_MATH_LOAD_H
