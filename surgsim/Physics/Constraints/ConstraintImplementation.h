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

#ifndef SURGSIM_PHYSICS_CONSTRAINT_IMPLEMENTATION_H
#define SURGSIM_PHYSICS_CONSTRAINT_IMPLEMENTATION_H

#include <Eigen/Core>

#include <memory>

namespace SurgSim
{

namespace Physics
{

class Localization;
class ConstraintData;

class ConstraintImplementation
{
public:
	ConstraintImplementation();
	ConstraintImplementation(std::shared_ptr<Localization> location);
	virtual ~ConstraintImplementation();

	void setLocation(std::shared_ptr<Localization> location)
	{
		m_location = location;
	}
	std::shared_ptr<Localization> getLocation() const
	{
		return m_location;
	}

	unsigned int getNumDof() const
	{
		return doGetNumDof();
	}

	void build(const ConstraintData& data, 
		const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor>& a, 
		const Eigen::Matrix<double, Eigen::Dynamic, 1,  Eigen::DontAlign>& b)
	{
		doBuild(data, a, b);
	}

private:
	std::shared_ptr<Localization> m_location;

	virtual unsigned int doGetNumDof() const = 0;

	virtual void doBuild(const ConstraintData& data,
		const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor>& a, 
		const Eigen::Matrix<double, Eigen::Dynamic, 1,  Eigen::DontAlign>& b) = 0;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINT_IMPLEMENTATION_H
