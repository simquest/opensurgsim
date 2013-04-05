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

#ifndef SURGSIM_PHYSICS_CONSTRAINT_H
#define SURGSIM_PHYSICS_CONSTRAINT_H

#include <SurgSim/Physics/Constraints/ConstraintImplementation.h>

#include <Eigen/Core>

#include <array>
#include <memory>

namespace SurgSim
{

namespace Physics
{

class ConstraintData;
class ConstraintImplementation;

class Constraint
{
public:
	Constraint();
	Constraint(std::shared_ptr<ConstraintImplementation> side0, std::shared_ptr<ConstraintImplementation> side1);
	virtual ~Constraint();

	void setSide(unsigned int sideId, std::shared_ptr<ConstraintImplementation> side)
	{
		m_sides[sideId] = side;
	}
	std::shared_ptr<ConstraintImplementation> getSide(unsigned int sideId)
	{
		return m_sides[sideId];
	}

	void setData(std::shared_ptr<ConstraintData> data)
	{
		m_data = data;
	}
	std::shared_ptr<ConstraintData> getData()
	{
		return m_data;
	}

	void build(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor>& a, 
		const Eigen::Matrix<double, Eigen::Dynamic, 1,  Eigen::DontAlign>& b)
	{
		doBuild(*m_data, a, b);
	}

	unsigned int getNumDof()
	{
		return std::max(m_sides[0]->getNumDof(), m_sides[1]->getNumDof());
	}

	void reset()
	{
		doReset();
		m_sides[0] = nullptr;
		m_sides[1] = nullptr;
		m_data = nullptr;
	}

private:
	std::shared_ptr<ConstraintData> m_data;
	std::array<std::shared_ptr<ConstraintImplementation>, 2> m_sides;

	virtual void doBuild(const ConstraintData& data, 
		const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign | Eigen::RowMajor>& a, 
		const Eigen::Matrix<double, Eigen::Dynamic, 1,  Eigen::DontAlign>& b);

	virtual void doReset();
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINT_H
