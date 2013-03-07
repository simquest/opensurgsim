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

#include "SimpleSquareForce.h"

#include <SurgSim/DataStructures/DataGroupBuilder.h>

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;


SimpleSquareForce::SimpleSquareForce() :
	m_squareHalfSize(0.050),
	m_surfaceStiffness(250.0),
	m_forceLimit(5.0),
	m_squareNormal(0, -1, 0),
	m_squareEdgeDirectionX(1, 0, 0),
	m_squareEdgeDirectionY(0, 0, 1),
	m_squareCenter(0, 0, 0),
	m_tipPoint(0, 0, 0)
{
	DataGroupBuilder builder;
	builder.addVector("force");
	builder.addVector("torque");
	m_outputData = std::move(builder.createData());

	m_outputData.vectors().put("force", Vector3d(0, 0, 0));
	m_outputData.vectors().put("torque", Vector3d(0, 0, 0));
}

void SimpleSquareForce::handleInput(const std::string& device, const DataGroup& inputData)
{
	m_outputData.resetAll();

	RigidTransform3d devicePose;
	if (! inputData.poses().get("pose", devicePose))
	{
		m_outputData.vectors().put("force", Vector3d(0, 0, 0));
		m_outputData.vectors().put("torque", Vector3d(0, 0, 0));
		return;
	}

	Vector3d tipPosition = devicePose * m_tipPoint;
	m_outputData.vectors().put("force", computeForce(tipPosition));
	m_outputData.vectors().put("torque", Vector3d(0, 0, 0));
}

bool SimpleSquareForce::requestOutput(const std::string& device, DataGroup* outputData)
{
	*outputData = m_outputData;
	return true;
}

Vector3d SimpleSquareForce::computeForce(const Vector3d& position)
{
	Vector3d force(0, 0, 0);

	Vector3d offset = position - m_squareCenter;
	double penetration = offset.dot(m_squareNormal);
	if (penetration > 0)
	{
		double planeDistanceX = offset.dot(m_squareEdgeDirectionX);
		double planeDistanceY = offset.dot(m_squareEdgeDirectionY);
		bool insideSquare = (fabs(planeDistanceX) < m_squareHalfSize) && (fabs(planeDistanceY) < m_squareHalfSize);

		force = -m_surfaceStiffness * penetration * m_squareNormal;

		if ((force.norm() > m_forceLimit) || ! insideSquare)
		{
			force.setZero();
			m_squareNormal = -m_squareNormal;
		}

	}
	return force;
}
