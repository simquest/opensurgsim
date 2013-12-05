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

#include "SurgSim/Testing/VisualTestCommon/MovingSquareForce.h"

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

#include "SurgSim/Framework/Assert.h"

#include "SurgSim/DataStructures/DataGroupBuilder.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;


MovingSquareForce::SquarePoseVectors::SquarePoseVectors() :
	normal(0, -1, 0),
	edgeDirectionX(1, 0, 0),
	edgeDirectionY(0, 0, 1),
	center(0, 0, 0)
{
}


MovingSquareForce::MovingSquareForce(const std::string& toolDeviceName, const std::string& squareDeviceName) :
	m_toolDeviceName(toolDeviceName),
	m_squareDeviceName(squareDeviceName),
	m_squareHalfSize(0.050),
	m_surfaceStiffness(250.0),
	m_forceLimit(5.0),
	m_squareNormalDirection(+1),
	m_tipPoint(0, 0, 0)
{
	DataGroupBuilder builder;
	builder.addVector("force");
	builder.addVector("torque");
	m_outputData = std::move(builder.createData());

	m_outputData.vectors().set("force", Vector3d(0, 0, 0));
	m_outputData.vectors().set("torque", Vector3d(0, 0, 0));
}

void MovingSquareForce::initializeInput(const std::string& device, const DataGroup& inputData)
{
}

void MovingSquareForce::handleInput(const std::string& device, const DataGroup& inputData)
{
	if (device == m_toolDeviceName)
	{
		updateTool(inputData);
	}
	else if (device == m_squareDeviceName)
	{
		updateSquare(inputData);
	}
	else
	{
		SURGSIM_FAILURE() << "Unknown device name '" << device << "'";
	}
}

void MovingSquareForce::updateTool(const DataGroup& inputData)
{
	m_outputData.resetAll();

	RigidTransform3d devicePose;
	if (! inputData.poses().get("pose", &devicePose))
	{
		m_outputData.vectors().set("force", Vector3d(0, 0, 0));
		m_outputData.vectors().set("torque", Vector3d(0, 0, 0));
		return;
	}

	Vector3d tipPosition = devicePose * m_tipPoint;
	m_outputData.vectors().set("force", computeForce(tipPosition));
	m_outputData.vectors().set("torque", Vector3d(0, 0, 0));
}

void MovingSquareForce::updateSquare(const DataGroup& inputData)
{
	RigidTransform3d devicePose;
	if (! inputData.poses().get("pose", &devicePose))
	{
		return;
	}

	SquarePoseVectors square;
	square.normal = devicePose.linear() * Vector3d(0, -1, 0);
	square.edgeDirectionX = devicePose.linear() * Vector3d(1, 0, 0);
	square.edgeDirectionY = devicePose.linear() * Vector3d(0, 0, 1);
	square.center = devicePose * Vector3d(0, 0, 0);

	// Push into thread-safe storage so the other device can safely access it
	m_square.set(square);
}

bool MovingSquareForce::requestOutput(const std::string& device, DataGroup* outputData)
{
	*outputData = m_outputData;
	return true;
}

Vector3d MovingSquareForce::computeForce(const Vector3d& position)
{
	Vector3d force(0, 0, 0);

	SquarePoseVectors square;
	m_square.get(&square);
	Vector3d offset = position - square.center;
	double orthogonalDistance = offset.dot(square.normal);

	if ((orthogonalDistance * m_squareNormalDirection) > 0)
	{
		double planeDistanceX = offset.dot(square.edgeDirectionX);
		double planeDistanceY = offset.dot(square.edgeDirectionY);
		bool insideSquare = (fabs(planeDistanceX) < m_squareHalfSize) && (fabs(planeDistanceY) < m_squareHalfSize);

		force = -m_surfaceStiffness * orthogonalDistance * square.normal;

		if ((force.norm() > m_forceLimit) || ! insideSquare)
		{
			force.setZero();
			m_squareNormalDirection = -m_squareNormalDirection;
		}

	}
	return force;
}
