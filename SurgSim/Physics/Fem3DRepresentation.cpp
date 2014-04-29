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

#include "SurgSim/Physics/Fem3DRepresentation.h"

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/Fem3DRepresentationPlyReaderDelegate.h"

using SurgSim::Framework::Logger;

namespace
{
void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform,
										SurgSim::Math::Vector* x, bool rotationOnly = false)
{
	typedef SurgSim::Math::Vector::Index IndexType;

	IndexType numNodes = x->size() / 3;
	SURGSIM_ASSERT(numNodes * 3 == x->size()) <<
		"Unexpected number of dof in a Fem3D state vector (not a multiple of 3)";

	for (IndexType nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::Vector3d xi = SurgSim::Math::getSubVector(*x, nodeId, 3);
		SurgSim::Math::Vector3d xiTransformed;
		if (rotationOnly)
		{
			xiTransformed = transform.linear() * xi;
		}
		else
		{
			xiTransformed = transform * xi;
		}
		SurgSim::Math::setSubVector(xiTransformed, nodeId, 3, x);
	}
}
}

namespace SurgSim
{

namespace Physics
{

Fem3DRepresentation::Fem3DRepresentation(const std::string& name) :
	FemRepresentation(name), m_doLoadFile(false)
{
	// Reminder: m_numDofPerNode is held by DeformableRepresentation
	// but needs to be set by all concrete derived classes
	m_numDofPerNode = 3;
}

Fem3DRepresentation::~Fem3DRepresentation()
{
}

RepresentationType Fem3DRepresentation::getType() const
{
	return REPRESENTATION_TYPE_FEM3D;
}

void Fem3DRepresentation::applyCorrection(double dt,
										  const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity)
{
	if (!isActive())
	{
		return;
	}

	m_currentState->getPositions() += deltaVelocity * dt;
	m_currentState->getVelocities() += deltaVelocity;

	if (!isValidState(*m_currentState))
	{
		deactivateAndReset();
	}
}

void Fem3DRepresentation::setFilename(const std::string& filename)
{
	m_filename = filename;

	m_doLoadFile = !m_filename.empty();
}

const std::string& Fem3DRepresentation::getFilename() const
{
	return m_filename;
}

bool Fem3DRepresentation::loadFile()
{
	if (m_filename.empty())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "Filename not set.";
		return false;
	}

	if (!m_doLoadFile)
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "File already loaded.";
		return false;
	}

	SurgSim::DataStructures::PlyReader reader(m_filename);
	auto thisAsSharedPtr = std::static_pointer_cast<Fem3DRepresentation>(getSharedPtr());
	auto readerDelegate = std::make_shared<Fem3DRepresentationPlyReaderDelegate>(thisAsSharedPtr);

	if (!reader.isValid())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "File " << m_filename << " is not valid.";
		return false;
	}

	if (!reader.setDelegate(readerDelegate))
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << "File " << m_filename << " is not acceptable an PLY.";
		return false;
	}

	// PlyReader::parseFile loads the fem into the shared_ptr passed to the readerDelegate constructor.
	reader.parseFile();

	m_doLoadFile = false;
	return true;
}

bool Fem3DRepresentation::doInitialize()
{
	if (m_doLoadFile && !loadFile())
	{
		SURGSIM_LOG_SEVERE(Logger::getDefaultLogger()) << "Failed to initialize from file " << m_filename;
		return false;
	}

	return FemRepresentation::doInitialize();
}

void Fem3DRepresentation::transformState(std::shared_ptr<DeformableRepresentationState> state,
	const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
	transformVectorByBlockOf3(transform, &state->getAccelerations(), true);
}

bool Fem3DRepresentation::isValidState(const DeformableRepresentationState &state) const
{
	return SurgSim::Math::isValid(state.getPositions())
		&& SurgSim::Math::isValid(state.getVelocities());
}

void Fem3DRepresentation::deactivateAndReset(void)
{
	SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), DEBUG)
		<< getName() << " deactivated and reset:" << std::endl
		<< "position=(" << m_currentState->getPositions() << ")" << std::endl
		<< "velocity=(" << m_currentState->getVelocities() << ")" << std::endl
		<< "acceleration=(" << m_currentState->getAccelerations() << ")" << std::endl;

	resetState();
	setIsActive(false);
}

} // namespace Physics

} // namespace SurgSim
