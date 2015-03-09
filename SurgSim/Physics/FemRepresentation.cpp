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

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemPlyReaderDelegate.h"
#include "SurgSim/Physics/FemRepresentation.h"

namespace SurgSim
{

namespace Physics
{

FemRepresentation::FemRepresentation(const std::string& name) :
	DeformableRepresentation(name)
{
	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(FemRepresentation, std::string, Filename, getFilename, setFilename);
}

FemRepresentation::~FemRepresentation()
{
}

void FemRepresentation::setFilename(const std::string& filename)
{
	m_filename = filename;
}

const std::string& FemRepresentation::getFilename() const
{
	return m_filename;
}

bool FemRepresentation::loadFile()
{
	using SurgSim::Framework::Logger;

	bool result = true;
	if (m_filename.empty())
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << __FUNCTION__ << "Filename is empty";
		result = false;
	}
	else
	{
		std::string filePath = getRuntime()->getApplicationData()->findFile(m_filename);
		if (filePath.empty())
		{
			SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << __FUNCTION__ <<
					"File " << m_filename << " can not be found.";
			result = false;
		}

		auto reader = std::make_shared<SurgSim::DataStructures::PlyReader>(filePath);
		if (result && !reader->isValid())
		{
			SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << __FUNCTION__ <<
					"File " << m_filename << " is invalid.";
			result = false;
		}

		if (result && !reader->parseWithDelegate(getDelegate()))
		{
			SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << __FUNCTION__ << "Failed to load file " << m_filename;
			result = false;
		}
	}

	return result;
}

bool FemRepresentation::doInitialize()
{
	if (!m_filename.empty() && !loadFile())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << __FUNCTION__ <<
				"Failed to initialize from file " << m_filename;
		return false;
	}

	SURGSIM_ASSERT(m_initialState != nullptr) << "You must set the initial state before calling Initialize";

	// Initialize the FemElements
	for (auto element = std::begin(m_femElements); element != std::end(m_femElements); element++)
	{
		(*element)->initialize(*m_initialState);
	}

	// Allocate the vector m_massPerNode
	if (m_massPerNode.size() == 0 || m_massPerNode.size() < m_initialState->getNumNodes())
	{
		m_massPerNode.resize(m_initialState->getNumNodes());
	}

	// Compute the entries of m_massPerNode from the FemElements mass information
	for (auto element = std::begin(m_femElements); element != std::end(m_femElements); element++)
	{
		double mass = (*element)->getMass(*m_initialState);
		for (auto nodeId = std::begin((*element)->getNodeIds()); nodeId != std::end((*element)->getNodeIds()); nodeId++)
		{
			m_massPerNode[*nodeId] += mass / (*element)->getNumNodes();
		}
	}

	return true;
}

void FemRepresentation::addFemElement(const std::shared_ptr<FemElement> femElement)
{
	m_femElements.push_back(femElement);
}

size_t FemRepresentation::getNumFemElements() const
{
	return m_femElements.size();
}

std::shared_ptr<FemElement> FemRepresentation::getFemElement(size_t femElementId)
{
	SURGSIM_ASSERT(femElementId < getNumFemElements()) << "Invalid femElement id";
	return m_femElements[femElementId];
}

bool FemRepresentation::isValidCoordinate(const SurgSim::DataStructures::IndexedLocalCoordinate& coordinate) const
{
	return (coordinate.index < m_femElements.size())
		   && m_femElements[coordinate.index]->isValidCoordinate(coordinate.coordinate);
}

double FemRepresentation::getTotalMass() const
{
	double mass = 0.0;
	for (auto it = std::begin(m_femElements); it != std::end(m_femElements); it++)
	{
		mass += (*it)->getMass(*m_currentState);
	}
	return mass;
}

double FemRepresentation::getRayleighDampingStiffness() const
{
	return m_rayleighDamping.stiffnessCoefficient;
}

double FemRepresentation::getRayleighDampingMass() const
{
	return m_rayleighDamping.massCoefficient;
}

void FemRepresentation::setRayleighDampingStiffness(double stiffnessCoef)
{
	m_rayleighDamping.stiffnessCoefficient = stiffnessCoef;
}

void FemRepresentation::setRayleighDampingMass(double massCoef)
{
	m_rayleighDamping.massCoefficient = massCoef;
}

void FemRepresentation::beforeUpdate(double dt)
{
	DeformableRepresentation::beforeUpdate(dt);

	SURGSIM_ASSERT(getNumFemElements())
			<< "No fem element specified yet, call addFemElement() prior to running the simulation";
	SURGSIM_ASSERT(getNumDof())
			<<	"State has not been initialized yet, call setInitialState() prior to running the simulation";
}

SurgSim::Math::Vector& FemRepresentation::computeF(const SurgSim::Math::OdeState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	addGravityForce(&m_f, state);
	addRayleighDampingForce(&m_f, state);
	addFemElementsForce(&m_f, state);

	// Add external generalized force
	if (m_hasExternalGeneralizedForce)
	{
		m_f += m_externalGeneralizedForce;
	}

	return m_f;
}

const SurgSim::Math::SparseMatrix& FemRepresentation::computeM(const SurgSim::Math::OdeState& state)
{
	// Make sure the mass matrix has been properly allocated and zeroed out
	m_M.resize(state.getNumDof(), state.getNumDof());

	std::cout << "State has " << state.getNumDof() << " matrix is: " << m_M.rows() << "x" << m_M.cols() <<
			  std::endl;
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addMass(state, &m_M);
	}

	return m_M;
}

const SurgSim::Math::SparseMatrix& FemRepresentation::computeD(const SurgSim::Math::OdeState& state)
{
	const double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const double& rayleighMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated and zeroed out
	m_D.resize(state.getNumDof(), state.getNumDof());

	// D += rayleighMass.M
	if (rayleighMass != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addMass(state, &m_D, rayleighMass);
		}
	}

	// D += rayleighStiffness.K
	if (rayleighStiffness != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addStiffness(state, &m_D, rayleighStiffness);
		}
	}

	// D += FemElements damping matrix
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addDamping(state, &m_D);
	}

	// Add external generalized damping
	if (m_hasExternalGeneralizedForce)
	{
		m_D += m_externalGeneralizedDamping;
	}

	return m_D;
}

const SurgSim::Math::SparseMatrix& FemRepresentation::computeK(const SurgSim::Math::OdeState& state)
{
	// Make sure the stiffness matrix has been properly allocated and zeroed out
	m_K.resize(state.getNumDof(), state.getNumDof());

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addStiffness(state, &m_K);
	}

	// Add external generalized stiffness
	if (m_hasExternalGeneralizedForce)
	{
		m_K += m_externalGeneralizedStiffness;
	}

	return m_K;
}

void FemRepresentation::computeFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector** f,
									SurgSim::Math::SparseMatrix** M, SurgSim::Math::SparseMatrix** D,
									SurgSim::Math::SparseMatrix** K)
{
	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	// Make sure the mass matrix has been properly allocated and zeroed out
	m_M.resize(state.getNumDof(), state.getNumDof());

	// Make sure the damping matrix has been properly allocated and zeroed out
	m_D.resize(state.getNumDof(), state.getNumDof());

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	m_K.resize(state.getNumDof(), state.getNumDof());

	// Add all the FemElement contribution to f, M, D, K
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addFMDK(state, &m_f, &m_M, &m_D, &m_K);
	}

	// Add the Rayleigh damping matrix
	if (m_rayleighDamping.massCoefficient)
	{
		m_D += m_M * m_rayleighDamping.massCoefficient;
	}
	if (m_rayleighDamping.stiffnessCoefficient)
	{
		m_D += m_K * m_rayleighDamping.stiffnessCoefficient;
	}

	// Add the gravity to m_f
	addGravityForce(&m_f, state);

	// Add the Rayleigh damping force to m_f
	addRayleighDampingForce(&m_f, state, true, true);

	// Add external generalized force, stiffness and damping
	if (m_hasExternalGeneralizedForce)
	{
		m_f += m_externalGeneralizedForce;
		m_K += m_externalGeneralizedStiffness;
		m_D += m_externalGeneralizedDamping;
	}

	*f = &m_f;
	*M = &m_M;
	*D = &m_D;
	*K = &m_K;
}

void FemRepresentation::addRayleighDampingForce(
	SurgSim::Math::Vector* force, const SurgSim::Math::OdeState& state,
	bool useGlobalStiffnessMatrix, bool useGlobalMassMatrix, double scale)
{
	// Temporary variables for convenience
	double& rayleighMass = m_rayleighDamping.massCoefficient;
	double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const SurgSim::Math::Vector& v = state.getVelocities();

	// Rayleigh damping mass: F = -rayleighMass.M.v(t)
	if (rayleighMass != 0.0)
	{
		// If we have the mass matrix, we can compute directly F = -rayleighMass.M.v(t)
		if (useGlobalMassMatrix)
		{
			Math::Vector tempForce = (scale * rayleighMass) * (m_M * v);
			*force -= tempForce;
		}
		else
		{
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
			{
				(*femElement)->addMatVec(state, - scale * rayleighMass, 0.0, 0.0, v, force);
			}
		}
	}

	// Rayleigh damping stiffness: F = - rayleighStiffness.K.v(t)
	// K is not diagonal and links all dof of the N connected nodes
	if (rayleighStiffness != 0.0)
	{
		if (useGlobalStiffnessMatrix)
		{
			Math::Vector tempForce = scale * rayleighStiffness * (m_K * v);
			*force -= tempForce;
		}
		else
		{
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
			{
				(*femElement)->addMatVec(state, 0.0, 0.0, - scale * rayleighStiffness, v, force);
			}
		}
	}
}

void FemRepresentation::addFemElementsForce(SurgSim::Math::Vector* force,
		const SurgSim::Math::OdeState& state,
		double scale)
{
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addForce(state, force, scale);
	}
}

void FemRepresentation::addGravityForce(SurgSim::Math::Vector* f,
										const SurgSim::Math::OdeState& state,
										double scale)
{
	using SurgSim::Math::addSubVector;

	SURGSIM_ASSERT(m_massPerNode.size() == state.getNumNodes()) <<
			"Mass per node has not been properly allocated. Did you call Initialize() ?";

	// Prepare a gravity vector of the proper size
	SurgSim::Math::Vector gravitynD = SurgSim::Math::Vector::Zero(getNumDofPerNode());
	gravitynD.segment(0, 3) = getGravity();

	if (isGravityEnabled())
	{
		for (size_t nodeId = 0; nodeId < state.getNumNodes(); nodeId++)
		{
			// F = mg
			addSubVector(gravitynD * (scale * m_massPerNode[nodeId]), nodeId, getNumDofPerNode(), f);
		}
	}
}

} // namespace Physics

} // namespace SurgSim
