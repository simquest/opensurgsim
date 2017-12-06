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
#include "SurgSim/Framework/ThreadPool.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemPlyReaderDelegate.h"
#include "SurgSim/Physics/FemRepresentation.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::OdeState;
using SurgSim::Math::SparseMatrix;

namespace SurgSim
{

namespace Physics
{

FemRepresentation::FemRepresentation(const std::string& name) :
	DeformableRepresentation(name),
	m_useMassLumping(false),
	m_useComplianceWarping(false),
	m_isComplianceWarpingSynchronous(true),
	m_isInitialComplianceMatrixComputed(false)
{
	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(FemRepresentation, bool, ComplianceWarping,
									  getComplianceWarping, setComplianceWarping);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(FemRepresentation, bool, MassLumping,
									  getMassLumping, setMassLumping);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(FemRepresentation, std::string, FemElementType,
									  getFemElementType, setFemElementType);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(FemRepresentation, double, RayleighDampingMass,
									  getRayleighDampingMass, setRayleighDampingMass);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(FemRepresentation, double, RayleighDampingStiffness,
									  getRayleighDampingStiffness, setRayleighDampingStiffness);

}


FemRepresentation::~FemRepresentation()
{
	//if (m_task.valid())
	//{
	//	m_task.get();
	//}
}

void FemRepresentation::setFemElementType(const std::string& type)
{
	SURGSIM_ASSERT(!isInitialized()) << "You must set the FemElement type before initializing";
	m_femElementType = type;
}

const std::string& FemRepresentation::getFemElementType() const
{
	return m_femElementType;
}

bool FemRepresentation::doInitialize()
{
	// DeformableRepresentation::doInitialize will
	// 1) assert if initial state is not set
	// 2) transform m_initialState properly with the initial pose
	// => FemElement::initialize(m_initialState) is using the correct transformed state
	if (!DeformableRepresentation::doInitialize())
	{
		return false;
	}

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
	for (auto element = std::cbegin(m_femElements); element != std::cend(m_femElements); ++element)
	{
		double mass = (*element)->getMass(*m_initialState);
		for (auto nodeId = std::cbegin((*element)->getNodeIds()); nodeId != std::cend((*element)->getNodeIds());
			++nodeId)
		{
			m_massPerNode[*nodeId] += mass / (*element)->getNumNodes();
		}
	}

	using Eigen::Index;

	// Precompute the sparsity pattern for the global arrays.
	m_M.resize(static_cast<Index>(getNumDof()), static_cast<Index>(getNumDof()));
	m_D.resize(static_cast<Index>(getNumDof()), static_cast<Index>(getNumDof()));
	m_K.resize(static_cast<Index>(getNumDof()), static_cast<Index>(getNumDof()));
	for (auto femElement = std::cbegin(m_femElements); femElement != std::cend(m_femElements); ++femElement)
	{
		Math::Matrix block = Math::Matrix::Zero(getNumDofPerNode() * (*femElement)->getNumNodes(),
												getNumDofPerNode() * (*femElement)->getNumNodes());
		(*femElement)->assembleMatrixBlocks(block, (*femElement)->getNodeIds(), getNumDofPerNode(), &m_M);
	}
	m_M.makeCompressed();
	m_D = m_K = m_M;

	// If we are using compliance warping for this representation, let's pre-allocate the rotation matrix
	// and pre-define its pattern, so we only access existing elements later on.
	if (m_useComplianceWarping)
	{
		Index numDofPerNode = static_cast<Index>(getNumDofPerNode());
		Index numDof = static_cast<Index>(getNumDof());

		// Rotation matrix allocation and creation of the sparse matrix pattern.
		m_complianceWarpingTransformation.resize(numDof, numDof);
		// n columns with numDofPerNode non-zero elements each
		m_complianceWarpingTransformation.reserve(
			Eigen::Matrix<Index, Eigen::Dynamic, 1>::Constant(numDof, numDofPerNode));

		auto logger = SurgSim::Framework::Logger::getLogger("Physics/FemRepresentation");
		SURGSIM_LOG_IF(numDofPerNode % 3 != 0, logger, SEVERE) << "Using compliance warping with representation " <<
				getName() << " which has " << numDofPerNode << " dof per node (not a factor of 3)";

		// Use a mask of 1 to setup the sparse matrix pattern
		for (Index nodeId = 0; nodeId < static_cast<Index>(m_initialState->getNumNodes()); ++nodeId)
		{
			for (Index i = 0; i < numDofPerNode; ++i)
			{
				for (Index j = 0; j < numDofPerNode; ++j)
				{
					m_complianceWarpingTransformation.insert(nodeId * numDofPerNode + i, nodeId * numDofPerNode + j) =
						1.0;
				}
			}
		}

		m_complianceWarpingTransformation.makeCompressed();
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

void FemRepresentation::update(double dt)
{
	if (!isActive())
	{
		return;
	}

	SURGSIM_ASSERT(m_odeSolver != nullptr) <<
										   "Ode solver has not been set yet. Did you call beforeUpdate() ?";
	SURGSIM_ASSERT(m_initialState != nullptr) <<
			"Initial state has not been set yet. Did you call setInitialState() ?";

	// Solve the ode and compute the requested compliance matrix
	if (getComplianceWarping())
	{
		if (!isInitialComplianceMatrixComputed())
		{
			m_odeSolver->computeMatrices(dt, *m_initialState, true);
			setIsInitialComplianceMatrixComputed(true);

			if (!m_isComplianceWarpingSynchronous)
			{
				auto calculation = [&]()
				{
					return (m_complianceWarpingTransformationForCalculation *
						m_odeSolver->getComplianceMatrix() *
						m_complianceWarpingTransformationForCalculation.transpose()).eval();
				};

				// Then, transform the initial compliance matrix to get the current compliance warping matrix
				m_complianceWarpingMatrix.noalias() = m_complianceWarpingTransformation *
					m_odeSolver->getComplianceMatrix() * m_complianceWarpingTransformation.transpose();

				m_complianceWarpingTransformationForCalculation = m_complianceWarpingTransformation;
				m_task = Framework::Runtime::getThreadPool()->enqueue<Math::Matrix>(calculation);
			}
		}
		m_odeSolver->solve(dt, *m_currentState, m_newState.get(), false);

		// Update the compliance matrix
		updateComplianceMatrix(*m_newState);
	}
	else
	{
		m_odeSolver->solve(dt, *m_currentState, m_newState.get());
	}

	// Back up the current state into the previous state (by swapping)
	m_currentState.swap(m_previousState);
	// Make the new state, the current state (by swapping)
	m_currentState.swap(m_newState);

	if (!m_currentState->isValid())
	{
		SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), DEBUG)
				<< getName() << " deactivated :" << std::endl
				<< "position=(" << m_currentState->getPositions().transpose() << ")" << std::endl
				<< "velocity=(" << m_currentState->getVelocities().transpose() << ")" << std::endl;

		setLocalActive(false);
	}
}

void FemRepresentation::setComplianceWarping(bool useComplianceWarping)
{
	SURGSIM_ASSERT(!isInitialized()) << "Compliance warping cannot be modified once the component is initialized";

	m_useComplianceWarping = useComplianceWarping;
}

bool FemRepresentation::getComplianceWarping() const
{
	return m_useComplianceWarping;
}

void FemRepresentation::setComplianceWarpingSynchronous(bool complianceWarpingSynchronous)
{
	SURGSIM_ASSERT(!isInitialized()) << "Compliance warping cannot be modified once the component is initialized";
	m_isComplianceWarpingSynchronous = complianceWarpingSynchronous;
}

bool FemRepresentation::isComplianceWarpingSynchronous() const
{
	return m_isComplianceWarpingSynchronous;
}


void FemRepresentation::setMassLumping(bool useMassLumping) {
	SURGSIM_ASSERT(!isInitialized()) << "Can't change mass lumping after initialization.";
	m_useMassLumping = useMassLumping;
}

bool FemRepresentation::getMassLumping() const {
	return m_useMassLumping;
}

Math::Matrix FemRepresentation::applyCompliance(const Math::OdeState& state, const Math::Matrix& b)
{
	SURGSIM_ASSERT(m_odeSolver) << "Ode solver not initialized, it should have been initialized on wake-up";

	if (m_useComplianceWarping)
	{
		// Then, update the compliance matrix using compliance warping
		return (m_complianceWarpingTransformation * DeformableRepresentation::applyCompliance(state,
				m_complianceWarpingTransformation.transpose() * b));
	}
	return DeformableRepresentation::applyCompliance(state, b);
}

const SurgSim::Math::Matrix& FemRepresentation::getComplianceMatrix() const
{
	SURGSIM_ASSERT(m_odeSolver) << "Ode solver not initialized, it should have been initialized on wake-up";

	if (m_useComplianceWarping)
	{
		return m_complianceWarpingMatrix;
	}
	return m_odeSolver->getComplianceMatrix();
}

SurgSim::Math::Matrix FemRepresentation::getNodeTransformation(const SurgSim::Math::OdeState& state,
	size_t nodeId) const
{
	SURGSIM_FAILURE() << "Any representation using compliance warping should override this method to provide the " <<
					  "proper nodes transformation";

	return SurgSim::Math::Matrix();
}

void FemRepresentation::calculateComplianceWarpingTransformation(const SurgSim::Math::OdeState& state)
{
	SURGSIM_FAILURE() << "Any representation using compliance warping should override this method to calculate and " <<
		"store the proper compliance warping transformation";
}

void FemRepresentation::updateComplianceMatrix(const SurgSim::Math::OdeState& state)
{
	calculateComplianceWarpingTransformation(state);

	if (m_isComplianceWarpingSynchronous)
	{
		m_complianceWarpingMatrix.noalias() = m_complianceWarpingTransformation *
			m_odeSolver->getComplianceMatrix() * m_complianceWarpingTransformation.transpose();
	}
	else
	{
		auto calculation = [&]()
		{
			return (m_complianceWarpingTransformationForCalculation *
				m_odeSolver->getComplianceMatrix() *
				m_complianceWarpingTransformationForCalculation.transpose()).eval();
		};

		if (m_task.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
		{
			m_complianceWarpingMatrix = m_task.get();
			m_complianceWarpingTransformationForCalculation = m_complianceWarpingTransformation;
			m_task = Framework::Runtime::getThreadPool()->enqueue<Math::Matrix>(calculation);
		}
	}
}

void FemRepresentation::computeF(const SurgSim::Math::OdeState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	addGravityForce(&m_f, state);
	addRayleighDampingForce(&m_f, state, hasK(), hasF());
	addFemElementsForce(&m_f, state);

	// Add external generalized force
	if (m_hasExternalGeneralizedForce)
	{
		m_f += m_externalGeneralizedForce;
	}
}

void FemRepresentation::computeM(const SurgSim::Math::OdeState& state)
{
	// Make sure the mass matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_M);

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addMass(&m_M);
	}
}

void FemRepresentation::computeD(const SurgSim::Math::OdeState& state)
{
	const double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const double& rayleighMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_D);

	// D += rayleighMass.M
	if (rayleighMass != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addMass(&m_D, rayleighMass);
		}
	}

	// D += rayleighStiffness.K
	if (rayleighStiffness != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addStiffness(&m_D, rayleighStiffness);
		}
	}

	// D += FemElements damping matrix
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addDamping(&m_D);
	}

	// Add external generalized damping
	if (m_hasExternalGeneralizedForce)
	{
		m_D += m_externalGeneralizedDamping;
	}
}

void FemRepresentation::computeK(const SurgSim::Math::OdeState& state)
{
	// Make sure the stiffness matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_K);

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addStiffness(&m_K);
	}

	// Add external generalized stiffness
	if (m_hasExternalGeneralizedForce)
	{
		m_K += m_externalGeneralizedStiffness;
	}
}

void FemRepresentation::computeFMDK(const SurgSim::Math::OdeState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	// Make sure the mass matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_M);

	// Make sure the damping matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_D);

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_K);

	// Add all the FemElement contribution to f, M, D, K
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addFMDK(&m_f, &m_M, &m_D, &m_K);
	}

	if (m_useMassLumping == true)
	{
		for (auto i = 0; i < m_M.rows(); i++)
		{
			double sum = m_M.row(i).sum();
			Math::zeroRow(i, &m_M);
			m_M.coeffRef(i, i) = sum;
		}
		m_M.makeCompressed();
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
	// The mass matrix should exist, the stiffness matrix was set above in the addFMDK loop over the FemElements.
	addRayleighDampingForce(&m_f, state, hasK(), true);

	// Add external generalized force, stiffness and damping
	if (m_previousHasExternalGeneralizedForce != m_hasExternalGeneralizedForce)
	{
		setIsInitialComplianceMatrixComputed(false);
	}

	if (m_hasExternalGeneralizedForce)
	{
		m_f += m_externalGeneralizedForce;
		m_K += m_externalGeneralizedStiffness;
		m_D += m_externalGeneralizedDamping;

		if (!m_previousExternalGeneralizedStiffness.isApprox(m_externalGeneralizedStiffness) ||
			!m_previousExternalGeneralizedDamping.isApprox(m_externalGeneralizedDamping))
		{
			setIsInitialComplianceMatrixComputed(false);
		}
	}
}

void FemRepresentation::updateFMDK(const SurgSim::Math::OdeState& state, int options)
{
	// This function updates the matrices needed to calculate F, M, D, K for each element.
	// Note that the relevant matrices are updated only for non-linear elements.
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->updateFMDK(state, options);
	}

// 	#pragma omp parallel for
// 	for (int i = 0; i < m_femElements.size(); ++i)
// 	{
// 		m_femElements[i]->updateFMDK(state, options);
// 	}

	OdeEquation::updateFMDK(state, options);
}

void FemRepresentation::addRayleighDampingForce(
	SurgSim::Math::Vector* force, const SurgSim::Math::OdeState& state,
	bool useGlobalStiffnessMatrix, bool useGlobalMassMatrix, double scale)
{
	// Temporary variables for convenience
	double rayleighMass = m_rayleighDamping.massCoefficient;
	double rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const SurgSim::Math::Vector& v = state.getVelocities();

	SurgSim::Math::Vector extractedX;
	SurgSim::Math::Vector accumulator;

	// #performance HS-20-apr-2017 The structure here is not ideal, I assume if raleighMass and raleighStiffness are
	// both set we should be able to run through 1 calculation rather than doing two separate passes through all the
	// fem elements

	// Rayleigh damping mass: F = -rayleighMass.M.v(t)
	if (rayleighMass != 0.0)
	{
		// If we have the mass matrix, we can compute directly F = -rayleighMass.M.v(t)
		if (useGlobalMassMatrix)
		{
			if (m_useMassLumping)
			{
				force->noalias() -= (scale * rayleighMass) * (m_M.diagonal().cwiseProduct(v));
			}
			else
			{
				force->noalias() -= (scale * rayleighMass) * (m_M * v);
			}
		}
		else
		{
			size_t size = m_femElements[0]->getNumNodes() * m_femElements[0]->getNumDofPerNode();
			extractedX.resize(size);
			accumulator.resize(size);
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
			{
				(*femElement)->addMatVec(- scale * rayleighMass, 0.0, 0.0, v, force, &extractedX, &accumulator);
			}
		}
	}

	// Rayleigh damping stiffness: F = - rayleighStiffness.K.v(t)
	// K is not diagonal and links all dof of the N connected nodes
	if (rayleighStiffness != 0.0)
	{
		if (useGlobalStiffnessMatrix)
		{
			force->noalias() -= scale * rayleighStiffness * (m_K * v);
		}
		else
		{
			size_t size = m_femElements[0]->getNumNodes() * m_femElements[0]->getNumDofPerNode();
			accumulator.resize(size);
			extractedX.resize(size);
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto femElement = std::cbegin(m_femElements); femElement != std::cend(m_femElements); ++femElement)
			{
				(*femElement)->addMatVec(0.0, 0.0, - scale * rayleighStiffness, v, force, &extractedX, &accumulator);
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
		(*femElement)->addForce(force, scale);
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

bool FemRepresentation::isInitialComplianceMatrixComputed() const
{
	return m_isInitialComplianceMatrixComputed;
}

void FemRepresentation::setIsInitialComplianceMatrixComputed(bool flag)
{
	m_isInitialComplianceMatrixComputed = flag;
}

} // namespace Physics

} // namespace SurgSim
