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

#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/FemPlyReaderDelegate.h"

namespace SurgSim
{
namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::MockRepresentation, MockRepresentation);
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::MockDeformableRepresentation,
				 MockDeformableRepresentation);

MockRepresentation::MockRepresentation(const std::string& name) :
	Representation(name),
	m_preUpdateCount(0),
	m_updateCount(0),
	m_postUpdateCount(0)
{
}

MockRepresentation::~MockRepresentation()
{
}

void MockRepresentation::beforeUpdate(double dt)
{
	m_preUpdateCount++;
}

void MockRepresentation::update(double dt)
{
	m_updateCount++;
}

void MockRepresentation::afterUpdate(double dt)
{
	m_postUpdateCount++;
}

int MockRepresentation::getPreUpdateCount() const
{
	return m_preUpdateCount;
}

int MockRepresentation::getUpdateCount() const
{
	return m_updateCount;
}

int MockRepresentation::getPostUpdateCount() const
{
	return m_postUpdateCount;
}

std::shared_ptr<Localization> MockRepresentation::createLocalization(
	const SurgSim::DataStructures::Location& location)
{
	return std::make_shared<MockLocalization>();
}

MockRigidRepresentation::MockRigidRepresentation() : RigidRepresentation("MockRigidRepresentation")
{
}

RigidState& MockRigidRepresentation::getInitialState()
{
	return m_initialState;
}

RigidState& MockRigidRepresentation::getCurrentState()
{
	return m_currentState;
}

RigidState& MockRigidRepresentation::getPreviousState()
{
	return m_previousState;
}

MockFixedRepresentation::MockFixedRepresentation() : FixedRepresentation("MockFixedRepresentation")
{
}

RigidState& MockFixedRepresentation::getInitialState()
{
	return m_initialState;
}

RigidState& MockFixedRepresentation::getCurrentState()
{
	return m_currentState;
}

RigidState& MockFixedRepresentation::getPreviousState()
{
	return m_previousState;
}

MockDeformableRepresentation::MockDeformableRepresentation(const std::string& name) :
	SurgSim::Physics::DeformableRepresentation(name)
{
	this->m_numDofPerNode = 3;
	m_f = Vector::LinSpaced(3, 1.0, 3.0);
	m_M.resize(3, 3);
	m_M.setIdentity();

	m_D.resize(3, 3);
	m_D.setIdentity();

	m_K.resize(3, 3);
	m_K.setIdentity();
}

void MockDeformableRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const Math::Vector& generalizedForce,
		const Math::Matrix& K,
		const Math::Matrix& D)
{
	std::shared_ptr<MockDeformableLocalization> loc =
		std::dynamic_pointer_cast<MockDeformableLocalization>(localization);

	m_externalGeneralizedForce.segment<3>(3 * loc->getLocalNode()) += generalizedForce;
	Math::addSubMatrix(K, static_cast<SparseMatrix::Index>(loc->getLocalNode()),
					   static_cast<SparseMatrix::Index>(loc->getLocalNode()),
					   &m_externalGeneralizedStiffness, true);
	Math::addSubMatrix(D, static_cast<SparseMatrix::Index>(loc->getLocalNode()),
					   static_cast<SparseMatrix::Index>(loc->getLocalNode()),
					   &m_externalGeneralizedDamping, true);
	m_hasExternalGeneralizedForce = true;
}

void MockDeformableRepresentation::computeF(const OdeState& state)
{
}

void MockDeformableRepresentation::computeM(const OdeState& state)
{
}

void MockDeformableRepresentation::computeD(const OdeState& state)
{
}

void MockDeformableRepresentation::computeK(const OdeState& state)
{
}

void MockDeformableRepresentation::computeFMDK(const OdeState& state)
{
}

void MockDeformableRepresentation::transformState(std::shared_ptr<OdeState> state, const RigidTransform3d& transform)
{
	using SurgSim::Math::setSubVector;
	using SurgSim::Math::getSubVector;

	Vector& x = state->getPositions();
	Vector& v = state->getVelocities();
	for (size_t nodeId = 0; nodeId < state->getNumNodes(); nodeId++)
	{
		Vector3d xi = getSubVector(x, nodeId, 3);
		Vector3d xiTransformed = transform * xi;
		setSubVector(xiTransformed, nodeId, 3, &x);

		Vector3d vi = getSubVector(v, nodeId, 3);
		Vector3d viTransformed = transform.linear() * vi;
		setSubVector(viTransformed, nodeId, 3, &v);
	}
}


MockSpring::MockSpring() : SurgSim::Physics::Spring()
{
	m_F = Vector::LinSpaced(6, 1.0, 6.0);

	m_D.resize(6, 6);
	m_D.setIdentity();
	m_D *= 2.0;

	m_K.resize(6, 6);
	m_K.setIdentity();
	m_K *= 3.0;
}

void MockSpring::addNode(size_t nodeId)
{
	this->m_nodeIds.push_back(nodeId);
}

void MockSpring::addForce(const OdeState& state, Vector* F, double scale)
{
	SurgSim::Math::addSubVector(scale * m_F, m_nodeIds, 3, F);
}

void MockSpring::addDamping(const OdeState& state, SparseMatrix* D, double scale)
{
	Matrix scaledDense(m_D.rows(), m_D.cols());
	scaledDense = scale * m_D;

	int index1 = 0;
	for (auto nodeId1 : m_nodeIds)
	{
		int index2 = 0;
		for (auto nodeId2 : m_nodeIds)
		{
			Math::addSubMatrix(scaledDense.block(3 * index1, 3 * index2, 3, 3),
							   static_cast<SparseMatrix::Index>(nodeId1),
							   static_cast<SparseMatrix::Index>(nodeId2), D, false);
			++index2;
		}
		++index1;
	}
}

void MockSpring::addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* K,
							  double scale)
{
	Matrix scaledDense(m_K.rows(), m_K.cols());
	scaledDense = scale * m_K;

	int index1 = 0;
	for (auto nodeId1 : m_nodeIds)
	{
		int index2 = 0;
		for (auto nodeId2 : m_nodeIds)
		{
			Math::addSubMatrix(scaledDense.block(3 * index1, 3 * index2, 3, 3),
							   static_cast<SparseMatrix::Index>(nodeId1),
							   static_cast<SparseMatrix::Index>(nodeId2), K, false);
			++index2;
		}
		++index1;
	}
}

void MockSpring::addFDK(const OdeState& state, Vector* f, SparseMatrix* D, SparseMatrix* K)
{
	addForce(state, f);
	addDamping(state, D);
	addStiffness(state, K);
}

void MockSpring::addMatVec(const OdeState& state, double alphaD, double alphaK,    const Vector& x, Vector* F)
{
	Vector xLocal(3 * m_nodeIds.size()), fLocal;
	SurgSim::Math::getSubVector(x, m_nodeIds, 3, &xLocal);
	fLocal = (alphaD * m_D + alphaK * m_K) * xLocal;
	SurgSim::Math::addSubVector(fLocal, m_nodeIds, 3, F);
}

MockMassSpring::MockMassSpring(const std::string& name,
							   const SurgSim::Math::RigidTransform3d& pose,
							   size_t numNodes, std::vector<size_t> nodeBoundaryConditions,
							   double totalMass,
							   double rayleighDampingMass, double rayleighDampingStiffness,
							   double springStiffness, double springDamping,
							   SurgSim::Math::IntegrationScheme integrationScheme) :
	SurgSim::Physics::MassSpringRepresentation(name)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::setSubVector;
	using SurgSim::Physics::Mass;
	using SurgSim::Physics::LinearSpring;

	// Note: setLocalPose MUST be called before WakeUp to be effective !
	setLocalPose(pose);

	std::shared_ptr<SurgSim::Math::OdeState> state;
	state = std::make_shared<SurgSim::Math::OdeState>();
	state->setNumDof(3, numNodes);
	for (size_t i = 0; i < numNodes; i++)
	{
		Vector3d p(static_cast<double>(i) / static_cast<double>(numNodes), 0, 0);
		setSubVector(p, i, 3, &state->getPositions());
		addMass(std::make_shared<Mass>(totalMass / numNodes));
	}
	for (auto bc = std::begin(nodeBoundaryConditions); bc != std::end(nodeBoundaryConditions); bc++)
	{
		state->addBoundaryCondition(*bc);
	}
	for (size_t i = 0; i < numNodes - 1; i++)
	{
		std::shared_ptr<LinearSpring> spring = std::make_shared<LinearSpring>(i, i + 1);
		spring->setDamping(springDamping);
		spring->setStiffness(springStiffness);
		const Vector3d& xi = getSubVector(state->getPositions(), i, 3);
		const Vector3d& xj = getSubVector(state->getPositions(), i + 1, 3);
		spring->setRestLength((xj - xi).norm());
		addSpring(spring);
	}
	setInitialState(state);
	setIntegrationScheme(integrationScheme);
	setRayleighDampingMass(rayleighDampingMass);
	setRayleighDampingStiffness(rayleighDampingStiffness);
}

MockMassSpring::~MockMassSpring()
{
}

const Vector3d& MockMassSpring::getGravityVector() const
{
	return getGravity();
}

void MockMassSpring::clearFMDK()
{
	m_f.setZero();
	SurgSim::Math::clearMatrix(&m_M);
	SurgSim::Math::clearMatrix(&m_D);
	SurgSim::Math::clearMatrix(&m_K);
}

MockFemElement::MockFemElement() : FemElement()
{
	initializeMembers();
}

MockFemElement::MockFemElement(std::shared_ptr<FemElementStructs::FemElementParameter> elementData) : FemElement()
{
	initializeMembers();
	m_nodeIds.assign(elementData->nodeIds.begin(), elementData->nodeIds.end());
}

void MockFemElement::addNode(size_t nodeId)
{
	this->m_nodeIds.push_back(nodeId);
}

double MockFemElement::getVolume(const OdeState& state) const
{
	return 1;
}

Vector MockFemElement::computeCartesianCoordinate(const OdeState& state, const Vector& barycentricCoordinate) const
{
	return SurgSim::Math::Vector3d::Zero();
}

Vector MockFemElement::computeNaturalCoordinate(const OdeState& state, const Vector& globalCoordinate) const
{
	return SurgSim::Math::Vector3d::Zero();
}

void MockFemElement::initializeMembers()
{
	m_isInitialized = false;
	m_useDamping = true;
	setNumDofPerNode(3);
}

void MockFemElement::doUpdateFMDK(const Math::OdeState& state, int options)
{
}

void MockFemElement::initialize(const OdeState& state)
{
	FemElement::initialize(state);
	const size_t numDof = 3 * m_nodeIds.size();
	m_f = Vector::LinSpaced(numDof, 1.0, static_cast<double>(numDof));
	m_M.setIdentity();

	m_D.setIdentity();
	m_D *= 2.0;

	m_K.setIdentity();
	m_K *= 3.0;

	m_isInitialized = true;
}

bool MockFemElement::isInitialized() const
{
	return m_isInitialized;
}

MockFemRepresentation::MockFemRepresentation(const std::string& name)
	: FemRepresentation(name), m_setInitialStateCalled(false)
{
	this->m_numDofPerNode = 3;
}

MockFemRepresentation::~MockFemRepresentation()
{
}

void MockFemRepresentation::setInitialState(std::shared_ptr<SurgSim::Math::OdeState> initialState)
{
	FemRepresentation::setInitialState(initialState);
	m_setInitialStateCalled = true;
}

void MockFemRepresentation::loadFem(const std::string &filename)
{
}

void MockFemRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const SurgSim::Math::Vector& generalizedForce,
		const SurgSim::Math::Matrix& K,
		const SurgSim::Math::Matrix& D)
{
	std::shared_ptr<MockDeformableLocalization> loc =
		std::dynamic_pointer_cast<MockDeformableLocalization>(localization);

	size_t numDofPerNode = getNumDofPerNode();
	m_externalGeneralizedForce.segment(numDofPerNode * loc->getLocalNode(), numDofPerNode) += generalizedForce;
	SurgSim::Math::addSubMatrix(K, static_cast<SparseMatrix::Index>(loc->getLocalNode()),
								static_cast<SparseMatrix::Index>(loc->getLocalNode()),
								&m_externalGeneralizedStiffness, true);
	SurgSim::Math::addSubMatrix(D, static_cast<SparseMatrix::Index>(loc->getLocalNode()),
								static_cast<SparseMatrix::Index>(loc->getLocalNode()),
								&m_externalGeneralizedDamping, true);
	m_hasExternalGeneralizedForce = true;
}

std::shared_ptr<FemPlyReaderDelegate> MockFemRepresentation::getDelegate()
{
	return nullptr;
}

std::shared_ptr<OdeSolver> MockFemRepresentation::getOdeSolver() const
{
	return this->m_odeSolver;
}

const std::vector<double>& MockFemRepresentation::getMassPerNode() const
{
	return m_massPerNode;
}

void MockFemRepresentation::clearFMDK()
{
	m_f.setZero();
	SurgSim::Math::clearMatrix(&m_M);
	SurgSim::Math::clearMatrix(&m_D);
	SurgSim::Math::clearMatrix(&m_K);
}

void MockFemRepresentation::transformState(std::shared_ptr<OdeState> state, const RigidTransform3d& transform)
{
}

bool MockFemRepresentation::hasSetInitialStateBeenCalled()
{
	return m_setInitialStateCalled;
}

SurgSim::Math::Matrix
MockFemRepresentationValidComplianceWarping::getNodeTransformation(const SurgSim::Math::OdeState& state, size_t nodeId)
{
	return SurgSim::Math::Matrix::Identity(getNumDofPerNode(), getNumDofPerNode());
}

MockFem1DRepresentation::MockFem1DRepresentation(const std::string& name) : SurgSim::Physics::Fem1DRepresentation(name)
{
}

bool MockFem1DRepresentation::doInitialize()
{
	return Fem1DRepresentation::doInitialize();
}

const std::shared_ptr<OdeSolver> MockFem1DRepresentation::getOdeSolver() const
{
	return this->m_odeSolver;
}


MockFixedConstraintFixedPoint::MockFixedConstraintFixedPoint() : ConstraintImplementation()
{
}

MockFixedConstraintFixedPoint::~MockFixedConstraintFixedPoint()
{
}

SurgSim::Physics::ConstraintType MockFixedConstraintFixedPoint::getConstraintType() const
{
	return SurgSim::Physics::FIXED_3DPOINT;
}

size_t MockFixedConstraintFixedPoint::doGetNumDof() const
{
	return 3;
}

void MockFixedConstraintFixedPoint::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 size_t indexOfRepresentation,
											 size_t indexOfConstraint,
											 ConstraintSideSign sign)
{
}

MockRigidConstraintFixedPoint::MockRigidConstraintFixedPoint() : ConstraintImplementation()
{
}

MockRigidConstraintFixedPoint::~MockRigidConstraintFixedPoint()
{
}

SurgSim::Physics::ConstraintType MockRigidConstraintFixedPoint::getConstraintType() const
{
	return SurgSim::Physics::FIXED_3DPOINT;
}

size_t MockRigidConstraintFixedPoint::doGetNumDof() const
{
	return 3;
}

void MockRigidConstraintFixedPoint::doBuild(double dt,
											 const ConstraintData& data,
											 const std::shared_ptr<Localization>& localization,
											 MlcpPhysicsProblem* mlcp,
											 size_t indexOfRepresentation,
											 size_t indexOfConstraint,
											 ConstraintSideSign sign)
{
}


MockLocalization::MockLocalization() : Localization()
{
}

MockLocalization::MockLocalization(std::shared_ptr<Representation> representation) : Localization(representation)
{
}

Vector3d MockLocalization::doCalculatePosition(double time)
{
	return SurgSim::Math::Vector3d::Zero();
}


SurgSim::Physics::ConstraintType MockConstraintImplementation::getConstraintType() const
{
	return SurgSim::Physics::FIXED_3DPOINT;
}

size_t MockConstraintImplementation::doGetNumDof() const
{
	return 1;
}

void MockConstraintImplementation::doBuild(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign)
{
}

MockVirtualToolCoupler::MockVirtualToolCoupler() : VirtualToolCoupler("Mock Virtual Tool Coupler")
{
}

const SurgSim::DataStructures::OptionalValue<double>& MockVirtualToolCoupler::getOptionalLinearStiffness() const
{
	return VirtualToolCoupler::getOptionalLinearStiffness();
}

const SurgSim::DataStructures::OptionalValue<double>& MockVirtualToolCoupler::getOptionalLinearDamping() const
{
	return VirtualToolCoupler::getOptionalLinearDamping();
}

const SurgSim::DataStructures::OptionalValue<double>& MockVirtualToolCoupler::getOptionalAngularStiffness() const
{
	return VirtualToolCoupler::getOptionalAngularStiffness();
}

const SurgSim::DataStructures::OptionalValue<double>& MockVirtualToolCoupler::getOptionalAngularDamping() const
{
	return VirtualToolCoupler::getOptionalAngularDamping();
}

const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>&
MockVirtualToolCoupler::getOptionalAttachmentPoint() const
{
	return VirtualToolCoupler::getOptionalAttachmentPoint();
}

void MockVirtualToolCoupler::setOptionalLinearStiffness(const SurgSim::DataStructures::OptionalValue<double>& val)
{
	VirtualToolCoupler::setOptionalLinearStiffness(val);
}

void MockVirtualToolCoupler::setOptionalLinearDamping(const SurgSim::DataStructures::OptionalValue<double>& val)
{
	VirtualToolCoupler::setOptionalLinearDamping(val);
}

void MockVirtualToolCoupler::setOptionalAngularStiffness(const SurgSim::DataStructures::OptionalValue<double>& val)
{
	VirtualToolCoupler::setOptionalAngularStiffness(val);
}

void MockVirtualToolCoupler::setOptionalAngularDamping(const SurgSim::DataStructures::OptionalValue<double>& val)
{
	VirtualToolCoupler::setOptionalAngularDamping(val);
}

void MockVirtualToolCoupler::setOptionalAttachmentPoint(
	const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>& val)
{
	VirtualToolCoupler::setOptionalAttachmentPoint(val);
}

const SurgSim::DataStructures::DataGroup& MockVirtualToolCoupler::getOutputData() const
{
	return m_outputData;
}

MockCollisionRepresentation::MockCollisionRepresentation(const std::string& name)
	: SurgSim::Collision::Representation(name), m_numberOfTimesUpdateCalled(0)
{}

int MockCollisionRepresentation::getShapeType() const
{
	return -1;
}

const std::shared_ptr<SurgSim::Math::Shape> MockCollisionRepresentation::getShape() const
{
	return nullptr;
}

void MockCollisionRepresentation::update(const double& dt)
{
	++m_numberOfTimesUpdateCalled;
}

/// \return The number of times update method has been invoked.
int MockCollisionRepresentation::getNumberOfTimesUpdateCalled() const
{
	return m_numberOfTimesUpdateCalled;
}

MockComputation::MockComputation(bool doCopyState) : Computation(doCopyState)
{
}

std::shared_ptr<PhysicsManagerState> MockComputation::doUpdate(const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	return state;
}
}; // Physics
}; // SurgSim
