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

#ifndef SURGSIM_PHYSICS_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_PHYSICS_UNITTESTS_MOCKOBJECTS_H

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/Computation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/MassSpringRepresentationLocalization.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

namespace SurgSim
{
namespace Physics
{

using SurgSim::Math::Matrix;
using SurgSim::Math::OdeSolver;
using SurgSim::Math::OdeState;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

class MockRepresentation : public Representation
{
protected:
	int m_preUpdateCount;
	int m_updateCount;
	int m_postUpdateCount;

public:
	explicit MockRepresentation(const std::string& name = "MockRepresention");

	virtual ~MockRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::MockRepresentation);

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt) override;

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	void update(double dt) override;

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	void afterUpdate(double dt) override;

	int getPreUpdateCount() const;

	int getUpdateCount() const;

	int getPostUpdateCount() const;

	std::shared_ptr<Localization> createLocalization(const SurgSim::DataStructures::Location& location) override;
};

class MockRigidRepresentation : public RigidRepresentation
{
public:
	MockRigidRepresentation();

	// Non constant access to the states
	RigidRepresentationState& getInitialState();
	RigidRepresentationState& getCurrentState();
	RigidRepresentationState& getPreviousState();
};

class MockFixedRepresentation : public FixedRepresentation
{
public:
	MockFixedRepresentation();

	// Non constant access to the states
	RigidRepresentationState& getInitialState();
	RigidRepresentationState& getCurrentState();
	RigidRepresentationState& getPreviousState();
};

class MockDeformableRepresentationLocalization : public SurgSim::Physics::Localization
{
public:
	MockDeformableRepresentationLocalization(){}

	explicit MockDeformableRepresentationLocalization(std::shared_ptr<Representation> representation) : Localization()
	{
		setRepresentation(representation);
	}

	virtual ~MockDeformableRepresentationLocalization(){}

	void setLocalNode(size_t nodeID){ m_nodeID = nodeID; }

	const size_t& getLocalNode() const { return m_nodeID; }

	bool isValidRepresentation(std::shared_ptr<Representation> representation) override
	{
		std::shared_ptr<DeformableRepresentation> defRepresentation =
			std::dynamic_pointer_cast<DeformableRepresentation>(representation);

		// Allows to reset the representation to nullptr ...
		return (defRepresentation != nullptr || representation == nullptr);
	}

private:
	SurgSim::Math::Vector3d doCalculatePosition(double time) override
	{
		std::shared_ptr<DeformableRepresentation> defRepresentation =
			std::static_pointer_cast<DeformableRepresentation>(getRepresentation());

		SURGSIM_ASSERT(defRepresentation != nullptr) << "Deformable Representation is null, it was probably not" <<
			" initialized";
		SURGSIM_ASSERT((0.0 <= time) && (time <= 1.0)) << "Time must be between 0.0 and 1.0 inclusive";

		const SurgSim::Math::Vector3d& currentPoint  = defRepresentation->getCurrentState()->getPosition(m_nodeID);
		const SurgSim::Math::Vector3d& previousPoint = defRepresentation->getPreviousState()->getPosition(m_nodeID);

		return SurgSim::Math::interpolate(previousPoint, currentPoint, time);
	}

	size_t m_nodeID;
};

class MockDeformableRepresentation : public SurgSim::Physics::DeformableRepresentation
{
public:
	explicit MockDeformableRepresentation(const std::string& name = "MockDeformableRepresentation");

	SURGSIM_CLASSNAME(SurgSim::Physics::MockDeformableRepresentation);

	void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const SurgSim::Math::Vector& generalizedForce,
		const SurgSim::Math::Matrix& K,
		const SurgSim::Math::Matrix& D) override;

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	Vector& computeF(const OdeState& state) override;

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeM(const OdeState& state) override;

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeD(const OdeState& state) override;

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeK(const OdeState& state) override;

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	void computeFMDK(const OdeState& state, Vector** f, Matrix** M, Matrix** D, Matrix** K) override;

protected:
	void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
			const SurgSim::Math::RigidTransform3d& transform) override;

	Vector m_F;
	Matrix m_M, m_D, m_K;
};

class MockSpring : public SurgSim::Physics::Spring
{
public:
	MockSpring();

	void addNode(size_t nodeId);

	void addForce(const OdeState& state, Vector* F, double scale = 1.0) override;
	void addDamping(const OdeState& state, Matrix* D, double scale = 1.0) override;
	void addStiffness(const OdeState& state, Matrix* K, double scale = 1.0) override;
	void addFDK(const OdeState& state, Vector* f, Matrix* D, Matrix* K) override;
	void addMatVec(const OdeState& state, double alphaD, double alphaK, const Vector& x, Vector* F) override;

private:
	Vector m_F;
	Matrix m_D, m_K;
};

class MockMassSpring : public SurgSim::Physics::MassSpringRepresentation
{
public:
	MockMassSpring() : MassSpringRepresentation("MassSpring")
	{
	}

	MockMassSpring(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose,
		size_t numNodes, std::vector<size_t> nodeBoundaryConditions,
		double totalMass,
		double rayleighDampingMass, double rayleighDampingStiffness,
		double springStiffness, double springDamping,
		SurgSim::Math::IntegrationScheme integrationScheme);

	virtual ~MockMassSpring();

	const Vector3d& getGravityVector() const;

	const SurgSim::Math::Vector& getExternalForce() const { return m_externalGeneralizedForce; }
	const SurgSim::Math::Matrix& getExternalStiffness() const { return m_externalGeneralizedStiffness; }
	const SurgSim::Math::Matrix& getExternalDamping() const { return m_externalGeneralizedDamping; }
};

class MockFemElement : public FemElement
{
public:
	MockFemElement();

	void addNode(size_t nodeId);

	double getVolume(const OdeState& state) const override;
	void addForce(const OdeState& state, Vector* F, double scale = 1.0) override;
	void addMass(const OdeState& state, Matrix* M, double scale = 1.0) override;
	void addDamping(const OdeState& state, Matrix* D, double scale = 1.0) override;
	void addStiffness(const OdeState& state, Matrix* K, double scale = 1.0) override;
	void addFMDK(const OdeState& state, Vector* f, Matrix* M, Matrix* D, Matrix* K) override;
	void addMatVec(const OdeState& state, double alphaM, double alphaD, double alphaK, const Vector& x, Vector* F)
		override;
	Vector computeCartesianCoordinate(const OdeState& state, const Vector &barycentricCoordinate) const override;
	Vector computeNaturalCoordinate(const SurgSim::Math::OdeState& state, const Vector &globalCoordinate) const
		override;

	void initialize(const SurgSim::Math::OdeState& state) override;

	bool isInitialized() const;

private:
	Vector m_F;
	Matrix m_M, m_D, m_K;
	bool m_isInitialized;
};

// Concrete class for testing
class MockFemRepresentation : public FemRepresentation
{
public:
	/// Constructor
	/// \param name The name of the FemRepresentation
	explicit MockFemRepresentation(const std::string& name);

	/// Destructor
	virtual ~MockFemRepresentation();

	void addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
			const SurgSim::Math::Vector& generalizedForce, const SurgSim::Math::Matrix& K,
			const SurgSim::Math::Matrix& D) override;

	std::shared_ptr<FemPlyReaderDelegate> getDelegate() override;

	std::shared_ptr<OdeSolver> getOdeSolver() const;

	const std::vector<double>& getMassPerNode() const;

protected:
	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	void transformState(std::shared_ptr<OdeState> state, const RigidTransform3d& transform) override;
};

class MockFemRepresentationValidComplianceWarping : public MockFemRepresentation
{
public:
	explicit MockFemRepresentationValidComplianceWarping(const std::string& name) : MockFemRepresentation(name)
	{}

protected:
	SurgSim::Math::Matrix getNodeTransformation(const SurgSim::Math::OdeState& state, size_t nodeId) override;
};

class MockFem1DRepresentation : public SurgSim::Physics::Fem1DRepresentation
{
public:
	explicit MockFem1DRepresentation(const std::string& name);

	const std::shared_ptr<OdeSolver> getOdeSolver() const;
};

class MockFixedConstraintBilateral3D : public ConstraintImplementation
{
public:
	MockFixedConstraintBilateral3D();
	virtual ~MockFixedConstraintBilateral3D();

	SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override;

private:
	size_t doGetNumDof() const override;

	void doBuild(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign) override;
};

class MockRigidConstraintBilateral3D : public ConstraintImplementation
{
public:
	MockRigidConstraintBilateral3D();
	virtual ~MockRigidConstraintBilateral3D();

	SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override;

private:
	size_t doGetNumDof() const override;

	void doBuild(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign) override;
};

template <class Base>
class MockDescendent : public Base
{
public:
	MockDescendent() : Base() {}
	explicit MockDescendent(const std::string &name) : Base(name) {}
};

class MockLocalization : public Localization
{
public:
	MockLocalization();

	explicit MockLocalization(std::shared_ptr<Representation> representation);

private:
	/// Calculates the global position of this localization
	/// \param time The time in [0..1] at which the position should be calculated
	/// \return The global position of the localization at the requested time
	/// \note time can useful when dealing with CCD
	SurgSim::Math::Vector3d doCalculatePosition(double time) override;
};

class MockConstraintImplementation : public ConstraintImplementation
{
public:
	SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override;

private:
	size_t doGetNumDof() const override;

	virtual void doBuild(double dt,
		const ConstraintData& data,
		const std::shared_ptr<Localization>& localization,
		MlcpPhysicsProblem* mlcp,
		size_t indexOfRepresentation,
		size_t indexOfConstraint,
		ConstraintSideSign sign);
};

class MockVirtualToolCoupler : public VirtualToolCoupler
{
public:
	MockVirtualToolCoupler();

	const SurgSim::DataStructures::OptionalValue<double>& getOptionalLinearStiffness() const;
	const SurgSim::DataStructures::OptionalValue<double>& getOptionalLinearDamping() const;
	const SurgSim::DataStructures::OptionalValue<double>& getOptionalAngularStiffness() const;
	const SurgSim::DataStructures::OptionalValue<double>& getOptionalAngularDamping() const;
	const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>& getOptionalAttachmentPoint() const;

	void setOptionalLinearStiffness(const SurgSim::DataStructures::OptionalValue<double>& val);
	void setOptionalLinearDamping(const SurgSim::DataStructures::OptionalValue<double>& val);
	void setOptionalAngularStiffness(const SurgSim::DataStructures::OptionalValue<double>& val);
	void setOptionalAngularDamping(const SurgSim::DataStructures::OptionalValue<double>& val);
	void setOptionalAttachmentPoint(const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>& val);

	const SurgSim::DataStructures::DataGroup& getOutputData() const;
};

inline std::shared_ptr<Constraint> makeMockConstraint(std::shared_ptr<MockRepresentation> firstRepresentation,
	std::shared_ptr<MockRepresentation> secondRepresentation)
{
	using SurgSim::DataStructures::Location;

	static auto type = (new MockConstraintImplementation())->getMlcpConstraintType();
	if (firstRepresentation->getConstraintImplementation(type) == nullptr)
	{
		ConstraintImplementation::getFactory().addImplementation(typeid(MockRepresentation),
			std::make_shared<MockConstraintImplementation>());
	}

	return std::make_shared<Constraint>(type, std::make_shared<ConstraintData>(),
		firstRepresentation, Location(),
		secondRepresentation, Location());
}

/// Class to represent a mock collision representation to test if update gets called from the Computation.
class MockCollisionRepresentation : public SurgSim::Collision::Representation
{
public:
	/// Default constructor
	/// \param name The name of the collision representation.
	explicit MockCollisionRepresentation(const std::string& name);

	int getShapeType() const override;
	const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;
	void update(const double& dt) override;

	/// \return The number of times update method has been invoked.
	int getNumberOfTimesUpdateCalled() const;

private:
	/// Number of times update method has been invoked.
	int m_numberOfTimesUpdateCalled;
};

class MockComputation : public Computation
{
public:
	explicit MockComputation(bool doCopyState = false);

protected:
	std::shared_ptr<PhysicsManagerState> doUpdate(const double& dt,
												  const std::shared_ptr<PhysicsManagerState>& state) override;
};

}; // Physics
}; // SurgSim

#endif // SURGSIM_PHYSICS_UNITTESTS_MOCKOBJECTS_H
