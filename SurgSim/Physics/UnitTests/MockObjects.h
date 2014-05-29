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

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::OdeSolver;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

class MockRepresentation : public Representation
{
protected:
	int m_preUpdateCount;
	int m_updateCount;
	int m_postUpdateCount;

public:
	explicit MockRepresentation(const std::string& name = "MockRepresention") :
		Representation(name),
		m_preUpdateCount(0),
		m_updateCount(0),
		m_postUpdateCount(0)
	{}

	virtual ~MockRepresentation()
	{}

	SURGSIM_CLASSNAME(SurgSim::Physics::MockRepresentation);

	virtual RepresentationType getType() const override
	{
		return REPRESENTATION_TYPE_FIXED;
	}

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt)
	{
		m_preUpdateCount++;
	}

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt)
	{
		m_updateCount++;
	}

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt)
	{
		m_postUpdateCount++;
	}

	int getPreUpdateCount() const
	{ return m_preUpdateCount; }

	int getUpdateCount() const
	{ return m_updateCount; }

	int getPostUpdateCount() const
	{ return m_postUpdateCount; }
};

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::MockRepresentation);
}

class MockRigidRepresentation : public RigidRepresentation
{
public:
	MockRigidRepresentation() :
		RigidRepresentation("MockRigidRepresentation")
	{
	}

	// Non constand access to the states
	RigidRepresentationState& getInitialState()
	{
		return m_initialState;
	}
	RigidRepresentationState& getCurrentState()
	{
		return m_currentState;
	}
	RigidRepresentationState& getPreviousState()
	{
		return m_previousState;
	}
};

class MockDeformableRepresentation : public SurgSim::Physics::DeformableRepresentation
{
public:
	explicit MockDeformableRepresentation(const std::string& name = "MockDeformableRepresentation") :
		SurgSim::Physics::DeformableRepresentation(name)
	{
		this->m_numDofPerNode = 3;
		m_F = Vector::LinSpaced(3, 1.0, 3.0);
		m_M = Matrix::Identity(3, 3);
		m_D = Matrix::Identity(3, 3);
		m_K = Matrix::Identity(3, 3);
	}

	/// Query the representation type
	/// \return the RepresentationType for this representation
	/// \note DeformableRepresentation is abstract because there is really no deformable behind this class !
	/// \note For the test, we simply set the type to INVALID
	virtual SurgSim::Physics::RepresentationType getType() const override
	{
		return SurgSim::Physics::REPRESENTATION_TYPE_INVALID;
	}

	SURGSIM_CLASSNAME(SurgSim::Physics::MockDeformableRepresentation);

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	Vector& computeF(const SurgSim::Math::OdeState& state) override
	{
		return m_F;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeM(const SurgSim::Math::OdeState& state) override
	{
		return m_M;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeD(const SurgSim::Math::OdeState& state) override
	{
		return m_D;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeK(const SurgSim::Math::OdeState& state) override
	{
		return m_K;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	void computeFMDK(const SurgSim::Math::OdeState& state,
					 Vector** f, Matrix** M, Matrix** D, Matrix** K) override
	{
		*f = &m_F;
		*M = &m_M;
		*D = &m_D;
		*K = &m_K;
	}

protected:
	void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
						const SurgSim::Math::RigidTransform3d& transform) override
	{
		using SurgSim::Math::setSubVector;
		using SurgSim::Math::getSubVector;

		Vector& x = state->getPositions();
		Vector& v = state->getVelocities();
		for (unsigned int nodeId = 0; nodeId < state->getNumNodes(); nodeId++)
		{
			Vector3d xi = getSubVector(x, nodeId, 3);
			Vector3d xiTransformed = transform * xi;
			setSubVector(xiTransformed, nodeId, 3, &x);

			Vector3d vi = getSubVector(v, nodeId, 3);
			Vector3d viTransformed = transform.linear() * vi;
			setSubVector(viTransformed, nodeId, 3, &v);
		}
	}

	Vector m_F;
	Matrix m_M, m_D, m_K;
};

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::MockDeformableRepresentation);
}

class MockSpring : public SurgSim::Physics::Spring
{
public:
	MockSpring() : SurgSim::Physics::Spring()
	{
		m_F = Vector::LinSpaced(6, 1.0, 6.0);
		m_D = Matrix::Identity(6, 6) * 2.0;
		m_K = Matrix::Identity(6, 6) * 3.0;
	}

	void addNode(unsigned int nodeId)
	{
		this->m_nodeIds.push_back(nodeId);
	}

	virtual void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubVector(scale * m_F, m_nodeIds, 3, F);
	}
	virtual void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* D,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubMatrix(scale * m_D, m_nodeIds, 3, D);
	}
	virtual void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubMatrix(scale * m_K, m_nodeIds, 3, K);
	}
	virtual void addFDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* f,
		SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K) override
	{
		addForce(state, f);
		addDamping(state, D);
		addStiffness(state, K);
	}
	virtual void addMatVec(const SurgSim::Math::OdeState& state, double alphaD, double alphaK,
		const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) override
	{
		Vector xLocal(3 * m_nodeIds.size()), fLocal;
		SurgSim::Math::getSubVector(x, m_nodeIds, 3, &xLocal);
		fLocal = (alphaD * m_D + alphaK * m_K) * xLocal;
		SurgSim::Math::addSubVector(fLocal, m_nodeIds, 3, F);
	}

private:
	Vector m_F;
	Matrix m_D, m_K;
};

class MockMassSpring : public SurgSim::Physics::MassSpringRepresentation
{
public:
	explicit MockMassSpring(const std::string& name,
		const SurgSim::Math::RigidTransform3d& pose,
		unsigned int numNodes, std::vector<unsigned int> nodeBoundaryConditions,
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
		for (unsigned int i = 0; i < numNodes; i++)
		{
			Vector3d p(static_cast<double>(i)/static_cast<double>(numNodes), 0, 0);
			setSubVector(p, i, 3, &state->getPositions());
			addMass(std::make_shared<Mass>(totalMass / numNodes));
		}
		for (auto bc = std::begin(nodeBoundaryConditions); bc != std::end(nodeBoundaryConditions); bc++)
		{
			state->addBoundaryCondition(*bc);
		}
		for (unsigned int i = 0; i < numNodes - 1; i++)
		{
			std::shared_ptr<LinearSpring> spring = std::make_shared<LinearSpring>(i, i+1);
			spring->setDamping(springDamping);
			spring->setStiffness(springStiffness);
			const Vector3d& xi = getSubVector(state->getPositions(), i, 3);
			const Vector3d& xj = getSubVector(state->getPositions(), i+1, 3);
			spring->setRestLength( (xj - xi).norm() );
			addSpring(spring);
		}
		setInitialState(state);
		setIntegrationScheme(integrationScheme);
		setRayleighDampingMass(rayleighDampingMass);
		setRayleighDampingStiffness(rayleighDampingStiffness);
	}

	virtual ~MockMassSpring()
	{}

	const Vector3d& getGravityVector() const { return getGravity(); }
};

class MockFemElement : public FemElement
{
public:
	MockFemElement() : FemElement(), m_isInitialized(false)
	{
		setNumDofPerNode(3);
	}

	void addNode(unsigned int nodeId)
	{
		this->m_nodeIds.push_back(nodeId);
	}

	virtual double getVolume(const SurgSim::Math::OdeState& state) const override
	{ return 1; }
	virtual void addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubVector(scale * m_F, m_nodeIds, 3, F);
	}
	virtual void addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* M,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubMatrix(scale * m_M, m_nodeIds, 3, M);
	}
	virtual void addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* D,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubMatrix(scale * m_D, m_nodeIds, 3, D);
	}
	virtual void addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K,
		double scale = 1.0) override
	{
		SurgSim::Math::addSubMatrix(scale * m_K, m_nodeIds, 3, K);
	}
	virtual void addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* f,
		SurgSim::Math::Matrix* M, SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K) override
	{
		addForce(state, f);
		addMass(state, M);
		addDamping(state, D);
		addStiffness(state, K);
	}
	virtual void addMatVec(const SurgSim::Math::OdeState& state, double alphaM, double alphaD, double alphaK,
		const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) override
	{
		Vector xLocal(3 * m_nodeIds.size()), fLocal;
		SurgSim::Math::getSubVector(x, m_nodeIds, 3, &xLocal);
		fLocal = (alphaM * m_M + alphaD * m_D + alphaK * m_K) * xLocal;
		SurgSim::Math::addSubVector(fLocal, m_nodeIds, 3, F);
	}
	virtual SurgSim::Math::Vector computeCartesianCoordinate(const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector &barycentricCoordinate) const override
	{ return SurgSim::Math::Vector3d::Zero(); }
	virtual SurgSim::Math::Vector computeNaturalCoordinate(
		const SurgSim::Math::OdeState& state,
		const SurgSim::Math::Vector &globalCoordinate) const override
	{ return SurgSim::Math::Vector3d::Zero(); }

	virtual void initialize(const SurgSim::Math::OdeState& state) override
	{
		FemElement::initialize(state);
		const int numDof = 3 * m_nodeIds.size();
		m_F = Vector::LinSpaced(numDof, 1.0, static_cast<double>(numDof));
		m_M = Matrix::Identity(numDof, numDof) * 1.0;
		m_D = Matrix::Identity(numDof, numDof) * 2.0;
		m_K = Matrix::Identity(numDof, numDof) * 3.0;
		m_isInitialized = true;
	}

	bool isInitialized() const
	{
		return m_isInitialized;
	}

private:
	Vector m_F;
	Matrix m_M, m_D, m_K;
	bool m_isInitialized;
};

class InvalidMockFemElement : public MockFemElement
{
public:
	virtual bool update(const SurgSim::Math::OdeState& state) override
	{
		return false;
	}
};

// Concrete class for testing
class MockFemRepresentation : public FemRepresentation
{
public:
	/// Constructor
	/// \param name The name of the FemRepresentation
	explicit MockFemRepresentation(const std::string& name) : FemRepresentation(name)
	{
		this->m_numDofPerNode = 3;
	}

	/// Destructor
	virtual ~MockFemRepresentation(){}

	/// Query the representation type
	/// \return the RepresentationType for this representation
	virtual RepresentationType getType() const override
	{
		return REPRESENTATION_TYPE_INVALID;
	}

	std::shared_ptr<OdeSolver> getOdeSolver() const
	{
		return this->m_odeSolver;
	}

	const std::vector<double>& getMassPerNode() const
	{
		return m_massPerNode;
	}

protected:
	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
		const SurgSim::Math::RigidTransform3d& transform) override
	{
	}
};

class MockFem1DRepresentation : public SurgSim::Physics::Fem1DRepresentation
{
public:
	explicit MockFem1DRepresentation(const std::string& name)
		: SurgSim::Physics::Fem1DRepresentation(name)
	{}

	const std::shared_ptr<SurgSim::Math::OdeSolver> getOdeSolver() const
	{
		return this->m_odeSolver;
	}
};

class MockFixedConstraintBilateral3D : public ConstraintImplementation
{
public:
	MockFixedConstraintBilateral3D() : ConstraintImplementation(){}
	virtual ~MockFixedConstraintBilateral3D(){}

	virtual SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override
	{
		return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
	}

	virtual RepresentationType getRepresentationType() const override
	{
		return REPRESENTATION_TYPE_FIXED;
	}

private:
	virtual unsigned int doGetNumDof() const override
	{
		return 3;
	}

	virtual void doBuild(double dt,
				const ConstraintData& data,
				const std::shared_ptr<Localization>& localization,
				MlcpPhysicsProblem* mlcp,
				unsigned int indexOfRepresentation,
				unsigned int indexOfConstraint,
				ConstraintSideSign sign) override
	{}
};

class MockRigidConstraintBilateral3D : public ConstraintImplementation
{
public:
	MockRigidConstraintBilateral3D() : ConstraintImplementation(){}
	virtual ~MockRigidConstraintBilateral3D(){}

	virtual SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const override
	{
		return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
	}

	virtual RepresentationType getRepresentationType() const override
	{
		return REPRESENTATION_TYPE_RIGID;
	}

private:
	virtual unsigned int doGetNumDof() const override
	{
		return 3;
	}

	virtual void doBuild(double dt,
				const ConstraintData& data,
				const std::shared_ptr<Localization>& localization,
				MlcpPhysicsProblem* mlcp,
				unsigned int indexOfRepresentation,
				unsigned int indexOfConstraint,
				ConstraintSideSign sign) override
	{}
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
	MockLocalization() : Localization()
	{
	}

	explicit MockLocalization(std::shared_ptr<Representation> representation) : Localization(representation)
	{
	}

private:
	/// Calculates the global position of this localization
	/// \param time The time in [0..1] at which the position should be calculated
	/// \return The global position of the localization at the requested time
	/// \note time can useful when dealing with CCD
	virtual SurgSim::Math::Vector3d doCalculatePosition(double time)
	{
		return SurgSim::Math::Vector3d::Zero();
	}
};

class MockConstraintImplementation : public ConstraintImplementation
{
public:
	virtual SurgSim::Math::MlcpConstraintType getMlcpConstraintType() const
	{
		return SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;
	}

	virtual RepresentationType getRepresentationType() const
	{
		return SurgSim::Physics::REPRESENTATION_TYPE_FIXED;
	}

private:
	virtual unsigned int doGetNumDof() const
	{
		return 1;
	}

	virtual void doBuild(double dt,
						 const ConstraintData& data,
						 const std::shared_ptr<Localization>& localization,
						 MlcpPhysicsProblem* mlcp,
						 unsigned int indexOfRepresentation,
						 unsigned int indexOfConstraint,
						 ConstraintSideSign sign)
	{
	}
};

class MockVirtualToolCoupler : public VirtualToolCoupler
{
public:
	MockVirtualToolCoupler() :
		VirtualToolCoupler("Mock Virtual Tool Coupler")
	{
	}

	double getLinearStiffness() const
	{
		return m_linearStiffness.getValue();
	}

	double getLinearDamping() const
	{
		return m_linearDamping.getValue();
	}

	double getAngularStiffness() const
	{
		return m_angularStiffness.getValue();
	}

	double getAngularDamping() const
	{
		return m_angularDamping.getValue();
	}

};

inline std::shared_ptr<Constraint> makeMockConstraint(std::shared_ptr<MockRepresentation> firstRepresentation,
													  std::shared_ptr<MockRepresentation> secondRepresentation)
{
	return std::make_shared<Constraint>(std::make_shared<ConstraintData>(),
										std::make_shared<MockConstraintImplementation>(),
										std::make_shared<MockLocalization>(firstRepresentation),
										std::make_shared<MockConstraintImplementation>(),
										std::make_shared<MockLocalization>(secondRepresentation));
}

}; // Physics
}; // SurgSim

#endif
