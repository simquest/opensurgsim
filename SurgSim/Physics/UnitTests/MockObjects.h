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

#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::OdeSolver;

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
	MockRepresentation() :
		Representation("MockRepresentation"), m_preUpdateCount(0), m_updateCount(0), m_postUpdateCount(0)
	{}

	virtual ~MockRepresentation()
	{}

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

class MockFemElement : public FemElement
{
public:
	MockFemElement() : FemElement()
	{
		setNumDofPerNode(3);
	}

	void addNode(unsigned int nodeId)
	{
		this->m_nodeIds.push_back(nodeId);
	}

	virtual double getVolume(const DeformableRepresentationState& state) const override
	{ return 1; }
	virtual void addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F,
		double scale = 1.0) override
	{}
	virtual void addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M,
		double scale = 1.0) override
	{}
	virtual void addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D,
		double scale = 1.0) override
	{}
	virtual void addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K,
		double scale = 1.0) override
	{}
	virtual void addFMDK(const DeformableRepresentationState& state, SurgSim::Math::Vector* f,
		SurgSim::Math::Matrix* M, SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K) override
	{}
	virtual void addMatVec(const DeformableRepresentationState& state, double alphaM, double alphaD, double alphaK,
		const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F) override
	{}
	virtual bool isValidCoordinate(const SurgSim::Math::Vector &naturalCoordinate) const override
	{ return true; }
	virtual SurgSim::Math::Vector computeCartesianCoordinate(
		const DeformableRepresentationState& state,
		const SurgSim::Math::Vector &barycentricCoordinate) const override
	{ return SurgSim::Math::Vector3d::Zero(); }
};

// Concrete class for testing
class MockFemRepresentation : public
	FemRepresentation<Matrix, Matrix, Matrix, Matrix>
{
public:
	/// Constructor
	/// \param name The name of the FemRepresentation
	explicit MockFemRepresentation(const std::string& name) :
	FemRepresentation<Matrix, Matrix, Matrix, Matrix>(name)
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

	std::shared_ptr<OdeSolver<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix>>
		getOdeSolver() const
	{
		return this->m_odeSolver;
	}

protected:
	/// Transform a state using a given transformation
	/// \param[in,out] state The state to be transformed
	/// \param transform The transformation to apply
	virtual void transformState(std::shared_ptr<DeformableRepresentationState> state,
		const SurgSim::Math::RigidTransform3d& transform) override
	{
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
