// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINTDATA_H
#define SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINTDATA_H

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/RigidRepresentationBase.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

/// Class for rotation vector constraint data between a rigid/fixed representation and Fem1d beam being controlled.
///
/// It considers the Fem1D rotational dof (beamRotationVector) to be the only variable to account for.
/// The equation to verify is rigidR * rigidRAtGrasp^-1 = beamR * beamRAtGrasp^-1
/// where R denotes the current prefixed object 3x3 rotation matrix
///       RAtGrasp is the 3x3 rotation matrix of the prefixed object at the time of the constraint creation
/// and beamR is decomposed into the initial rotation beamR0 and the current rotation given by the rotational dof
/// rigidR * rigidRAtGrasp^-1 = R(beamRotationVector) * beamR0 * beamRAtGrasp^-1
/// rigidR * rigidRAtGrasp^1 * beamRAtGrasp * beamR0^-1 = R(beamRotationVector)
/// rotVec(rigidR * rigidRAtGrasp^1 * beamRAtGrasp * beamR0^-1) = beamRotationVector
class RotationVectorRigidFem1DConstraintData : public ConstraintData
{
public:
	/// Default constructor
	RotationVectorRigidFem1DConstraintData() :
		ConstraintData(), m_initialized(false)
	{
	}

	/// Destructor
	virtual ~RotationVectorRigidFem1DConstraintData()
	{
	}

	/// Set the rigid/fixed object part that will control the fem1d
	/// \param rigid The rigid base representation (either a RigidRepresentation or FixedRepresentation)
	/// \param rigidRAtGrasp The rigid rotation at the time of the constraint creation
	void setRigidOrFixedRotation(std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> rigid,
		const SurgSim::Math::Matrix33d& rigidRAtGrasp)
	{
		m_rigidRAtGrasp = rigidRAtGrasp;
		m_rigid = rigid;
	}

	/// Set the fem1d object part
	/// \param beams The Fem1DRepresentation to be controlled by the rigid/fixed representation orientation
	/// \param beamId The beam id that is going to be controlled by the rigid/fixed representation orientation
	void setFem1DRotation(std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> beams, int beamId)
	{
		m_beams = beams;
		m_beamId = beamId;
	}

	/// \return The current rotation vector that should correspond to the beam rotation vector
	/// i.e. rotVec(rigidR * rigidRAtGrasp^1 * beamRAtGrasp * beamR0^-1)
	SurgSim::Math::Vector3d getCurrentRotationVector()
	{
		SURGSIM_ASSERT(nullptr != m_rigid) << "Did you call setRigidOrFixedRotation prior to using this class ?";
		SURGSIM_ASSERT(nullptr != m_beams) << "Did you call setFem1DRotation prior to using this class ?";
		SURGSIM_ASSERT(m_beams->getNumFemElements() > m_beamId) << "The beam id " << m_beamId
			<< " does not exists, the fem1d has " << m_beams->getNumFemElements()  << " beams";

		SurgSim::Math::Matrix33d rigidR = m_rigid->getCurrentState().getPose().linear();
		if (!m_initialized)
		{
			auto beam = std::dynamic_pointer_cast<SurgSim::Physics::Fem1DElementBeam>(m_beams->getFemElement(m_beamId));
			m_beamR0 = beam->getInitialRotation();
			m_beamRAtGrasp = m_beamR0;
			m_initialized = true;
		}
		SurgSim::Math::Matrix33d rotation = rigidR * m_rigidRAtGrasp.inverse() * m_beamRAtGrasp * m_beamR0.inverse();
		Eigen::AngleAxisd aa(rotation);
		return aa.angle() * aa.axis();
	}

private:
	/// Rigid/Fixed representation
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> m_rigid;
	/// Rigid/Fixed rotation information at the time of the constraint creation
	SurgSim::Math::Matrix33d m_rigidRAtGrasp;

	/// Fem1D representation
	std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> m_beams;
	/// The beam in the fem1d representation that is constrained
	int m_beamId;
	/// The beam initial rotation and rotation at the time of the constraint creation
	SurgSim::Math::Matrix33d m_beamR0, m_beamRAtGrasp;
	/// Flag to keep track of the beam rotation information calculation
	bool m_initialized;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_ROTATIONVECTORCONSTRAINTDATA_H
