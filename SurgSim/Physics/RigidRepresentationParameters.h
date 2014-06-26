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

#ifndef SURGSIM_PHYSICS_RIGIDREPRESENTATIONPARAMETERS_H
#define SURGSIM_PHYSICS_RIGIDREPRESENTATIONPARAMETERS_H

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{
class Shape;
}

namespace Physics
{

/// This class defines the physical parameters for a rigid body:
/// mass density, mass, mass center, inertia, damping (linear and angular)
/// It also embeds a list of shapes and a designated shape to base the physical
/// parameter calculation on.
/// Usage:
///	1) without a designated shape and simply set the mass/inertia parameters
/// 2) with    a designated shape and set the mass density
/// \note At any moment, you can always overwrite mass/inertia with values
/// \note of your own.
/// \note Setting the mass directly will discard the mass density (reset to 0).
class RigidRepresentationParameters : public SurgSim::Framework::Accessible
{
public:
	/// Default constructor
	RigidRepresentationParameters();

	/// Copy Constructor
	/// \param rhs Right hand side RigidRepresentationParameters used to initialize a new RigidRepresentationParameters
	RigidRepresentationParameters(const RigidRepresentationParameters& rhs);

	/// Copy assignment
	/// \param rhs Right hand side RigidRepresentationParameters from which data are copied.
	/// \note 'm_functors' in base class Accessible is NOT copied.
	RigidRepresentationParameters& operator=(const RigidRepresentationParameters& rhs);

	/// Destructor
	virtual ~RigidRepresentationParameters();

	SURGSIM_CLASSNAME(SurgSim::Physics::RigidRepresentationParameters);

	/// Comparison operator
	/// \param rhs A RigidRepresentationParameters to compare it to
	/// \return True if the 2 parameters set are equals, False otherwise
	bool operator==(const RigidRepresentationParameters& rhs) const;

	/// Comparison operator (for difference)
	/// \param rhs A RigidRepresentationParameters to compare it to
	/// \return False if the 2 parameters set are equals, True otherwise
	bool operator!=(const RigidRepresentationParameters& rhs) const;

	/// Set the mass density of the rigid representation
	/// There is 2 ways to define a rigid representation mass/inertia:
	/// * Set its density and its shape will compute automatically mass/inertia
	/// * Set the mass/inertia directly.
	/// \param rho The density (in Kg.m-3)
	/// \note rho = 0, resets the density and keeps the mass/inertia as is.
	/// \note rho != 0 will overwrite mass/inertia using the given mesh/shape
	/// \note if any.
	void setDensity(double rho);

	/// Get the mass density of the rigid representation
	/// \return The density if it has been provided, 0 otherwise (in Kg.m-3)
	double getDensity() const;

	/// Set the mass of the rigid body
	/// There is 2 ways to define a rigid representation mass/inertia:
	/// * Set its density and its shape will compute automatically mass/inertia
	/// * Set the mass/inertia directly
	/// \param mass The mass (in Kg)
	/// This reset the density to 0 (unknown)
	void setMass(double mass);

	/// Get the mass of the rigid body
	/// \return The mass (in Kg)
	double getMass() const;

	/// Get the mass center of the rigid body
	/// \return The mass center (in local coordinate)
	const SurgSim::Math::Vector3d& getMassCenter() const;

	/// Set the local inertia of the rigid body
	/// \param localInertia The inertia 3x3 symmetric matrix of the object
	/// This overwrites any previously computed/set local inertia
	void setLocalInertia(const SurgSim::Math::Matrix33d& localInertia);

	/// Get the local inertia 3x3 matrix of the rigid body
	/// \return The inertia 3x3 matrix of the object
	const SurgSim::Math::Matrix33d& getLocalInertia() const;

	/// Set the linear damping parameter
	/// \param linearDamping The linear damping parameter (in N.s.m-1)
	void setLinearDamping(double linearDamping);

	/// Get the linear damping parameter
	/// \return The linear damping parameter (in N.s.m-1)
	double getLinearDamping() const;

	/// Set the angular damping parameter
	/// \param angularDamping The angular damping parameter (in N.m.s.rad-1)
	void setAngularDamping(double angularDamping);

	/// Get the angular damping parameter
	/// \return The angular damping parameter (in N.m.s.rad-1)
	double getAngularDamping() const;

	/// Add a shape to this rigid representation
	/// \param shape A shape to add to this rigid representation
	void addShape(const std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Remove a shape from this rigid representation
	/// \param shape The shape to be detached from this rigid representation
	/// \note Does nothing if the shape is not found
	void removeShape(const std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Get all the shapes associated to this rigid representation
	/// \return The vector containing all the shapes
	std::vector<std::shared_ptr<SurgSim::Math::Shape>> getShapes() const;

	/// Set the shape to use internally for physical parameters computation
	/// \param shape The shape to use for the mass/inertia calculation
	/// \note Also add the shape to the shape list if it has not been added yet
	void setShapeUsedForMassInertia(const std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Get the shape used internally for physical parameters computation
	/// \return The shape used for calculation, nullptr if none exist
	const std::shared_ptr<SurgSim::Math::Shape> getShapeUsedForMassInertia() const;

	/// Test if the the parameters are fully set and ready
	/// \return True if mass and inertia are defined, False otherwise
	/// A rigid body simulation needs these 2 parameters to be defined
	bool isValid() const;

	/// Updates mass, mass center and inertia when density and/or shape used for mass inertia is updated.
	void updateProperties();

private:
	/// Check the validity of the parameters
	/// \return True if mass/inertia are valid and non null, False otherwise
	bool checkValidity() const;

	/// Register accessors of serializable properties
	void addSerializableProperty();

	/// Validity of the set of parameters
	bool m_isValid;

	/// Density of the object (in Kg.m-3)
	double m_rho;

	/// Total mass of the object (in Kg)
	double m_mass;

	/// Linear damping parameter (in N.s.m-1 or Kg.s-1)
	double m_linearDamping;

	/// Angular damping parameter (in N.m.s.rad-1)
	double m_angularDamping;

	/// Mass-center of the object
	SurgSim::Math::Vector3d m_massCenter;

	/// Inertia matrix in local coordinates
	SurgSim::Math::Matrix33d m_localInertia;

	/// Shapes associated with this rigid representation.
	/// Different shape representations could be used for different purposes.
	/// For example, one shape for rigid inertia and another shape for collision detection.
	std::vector<std::shared_ptr<SurgSim::Math::Shape>>  m_shapes;

	/// Shape to be used for the mass/inertia calculation
	std::shared_ptr<SurgSim::Math::Shape> m_shapeForMassInertia;
};

}; // namespace Physics
}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONPARAMETERS_H
