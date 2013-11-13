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

#include <SurgSim/Math/Shapes.h>
#include <SurgSim/Math/Valid.h>

namespace SurgSim
{

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
class RigidRepresentationParameters
{
public:
	/// Default constructor
	RigidRepresentationParameters()
		: m_rho(0.0), m_linearDamping(0.0), m_angularDamping(0.0), m_isValid(false)
	{
		m_massCenter.setZero();

		// Only mass and inertia are initialized with qNaN because they are
		// the only 2 variables on which the validity test relies on.
		m_mass = std::numeric_limits<double>::quiet_NaN();
		m_localInertia.setConstant(std::numeric_limits<double>::quiet_NaN());
	}

	/// Destructor
	virtual ~RigidRepresentationParameters()
	{
	}

	/// Comparison operator
	/// \param c A RigidRepresentationParameters to compare it to
	/// \return True if the 2 parameters set are equals, False otherwise
	bool operator ==(const RigidRepresentationParameters &c) const
	{
		using SurgSim::Math::isValid;

		bool isMassEqual = m_mass == c.m_mass;
		isMassEqual |= ! isValid(m_mass) && ! isValid(c.m_mass);

		bool isInertiaEqual = m_localInertia == c.m_localInertia;
		isInertiaEqual |= ! isValid(m_localInertia) && ! isValid(c.m_localInertia);

		return (m_isValid == c.m_isValid &&
			m_rho == c.m_rho &&
			isMassEqual &&
			m_linearDamping == c.m_linearDamping &&
			m_angularDamping == c.m_angularDamping &&
			m_massCenter == c.m_massCenter &&
			isInertiaEqual &&
			m_shapeForMassInertia == c.m_shapeForMassInertia &&
			m_shapes == c.m_shapes);
	}

	/// Comparison operator (for difference)
	/// \param c A RigidRepresentationParameters to compare it to
	/// \return False if the 2 parameters set are equals, True otherwise
	bool operator !=(const RigidRepresentationParameters &c) const
	{
		return ! ((*this) == c);
	}

	/// Set the mass density of the rigid representation
	/// There is 2 ways to define a rigid representation mass/inertia:
	/// * Set its density and its shape will compute automatically mass/inertia
	/// * Set the mass/inertia directly.
	/// \param rho The density (in Kg.m-3)
	/// \note rho = 0, resets the density and keeps the mass/inertia as is.
	/// \note rho != 0 will overwrite mass/inertia using the given mesh/shape
	/// \note if any.
	void setDensity(double rho)
	{
		m_rho = rho;

		/// Lets update the mass, mass-center and inertia if possible
		if (m_rho && m_shapeForMassInertia)
		{
			// If a shape overwrite a mesh, the shape should be used !
			m_mass         = m_shapeForMassInertia->calculateMass(m_rho);
			m_massCenter   = m_shapeForMassInertia->calculateMassCenter();
			m_localInertia = m_shapeForMassInertia->calculateInertia(m_rho);

			m_isValid = checkValidity();
		}
	}

	/// Get the mass density of the rigid representation
	/// \return The density if it has been provided, 0 otherwise (in Kg.m-3)
	double getDensity() const
	{
		return m_rho;
	}

	/// Set the mass of the rigid body
	/// There is 2 ways to define a rigid representation mass/inertia:
	/// * Set its density and its shape will compute automatically mass/inertia
	/// * Set the mass/inertia directly
	/// \param mass The mass (in Kg)
	/// This reset the density to 0 (unknown)
	void setMass(double mass)
	{
		m_mass = mass;

		m_rho = 0.0; // Invalidate the density information
					 // Density is not automcatically computed, only set

		m_isValid = checkValidity();
	}

	/// Get the mass of the rigid body
	/// \return The mass (in Kg)
	double getMass() const
	{
		return m_mass;
	}

	/// Get the mass center of the rigid body
	/// \return The mass center (in local coordinate)
	const SurgSim::Math::Vector3d& getMassCenter() const
	{
		return m_massCenter;
	}

	/// Set the local inertia of the rigid body
	/// \param localInertia The inertia 3x3 symetric matrix of the object
	/// This overwrites any previously computed/set local inertia
	void setLocalInertia(const SurgSim::Math::Matrix33d& localInertia)
	{
		m_localInertia = localInertia;

		m_isValid = checkValidity();
	}

	/// Get the local inertia 3x3 matrix of the rigid body
	/// \return The inertia 3x3 matrix of the object
	const SurgSim::Math::Matrix33d& getLocalInertia() const
	{
		return m_localInertia;
	}

	/// Set the linear damping parameter
	/// \param linearDamping The linear damping parameter (in N.s.m-1)
	void setLinearDamping(double linearDamping)
	{
		m_linearDamping = linearDamping;
	}

	/// Get the linear damping parameter
	/// \return The linear damping parameter (in N.s.m-1)
	double getLinearDamping() const
	{
		return m_linearDamping;
	}

	/// Set the angular damping parameter
	/// \param angularDamping The angular damping parameter (in N.m.s.rad-1)
	void setAngularDamping(double angularDamping)
	{
		m_angularDamping = angularDamping;
	}

	/// Get the angular damping parameter
	/// \return The angular damping parameter (in N.m.s.rad-1)
	double getAngularDamping() const
	{
		return m_angularDamping;
	}

	/// Add a shape to this rigid representation
	/// \param shape A shape to add to this rigid representation
	void addShape(const std::shared_ptr<SurgSim::Math::Shape> shape)
	{
		m_shapes.push_back(shape);
	}

	/// Remove a shape from this rigid representation
	/// \param shape The shape to be detached from this rigid representation
	/// \note Does nothing if the shape is not found
	void removeShape(const std::shared_ptr<SurgSim::Math::Shape> shape)
	{
		auto it = std::find(m_shapes.begin(), m_shapes.end(), shape);
		if (it != m_shapes.end())
		{
			m_shapes.erase(it);
		}

		if (m_shapeForMassInertia == shape)
		{
			m_shapeForMassInertia = nullptr;
		}
	}

	/// Get all the shapes associated to this rigid representation
	/// \return The vector containing all the shapes
	std::vector<std::shared_ptr<SurgSim::Math::Shape>> getShapes() const
	{
		return m_shapes;
	}

	/// Set the shape to use internally for physical parameters computation
	/// \param shape The shape to use for the mass/inertia calculation
	/// \note Also add the shape to the shape list if it has not been added yet
	void setShapeUsedForMassInertia(const std::shared_ptr<SurgSim::Math::Shape> shape)
	{
		m_shapeForMassInertia = shape;

		if (shape == nullptr)
		{
			return;
		}

		if (std::find(m_shapes.begin(), m_shapes.end(), shape) == m_shapes.end())
		{
			addShape(shape);
		}

		if (m_rho && m_shapeForMassInertia)
		{
			m_mass         = m_shapeForMassInertia->calculateMass(m_rho);
			m_massCenter   = m_shapeForMassInertia->calculateMassCenter();
			m_localInertia = m_shapeForMassInertia->calculateInertia(m_rho);

			m_isValid = checkValidity();
		}
	}

	/// Get the shape used internally for physical parameters computation
	/// \return The shape used for calculation, nullptr if none exist
	const std::shared_ptr<SurgSim::Math::Shape> getShapeUsedForMassInertia() const
	{
		return m_shapeForMassInertia;
	}

		/// Test if the the parameters are fully set and ready
	/// \return True if mass and inertia are defined, False otherwise
	/// A rigid body simulation needs these 2 parameters to be defined
	bool isValid() const
	{
		return m_isValid;
	}

private:
	/// Check the validity of the parameters
	/// \return True if mass/inertia are valid and non null, False otherwise
	bool checkValidity() const
	{
		using SurgSim::Math::isValid;
		if (isValid(m_localInertia) && ! m_localInertia.isZero() &&
			isValid(m_mass) && m_mass != 0.0)
		{
			return true;
		}
		return false;
	}

	/// Density of the object (in Kg.m-3)
	double m_rho;

	/// Total mass of the object (in Kg)
	double m_mass;

	/// Mass-center of the object
	SurgSim::Math::Vector3d m_massCenter;

	/// Inertia matrix in local coordinates
	SurgSim::Math::Matrix33d m_localInertia;

	/// Linear damping parameter (in N.s.m-1 or Kg.s-1)
	double m_linearDamping;

	/// Angular damping parameter (in N.m.s.rad-1)
	double m_angularDamping;

	/// Shapes associated to this rigid representation
	std::vector<std::shared_ptr<SurgSim::Math::Shape>>  m_shapes;

	/// Shape to be used for the mass/inertia calculation
	std::shared_ptr<SurgSim::Math::Shape> m_shapeForMassInertia;

	/// Validity of the set of parameters
	bool m_isValid;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDREPRESENTATIONPARAMETERS_H
