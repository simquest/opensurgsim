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

#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"

namespace SurgSim
{

namespace Physics
{

RigidRepresentationParameters::RigidRepresentationParameters() :
	m_isValid(false),
	m_rho(0.0),
	m_linearDamping(0.0),
	m_angularDamping(0.0),
	m_massCenter(SurgSim::Math::Vector3d::Zero())
{
	// Only mass and inertia are initialized with qNaN because they are
	// the only 2 variables on which the validity test relies on.
	m_mass = std::numeric_limits<double>::quiet_NaN();
	m_localInertia.setConstant(std::numeric_limits<double>::quiet_NaN());

	addSerializableProperty();
}

RigidRepresentationParameters::RigidRepresentationParameters(const RigidRepresentationParameters& rhs) :
	m_isValid(rhs.m_isValid),
	m_rho(rhs.m_rho),
	m_mass(rhs.m_mass),
	m_linearDamping(rhs.m_linearDamping),
	m_angularDamping(rhs.m_angularDamping),
	m_massCenter(rhs.m_massCenter),
	m_localInertia(rhs.m_localInertia),
	m_shapes(rhs.m_shapes),
	m_shapeForMassInertia(rhs.m_shapeForMassInertia)
{
	addSerializableProperty();
}


void RigidRepresentationParameters::addSerializableProperty()
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationParameters, double, Density, getDensity, setDensity);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationParameters, double, LinearDamping,
									  getLinearDamping, setLinearDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationParameters, double, AngularDamping,
									  getAngularDamping, setAngularDamping);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidRepresentationParameters, std::shared_ptr<SurgSim::Math::Shape>,
									  ShapeUsedForMassInertia, getShapeUsedForMassInertia, setShapeUsedForMassInertia);
}


RigidRepresentationParameters& RigidRepresentationParameters::operator=(const RigidRepresentationParameters& rhs)
{
	m_rho = rhs.m_rho;
	m_mass = rhs.m_mass;
	m_massCenter = rhs.m_massCenter;
	m_localInertia = rhs.m_localInertia;
	m_linearDamping = rhs.m_linearDamping;
	m_angularDamping = rhs.m_angularDamping;
	m_shapes = rhs.m_shapes;
	m_shapeForMassInertia = rhs.m_shapeForMassInertia;
	m_isValid = rhs.m_isValid;

	return *this;
}

RigidRepresentationParameters::~RigidRepresentationParameters()
{
}

bool RigidRepresentationParameters::operator==(const RigidRepresentationParameters& rhs) const
{
	using SurgSim::Math::isValid;

	bool isMassEqual = (m_mass == rhs.m_mass);
	isMassEqual |= !isValid(m_mass) && !isValid(rhs.m_mass);

	bool isInertiaEqual = (m_localInertia == rhs.m_localInertia);
	isInertiaEqual |= !isValid(m_localInertia) && !isValid(rhs.m_localInertia);

	return (m_isValid == rhs.m_isValid &&
			m_rho == rhs.m_rho &&
			isMassEqual &&
			m_linearDamping == rhs.m_linearDamping &&
			m_angularDamping == rhs.m_angularDamping &&
			m_massCenter == rhs.m_massCenter &&
			isInertiaEqual &&
			m_shapeForMassInertia == rhs.m_shapeForMassInertia &&
			m_shapes == rhs.m_shapes);
}

bool RigidRepresentationParameters::operator!=(const RigidRepresentationParameters& rhs) const
{
	return !((*this) == rhs);
}

void RigidRepresentationParameters::setDensity(double rho)
{
	m_rho = rho;
	updateProperties();
}

double RigidRepresentationParameters::getDensity() const
{
	return m_rho;
}

void RigidRepresentationParameters::setMass(double mass)
{
	m_mass = mass;

	m_rho = 0.0; // Invalidate the density information
				 // Density is not automatically computed, only set

	m_isValid = checkValidity();
}

double RigidRepresentationParameters::getMass() const
{
	return m_mass;
}

const SurgSim::Math::Vector3d& RigidRepresentationParameters::getMassCenter() const
{
	return m_massCenter;
}

void RigidRepresentationParameters::setLocalInertia(const SurgSim::Math::Matrix33d& localInertia)
{
	m_localInertia = localInertia;
	m_isValid = checkValidity();
}

const SurgSim::Math::Matrix33d& RigidRepresentationParameters::getLocalInertia() const
{
	return m_localInertia;
}

void RigidRepresentationParameters::setLinearDamping(double linearDamping)
{
	m_linearDamping = linearDamping;
}

double RigidRepresentationParameters::getLinearDamping() const
{
	return m_linearDamping;
}

void RigidRepresentationParameters::setAngularDamping(double angularDamping)
{
	m_angularDamping = angularDamping;
}

double RigidRepresentationParameters::getAngularDamping() const
{
	return m_angularDamping;
}

void RigidRepresentationParameters::addShape(const std::shared_ptr<SurgSim::Math::Shape> shape)
{
	m_shapes.push_back(shape);
}

void RigidRepresentationParameters::removeShape(const std::shared_ptr<SurgSim::Math::Shape> shape)
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

std::vector<std::shared_ptr<SurgSim::Math::Shape>> RigidRepresentationParameters::getShapes() const
{
	return m_shapes;
}

void RigidRepresentationParameters::setShapeUsedForMassInertia(const std::shared_ptr<SurgSim::Math::Shape> shape)
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

	updateProperties();
}

const std::shared_ptr<SurgSim::Math::Shape> RigidRepresentationParameters::getShapeUsedForMassInertia() const
{
	return m_shapeForMassInertia;
}

bool RigidRepresentationParameters::isValid() const
{
	return m_isValid;
}

bool RigidRepresentationParameters::checkValidity() const
{
	using SurgSim::Math::isValid;

	return (isValid(m_localInertia) &&
			!m_localInertia.isZero() &&
			m_localInertia.diagonal().minCoeff() > 0.0 &&
			isValid(m_mass) && m_mass > 0.0);
}

void RigidRepresentationParameters::updateProperties()
{
	if (nullptr != m_shapeForMassInertia)
	{
		SURGSIM_ASSERT(m_shapeForMassInertia->isValid()) << "Invalid shape used for mass inertia.";
		if (m_rho > 0)
		{
			m_mass         = m_rho * m_shapeForMassInertia->getVolume();
			m_massCenter   = m_shapeForMassInertia->getCenter();
			m_localInertia = m_rho * m_shapeForMassInertia->getSecondMomentOfVolume();

			m_isValid = checkValidity();
		}
	}
}

}; // namespace Physics
}; // namespace SurgSim