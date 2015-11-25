// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Math/MathConvert.h"

namespace SurgSim
{

namespace Math
{

CompoundShape::CompoundShape()
{
	{
		typedef std::vector<SubShape> PropertyType;
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Math::CompoundShape, PropertyType, Shapes, getShapes, setShapes);
	}
}

CompoundShape::~CompoundShape()
{
}

int CompoundShape::getType() const
{
	return SHAPE_TYPE_COMPOUNDSHAPE;
}

double CompoundShape::getVolume() const
{
	ReadLock lock(m_mutex);
	if (!m_volume.hasValue())
	{
		UpgradeLock write(lock);
		double volume = 0.0;

		for (const auto& shape : m_shapes)
		{
			volume += (shape.first)->getVolume();
		}
		m_volume = volume;
	}
	return *m_volume;
}


Vector3d CompoundShape::getCenter() const
{
	ReadLock lock(m_mutex);
	if (!m_center.hasValue())
	{
		UpgradeLock write(lock);
		Vector3d result = Vector3d::Zero();
		if (m_shapes.size() > 0)
		{
			double total = 0.0;
			for (const auto& shape : m_shapes)
			{
				double volume = shape.first->getVolume();
				result += shape.second * (shape.first->getCenter()) * volume;
				total += volume;
			}
			result /= total;

			// We have it, write the total volume as well ...
			m_volume = total;
		}
		m_center = result;
	}

	return *m_center;
}


Matrix33d CompoundShape::getSecondMomentOfVolume() const
{
	// Calculate the compound values, this needs to be done outside of the ReadLock, otherwise
	// this might freeze up
	auto center = getCenter();

	ReadLock lock(m_mutex);

	if (!m_secondMoment.hasValue())
	{
		Matrix33d result = Matrix33d::Zero();

		UpgradeLock write(lock);

		if (m_shapes.size() > 0)
		{
			for (const auto& subShape : m_shapes)
			{
				const auto& shape = subShape.first;
				const auto& pose = subShape.second;
				const auto& r = pose.linear();
				Matrix33d skew = makeSkewSymmetricMatrix((center - pose * shape->getCenter()).eval());
				Matrix33d inertia =
					r * shape->getSecondMomentOfVolume() * r.transpose() - shape->getVolume() * skew * skew;

				result += inertia;
			}

		}
		m_secondMoment = result;
	}
	return *m_secondMoment;
}


bool CompoundShape::isValid() const
{
	return true;
}

const Math::Aabbd& CompoundShape::getBoundingBox() const
{
	ReadLock lock(m_mutex);
	if (!m_localAabb.hasValue())
	{
		UpgradeLock write(lock);
		Math::Aabbd result;
		for (const auto& subShape : m_shapes)
		{
			result.extend(Math::transformAabb(subShape.second, subShape.first->getBoundingBox()));
		}
		m_localAabb.setValue(result);
	}

	return *m_localAabb;
}

void CompoundShape::invalidateData()
{
	m_volume.invalidate();
	m_center.invalidate();
	m_secondMoment.invalidate();
	m_localAabb.invalidate();
}

size_t CompoundShape::addShape(const std::shared_ptr<Shape>& shape, const RigidTransform3d& pose)
{
	WriteLock lock(m_mutex);
	m_shapes.emplace_back(shape, pose);
	invalidateData();
	return m_shapes.size() - 1;
}

void CompoundShape::setShapes(const std::vector<SubShape>& shapes)
{
	WriteLock lock(m_mutex);
	m_shapes = shapes;
	invalidateData();
}

const std::vector<CompoundShape::SubShape>& CompoundShape::getShapes() const
{
	ReadLock lock(m_mutex);
	return m_shapes;
}

const std::shared_ptr<Shape>& CompoundShape::getShape(size_t index) const
{
	ReadLock lock(m_mutex);
	SURGSIM_ASSERT(index < m_shapes.size()) << "Shape index out of range.";
	return m_shapes[index].first;
}

RigidTransform3d CompoundShape::getPose(size_t index) const
{
	ReadLock lock(m_mutex);
	SURGSIM_ASSERT(index < m_shapes.size()) << "Shape index out of range.";
	return m_shapes[index].second;
}

void CompoundShape::setPoses(const std::vector<RigidTransform3d>& poses)
{
	WriteLock lock(m_mutex);
	SURGSIM_ASSERT(poses.size() == m_shapes.size()) << "New poses and number of shapes differ in size";
	size_t i = 0;
	for (auto& shape : m_shapes)
	{
		shape.second = poses[i++];
	}
	invalidateData();
}


void CompoundShape::setPose(size_t index, const RigidTransform3d& pose)
{
	WriteLock(m_mutex);
	SURGSIM_ASSERT(index < m_shapes.size()) << "Shape index out of range.";
	m_shapes[index].second = pose;
	invalidateData();
}

size_t CompoundShape::getNumShapes() const
{
	ReadLock lock(m_mutex);
	return m_shapes.size();
}

void CompoundShape::clearShapes()
{
	WriteLock lock(m_mutex);
	m_shapes.clear();
}

bool CompoundShape::isTransformable() const
{
	return true;
}

std::shared_ptr<Shape> CompoundShape::getTransformed(const RigidTransform3d& pose) const
{
	auto transformed = std::make_shared<CompoundShape>();
	for (const auto& shape : m_shapes)
	{
		std::shared_ptr<Shape> newShape;
		RigidTransform3d newPose = pose * shape.second;
		if (shape.first->isTransformable())
		{
			newShape = shape.first->getTransformed(newPose);
		}
		else
		{
			newShape = shape.first;
		}
		transformed->addShape(newShape, newPose);
	}
	return transformed;
}

}
}