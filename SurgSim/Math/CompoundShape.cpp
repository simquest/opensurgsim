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
#include "SurgSim/Math/MeshShape.h"

namespace SurgSim
{

namespace Math
{

SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::CompoundShape, CompoundShape);

CompoundShape::CompoundShape() : m_lastSetPose(RigidTransform3d::Identity())
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
	{
		ReadLock lock(m_mutex);
		if (m_volume.hasValue())
		{
			return *m_volume;
		}
	}

	{
		WriteLock lock(m_mutex);
		if (!m_volume.hasValue())
		{
			double volume = 0.0;

			for (const auto& shape : m_shapes)
			{
				volume += (shape.first)->getVolume();
			}
			m_volume = volume;
		}
		return *m_volume;
	}
}


Vector3d CompoundShape::getCenter() const
{
	{
		ReadLock lock(m_mutex);
		if (m_center.hasValue())
		{
			return *m_center;
		}
	}

	{
		WriteLock lock(m_mutex);
		if (!m_center.hasValue())
		{
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
}

Matrix33d CompoundShape::getSecondMomentOfVolume() const
{
	// Calculate the compound values, this needs to be done outside of the ReadLock, otherwise
	// this might freeze up
	auto center = getCenter();

	{
		ReadLock lock(m_mutex);
		if (m_secondMoment.hasValue())
		{
			return *m_secondMoment;
		}
	}

	{
		WriteLock lock(m_mutex);
		if (!m_secondMoment.hasValue())
		{
			Matrix33d result = Matrix33d::Zero();
			if (m_shapes.size() > 0)
			{
				for (const auto& subShape : m_shapes)
				{
					const auto& shape = subShape.first;
					const auto& pose = subShape.second;
					const auto& r = pose.linear();
					Matrix33d skew = makeSkewSymmetricMatrix((center - pose * shape->getCenter()).eval()); /// check whether istransformable subshapes should multiply by pose (their localpose)
					Matrix33d inertia =
						r * shape->getSecondMomentOfVolume() * r.transpose() - shape->getVolume() * skew * skew; /// check whether istransformable subshapes should multiply by r (the linear part of their localpose)

					result += inertia;
				}
			}
			m_secondMoment = result;
		}
		return *m_secondMoment;
	}
}


bool CompoundShape::isValid() const
{
	return true;
}

const Math::Aabbd& CompoundShape::getBoundingBox() const
{
	{
		ReadLock lock(m_mutex);
		if (m_localAabb.hasValue())
		{
			return *m_localAabb;
		}
	}

	{
		WriteLock lock(m_mutex);
		if (!m_localAabb.hasValue())
		{
			Math::Aabbd result;
			for (const auto& subShape : m_shapes)
			{
				result.extend(Math::transformAabb(subShape.second, subShape.first->getBoundingBox()));
			}
			m_localAabb.setValue(result);
		}
		return *m_localAabb;
	}
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
	m_localPoses.push_back(pose);

	const auto newPose = m_lastSetPose * pose;
	if (shape->isTransformable())
	{
		shape->setPose(newPose);
		m_shapes.emplace_back(shape, pose);
	}
	else
	{
		m_shapes.emplace_back(shape, newPose);
	}

	invalidateData();
	return m_shapes.size() - 1;
}

void CompoundShape::setShapes(const std::vector<SubShape>& shapes)
{
	WriteLock lock(m_mutex);
	m_shapes.clear();
	m_shapes = shapes;
	m_localPoses.clear();
	for (auto& shape : m_shapes)
	{
		m_localPoses.push_back(shape.second);
		const auto newPose = m_lastSetPose * shape.second;
		if (shape.first->isTransformable())
		{
			shape.first->setPose(newPose);
		}
		else
		{
			shape.second = newPose;
		}
	}
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

void CompoundShape::setPoses(const std::vector<UnalignedRigidTransform3d>& poses)
{
	WriteLock lock(m_mutex);
	SURGSIM_ASSERT(poses.size() == m_shapes.size()) << "New poses and number of shapes differ in size";
	size_t i = 0;
	for (auto& shape : m_shapes)
	{
		const auto newPose = m_lastSetPose * poses[i];
		if (shape.first->isTransformable())
		{
			shape.first->setPose(newPose);
			shape.second = poses[i];
		}
		else
		{
			shape.second = newPose;
		}
		m_localPoses[i] = poses[i];
		++i;
	}
	invalidateData();
}


void CompoundShape::setPose(size_t index, const RigidTransform3d& pose)
{
	WriteLock(m_mutex);
	SURGSIM_ASSERT(index < m_shapes.size()) << "Shape index out of range.";
	m_localPoses[index] = pose;
	auto& shape = m_shapes[index];
	const auto newPose = m_lastSetPose * pose;
	if (shape.first->isTransformable())
	{
		shape.first->setPose(newPose);
		shape.second = pose;
	}
	else
	{
		shape.second = newPose;
	}
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
	invalidateData();
}

bool CompoundShape::isTransformable() const
{
	return true;
}

std::shared_ptr<Shape> CompoundShape::getTransformed(const RigidTransform3d& pose) const
{
	ReadLock lock(m_mutex);
	auto transformed = std::make_shared<CompoundShape>();
	for (const auto& shape : m_shapes)
	{
		std::shared_ptr<Shape> newShape;
		RigidTransform3d newPose = pose * shape.second; // not sure what to do here...if we call setPose and then getTransformed
		if (shape.first->isTransformable())
		{
			newShape = shape.first->getTransformed(newPose);
			transformed->addShape(newShape, shape.second);
		}
		else
		{
			newShape = shape.first;
			transformed->addShape(newShape, newPose);
		}
	}
	return transformed;
}

void CompoundShape::setPose(const RigidTransform3d& pose)
{
	WriteLock(m_mutex);
	size_t index = 0;
	for (auto& shape : m_shapes)
	{
		const auto& localPose = m_localPoses[index++];
		const auto newPose = pose * localPose;
		if (shape.first->isTransformable())
		{
			shape.first->setPose(newPose);
			shape.second = localPose;
		}
		else
		{
			shape.second = newPose;
		}
		//shape.second = newPose; //is this the problem?  When we setPose on the CompoundShape, maybe it shouldn't be in the pose for the subshapes.  Maybe the subshape's poses should always be local.
		// but that's what we do in getTransformed
		//test this change with the non-transformable subshapes...
	}
	m_lastSetPose = pose;
	invalidateData();
}

void CompoundShape::updateShape()
{
	WriteLock(m_mutex);
	for (auto& shape : m_shapes)
	{
		shape.first->updateShape();
	}
}

void CompoundShape::updateShapePartial()
{
	WriteLock(m_mutex);
	for (auto& shape : m_shapes)
	{
		shape.first->updateShapePartial();
	}
}

}
}