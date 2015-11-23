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

#ifndef SURGSIM_MATH_SHAPE_H
#define SURGSIM_MATH_SHAPE_H

#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Math
{

/// Type defining the shape direction for certain templatized shape
/// (i.e. CylinderShape and CapsuleShape)
typedef enum
{
	SHAPE_DIRECTION_AXIS_X = 0,
	SHAPE_DIRECTION_AXIS_Y = 1,
	SHAPE_DIRECTION_AXIS_Z = 2
} ShapeDirection;

/// Fixed List of enums for the available Shape types, do not explicitly assign values, ShapeCount is
/// used to determine the number of actual shape types
typedef enum
{
	SHAPE_TYPE_NONE = -1,
	SHAPE_TYPE_BOX,
	SHAPE_TYPE_CAPSULE,
	SHAPE_TYPE_CYLINDER,
	SHAPE_TYPE_DOUBLESIDEDPLANE,
	SHAPE_TYPE_MESH,
	SHAPE_TYPE_OCTREE,
	SHAPE_TYPE_PARTICLES,
	SHAPE_TYPE_PLANE,
	SHAPE_TYPE_SPHERE,
	SHAPE_TYPE_SURFACEMESH,
	SHAPE_TYPE_SEGMENTMESH,
	SHAPE_TYPE_COMPOUNDSHAPE,
	SHAPE_TYPE_COUNT
} ShapeType;

/// Generic rigid shape class defining a shape
/// \note This class gives the ability to analyze the shape and compute
/// \note physical information (volume, mass, mass center, inertia)
class Shape : virtual public SurgSim::Framework::Accessible, public Framework::FactoryBase<Shape>
{
public:
	typedef ::SurgSim::Math::Vector3d Vector3d;
	typedef ::SurgSim::Math::Matrix33d Matrix33d;

	// Destructor
	virtual ~Shape();

	/// \return the type of shape
	virtual int getType() const = 0;

	/// Get the volume of the shape
	/// \return The volume of the shape (in m-3)
	virtual double getVolume() const = 0;

	/// Get the volumetric center of the shape
	/// \return The center of the shape
	virtual Vector3d getCenter() const = 0;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	virtual Matrix33d getSecondMomentOfVolume() const = 0;

	/// \return true if the the shape can be transformed
	virtual bool isTransformable() const;

	/// Get a copy of this shape with an applied rigid transform
	/// \param pose The pose to transform the shape by
	/// \return the posed shape
	virtual std::shared_ptr<Shape> getTransformed(const RigidTransform3d& pose);

	/// Get class name
	virtual std::string getClassName() const;

	/// Check if the shape is valid
	/// \return True if shape is valid; Otherwise, false.
	virtual bool isValid() const = 0;
};

/// PosedShape is a transformed shape with a record of the pose used to transform it.
template <class T>
struct PosedShape
{
	PosedShape() { pose = Math::RigidTransform3d::Identity(); }
	PosedShape(const T& shapeInput, const Math::RigidTransform3d& poseInput) : shape(shapeInput), pose(poseInput) {}

	void invalidate() { shape = nullptr; }
	const T& getShape() const { return shape; }
	const Math::RigidTransform3d& getPose() const { return pose; }

protected:
	T shape;
	Math::RigidTransform3d pose;
};

/// PosedShapeMotion is embedding the motion of a PosedShape, providing a posed shape at 2 different instant.
template <class T>
struct PosedShapeMotion : public std::pair<PosedShape<T>, PosedShape<T>>
{
	PosedShapeMotion() {}
	PosedShapeMotion(const PosedShape<T>& posedShapeFirst, const PosedShape<T>& posedShapeSecond)
	{
		this->first = posedShapeFirst;
		this->second = posedShapeSecond;
	}
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_SHAPE_H
