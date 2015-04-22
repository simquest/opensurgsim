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
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"

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
	SHAPE_TYPE_COUNT
} ShapeType;

/// Generic rigid shape class defining a shape
/// \note This class gives the ability to analyze the shape and compute
/// \note physical information (volume, mass, mass center, inertia)
class Shape : virtual public SurgSim::Framework::Accessible
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

	typedef SurgSim::Framework::ObjectFactory<SurgSim::Math::Shape> FactoryType;

	/// \return The static class factory that is being used in the conversion.
	static FactoryType& getFactory();

	/// Get class name
	virtual std::string getClassName() const;

	/// Check if the shape is valid
	/// \return True if shape is valid; Otherwise, false.
	virtual bool isValid() const = 0;
};

}; // Math
}; // SurgSim

#endif // SURGSIM_MATH_SHAPE_H
