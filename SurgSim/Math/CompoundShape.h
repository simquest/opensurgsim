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

#ifndef SURGSIM_MATH_COMPOUNDSHAPE_H
#define SURGSIM_MATH_COMPOUNDSHAPE_H

#include <memory>
#include <utility>

#include <boost/thread.hpp>

#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/DataStructures/OptionalValue.h"

namespace SurgSim
{

namespace Math
{

SURGSIM_STATIC_REGISTRATION(CompoundShape);

class CompoundShape : public Shape
{
public:
	/// Constructor
	CompoundShape();

	/// Destructor
	~CompoundShape();

	SURGSIM_CLASSNAME(SurgSim::Math::CompoundShape);

	typedef std::pair<std::shared_ptr<Shape>, RigidTransform3d> SubShape;

	/// Add a shape to this shape, you can optionally supply a local pose for the added shape w.r.t. this CompoundShape.
	/// \param shape to be added
	/// \param pose The local pose for the newly added shape
	/// \return the index of the newly added shape
	size_t addShape(const std::shared_ptr<Shape>& shape, const RigidTransform3d& pose = RigidTransform3d::Identity());

	/// Sets the shapes for this object, the shapes should be a list of shapes together with their respective poses,
	/// this will clear all the previously contained shapes
	/// \param shapes list of shapes
	void setShapes(const std::vector<SubShape>& shapes);

	/// \return all the contained shapes and their respective poses
	const std::vector<SubShape>& getShapes() const;

	/// \return a specific shape
	/// \throws SurgSim::AssertionFailure if the index exceeds the current number of shapes
	const std::shared_ptr<Shape>& getShape(size_t index) const;

	/// \return The compound pose of a specific shape
	///   i.e., the local pose of this CompoundShape times the local pose of the subshape in the CompoundShape.
	/// \throws SurgSim::AssertionFailure if the index exceeds the current number of shapes
	RigidTransform3d getCompoundPose(size_t index) const;

	/// \return The local pose of a specific shape with respect to this CompoundShape.
	/// \throws SurgSim::AssertionFailure if the index exceeds the current number of shapes
	RigidTransform3d getPose(size_t index) const;

	/// \return The local poses of the subshapes with respect to this CompoundShape.
	const std::vector<RigidTransform3d>& getPoses() const;

	/// Sets the local poses for all subshapes with respect to this CompoundShape.
	/// \param poses array of poses to be copied to each shape
	/// \throws SurgSimm::AssertialFailure if the size of poses.size() != getNumShapes()
	void setPoses(const std::vector<RigidTransform3d>& poses);

	/// Set the local pose for the specified shape with respect to this CompoundShape.
	/// \param index index of the target shape
	/// \param pose new pose for the indicated shape
	/// \throws SurgSim::AssertionFailure if the index exceeds the current number of shapes
	void setPose(size_t index, const RigidTransform3d& pose);

	/// \return the number of shapes in this shape
	size_t getNumShapes() const;

	/// clears all the enclosed shapes
	void clearShapes();

	int getType() const override;

	/// \return the volume of the shape
	/// \note that currently this is a very simple sum of all the volumes of the enclosed shapes it will
	/// disregard any overlapping shapes.
	double getVolume() const override;

	Vector3d getCenter() const override;

	Matrix33d getSecondMomentOfVolume() const override;

	bool isValid() const override;

	const Math::Aabbd& getBoundingBox() const override;

	bool isTransformable() const override;
	std::shared_ptr<Shape> getTransformed(const RigidTransform3d& pose) const override;
	void setPose(const RigidTransform3d& pose) override;
	void updateShape() override;
	void updateShapePartial() override;

private:

	/// Clears the data for the volume, center and secondMoment and aabb
	/// so it can be recalculated when needed again
	void invalidateData();

	std::vector<SubShape> m_shapes;
	/// The local poses of the subshapes with respect to this CompoundShape.
	std::vector<RigidTransform3d> m_poses;
	RigidTransform3d m_lastSetPose;

	typedef boost::shared_lock<boost::shared_mutex> ReadLock;
	typedef boost::unique_lock<boost::shared_mutex> WriteLock;

	mutable boost::shared_mutex m_mutex;

	///@{
	/// Storage for the physical properties of this shape
	/// mutable so they can be recalculated inside the
	/// const functions getVolume(), getCenter(), getSecondMommentOfVolume()
	mutable DataStructures::OptionalValue<Vector3d> m_center;
	mutable DataStructures::OptionalValue<double> m_volume;
	mutable DataStructures::OptionalValue<Matrix33d> m_secondMoment;
	mutable DataStructures::OptionalValue<Math::Aabbd> m_localAabb;
	///@}

};

}
}

#endif
