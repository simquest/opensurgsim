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

class CompoundShape : public Shape
{
public:
	/// Constructor
	CompoundShape();

	explicit CompoundShape(const CompoundShape& other);

	/// Destructor
	~CompoundShape();

	typedef std::pair<std::shared_ptr<Shape>, RigidTransform3d> SubShape;

	/// Add a shape to this shape, you can optionally supply a pose for the added shape
	/// \param shape to be added
	/// \param pose for the newly added shape
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

	using Shape::getPose;

	/// \return the pose of a specific shape
	/// \throws SurgSim::AssertionFailure if the index exceeds the current number of shapes
	RigidTransform3d getPose(size_t index) const;

	/// Sets the poses for all shapes
	/// \param poses array of poses to be copied to each shape
	/// \throws SurgSimm::AssertialFailure if the size of poses.size() != getNumShapes()
	void setPoses(const std::vector<RigidTransform3d>& poses);

	void setPose(const RigidTransform3d& pose) override;

	/// Set the pose for the specified shape
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

	Aabbd getBoundingBox() const override;

	std::shared_ptr<Shape> getCopy() const override;

private:

	/// Clears the data for the volume, center and secondMoment and aabb
	/// so it can be recalculated when needed again
	void invalidateData();

	void updateAabb() const override;

	std::vector<SubShape> m_shapes;

	typedef boost::upgrade_lock<boost::shared_mutex> ReadLock;
	typedef boost::unique_lock<boost::shared_mutex> WriteLock;
	typedef boost::upgrade_to_unique_lock<boost::shared_mutex> UpgradeLock;

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
