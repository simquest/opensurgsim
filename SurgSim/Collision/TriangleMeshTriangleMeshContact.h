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

#ifndef SURGSIM_COLLISION_TRIANGLEMESHTRIANGLEMESHCONTACT_H
#define SURGSIM_COLLISION_TRIANGLEMESHTRIANGLEMESHCONTACT_H

#include <memory>

#include "SurgSim/Collision/ShapeShapeContactCalculation.h"
#include "SurgSim/Math/MeshShape.h"


namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// Class to calculate intersections between a triangle mesh and a triangle mesh
class TriangleMeshTriangleMeshContact : public ShapeShapeContactCalculation<Math::MeshShape, Math::MeshShape>
{
public:
	using ContactCalculation::calculateDcdContact;

	std::list<std::shared_ptr<Contact>> calculateDcdContact(
										 const Math::MeshShape& mesh1,
										 const Math::RigidTransform3d& mesh1Pose,
										 const Math::MeshShape& mesh2,
										 const Math::RigidTransform3d& mesh2Pose) const override;

	std::list<std::shared_ptr<Contact>> calculateCcdContact(
		const Math::MeshShape& shape1AtTime0, const Math::RigidTransform3d& pose1AtTime0,
		const Math::MeshShape& shape1AtTime1, const Math::RigidTransform3d& pose1AtTime1,
		const Math::MeshShape& shape2AtTime0, const Math::RigidTransform3d& pose2AtTime0,
		const Math::MeshShape& shape2AtTime1, const Math::RigidTransform3d& pose2AtTime1) const override;

	std::pair<int, int> getShapeTypes() override;

private:
	/// Handles the DCD case in the calculateCcdContact.
	/// \param t1v0,t1v1,t1v2 The first triangle's vertices.
	/// \param t2v0,t2v1,t2v2 The second triangle's vertices.
	/// \param t1n Normal of the first triangle.
	/// \param t2n Normal of the second triangle.
	/// \param triangle1Id The index of the first triangle in it's mesh.
	/// \param triangle2Id The index of the second triangle in it's mesh.
	/// \param pose1AtTime1 The pose of Shape 1 (first triangle's shape) at Time 1.
	/// \param pose2AtTime1 The pose of Shape 2 (second triangle's shape) at Time 1.
	/// \param [out] contacts The contacts generated (if any) will be added to this list.
	void ccdContactDcdCase(
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
		const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
		const Math::Vector3d& t1n, const Math::Vector3d& t2n, size_t triangle1Id, size_t triangle2Id,
		const Math::RigidTransform3d& pose1AtTime1, const Math::RigidTransform3d& pose2AtTime1,
		std::list<std::shared_ptr<Contact>>* contacts) const;

	/// Handles the CCD case in the calculateCcdContact.
	/// \param t1v0,t1v1,t1v2 The first triangle's vertices.
	/// \param t2v0,t2v1,t2v2 The second triangle's vertices.
	/// \param t1n Normal of the first triangle.
	/// \param t2n Normal of the second triangle.
	/// \param triangle1Id The index of the first triangle in it's mesh.
	/// \param triangle2Id The index of the second triangle in it's mesh.
	/// \param pose1AtTime1 The pose of Shape 1 (first triangle's shape) at Time 1.
	/// \param pose2AtTime1 The pose of Shape 2 (second triangle's shape) at Time 1.
	/// \param [out] contacts The contacts generated (if any) will be added to this list.
	void ccdContactCcdCase(
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
		const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
		const Math::Vector3d& t1n, const Math::Vector3d& t2n, size_t triangle1Id, size_t triangle2Id,
		const Math::RigidTransform3d& pose1AtTime1, const Math::RigidTransform3d& pose2AtTime1,
		std::list<std::shared_ptr<Contact>>* contacts) const;

	/// Handles the Segment/Segment intersection case in the ccdContactCcdCase.
	/// \param t1v0,t1v1,t1v2 The first triangle's vertices.
	/// \param t2v0,t2v1,t2v2 The second triangle's vertices.
	/// \param earliestTimeOfImpact [in,out] The earliest time of impact found so far, and updated in this method.
	/// \param triangle1Alpha,triangle1Beta [out] Point in the first triangle is represented as
	///        t1v0 + triangle1Alpha * (t1v1 - t1v0) + triangle1Beta * (t1v2 - t1v0).
	/// \param triangle2Alpha,triangle2Beta [out] Point in second triangle is represented as
	///        t2v0 + triangle2Alpha * (t2v1 - t2v0) + triangle2Beta * (t2v2 - t2v0).
	/// \return True if segment/segment intersection is found.
	bool ccdContactCcdCaseSegmentSegment(
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
		const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
		double* earliestTimeOfImpact, double* triangle1Alpha, double* triangle1Beta, double* triangle2Alpha,
		double* triangle2Beta) const;

	/// Handles the Point/Triangle intersection case in the ccdContactCcdCase.
	/// \param t1v0,t1v1,t1v2 The first triangle's vertices.
	/// \param t2v0,t2v1,t2v2 The second triangle's vertices.
	/// \param earliestTimeOfImpact [in,out] The earliest time of impact found so far, and updated in this method.
	/// \param triangle1Alpha,triangle1Beta [out] Point in the first triangle is represented as
	///        t1v0 + triangle1Alpha * (t1v1 - t1v0) + triangle1Beta * (t1v2 - t1v0).
	/// \param triangle2Alpha,triangle2Beta [out] Point in second triangle is represented as
	///        t2v0 + triangle2Alpha * (t2v1 - t2v0) + triangle2Beta * (t2v2 - t2v0).
	/// \param t1VertexThroughT2 [out] True when the vertex of first triangle goes through the area of the second.
	/// \return True if point/triangle intersection is found.
	bool ccdContactCcdCasePointTriangle(
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
		const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
		const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
		double* earliestTimeOfImpact, double* triangle1Alpha, double* triangle1Beta, double* triangle2Alpha,
		double* triangle2Beta, bool* t1VertexThroughT2) const;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_TRIANGLEMESHTRIANGLEMESHCONTACT_H
