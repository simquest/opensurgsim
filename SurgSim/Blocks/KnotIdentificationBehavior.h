// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_KNOTIDENTIFICATIONBEHAVIOR_H
#define SURGSIM_BLOCKS_KNOTIDENTIFICATIONBEHAVIOR_H

#include <vector>

#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{
namespace Physics
{
class Fem1DRepresentation;
}
}

namespace SurgSim
{
namespace Blocks
{
/// The KnotIdentificationBehavior detects and identifies a knot in a fem1d representation.
/// The document below gives an overview of the algorithm used to detect the knot.
/// https://docs.google.com/document/d/1a8hCCvtFuOapYsj81enORBiN9pcoCS7HIDioquto6LA
class KnotIdentificationBehavior : public SurgSim::Framework::Behavior
{
public:
	/// \param name The name of this behavior
	explicit KnotIdentificationBehavior(const std::string& name);

	/// \param fem1d The fem1d within which the knot is checked for.
	void setFem1d(const std::shared_ptr<SurgSim::Framework::Component>& fem1d);

	/// \return The fem1d within which the knot is checked for.
	const std::shared_ptr<SurgSim::Physics::Fem1DRepresentation>& getFem1d() const;

	/// \return The knot name.
	const std::string& getKnotName();

	void update(double dt) override;
	int getTargetManagerType() const override;
	bool doInitialize() override;
	bool doWakeUp() override;

protected:
	/// \param projection The projection matrix to be used.
	/// \return The knot ID enum.
	int detectAndIdentifyKnot(const SurgSim::Math::Matrix33d& projection);

	/// \param projection The projection matrix to be used.
	/// \param [out] guassCode The gauss code of the knot projection diagram.
	void getGaussCode(const SurgSim::Math::Matrix33d& projection, std::vector<int>* guassCode);

	/// Build the node data needed to setup the knot identification.
	/// \param projectionX The x-axis of the projection matrix.
	/// \param projectionY The y-axis of the projection matrix.
	/// \param nodes3d [out] The 3d positions of the nodes.
	/// \param nodes2d [out] The 2d (projected) positions of the nodes.
	/// \param segments3d [out] The 3d segments between the nodes.
	void buildNodeData(Math::Vector3d projectionX, Math::Vector3d projectionY,
		std::vector<SurgSim::Math::Vector3d>* nodes3d,
		std::vector<SurgSim::Math::Vector2d>* nodes2d,
		std::vector<SurgSim::Math::Vector3d>* segments3d);

	/// Calculate the crossings from the node data.
	/// \param projectionZ The z-axis of the projection matrix.
	/// \param [in,out] crossings The crossings in the fem1d projection.
	/// \param nodes3d The 3d positions of the nodes.
	/// \param nodes2d The 2d (projected) positions of the nodes.
	/// \param segments3d  The 3d segments between the nodes.
	void calculateCrossings(Math::Vector3d projectionZ,
		std::list<std::tuple<int, size_t, double>>* crossings,
		const std::vector<SurgSim::Math::Vector3d>& nodes3d,
		const std::vector<SurgSim::Math::Vector2d>& nodes2d,
		const std::vector<SurgSim::Math::Vector3d>& segments3d);

	/// Perform Reidmeister moves.
	/// \param [in,out] guassCode The gauss code of the knot projection diagram.
	void performReidmeisterMoves(std::vector<int>* guassCode);

	/// Perform Reidmeister move 1.
	/// \param [in,out] guassCode The gauss code of the knot projection diagram.
	/// \param erased The crossings that were erased during this move
	/// \return True, if a move was done.
	bool tryReidmeisterMove1(std::vector<int>* guassCode, std::vector<int>* erased);

	/// Perform Reidmeister move 2.
	/// \param [in,out] guassCode The gauss code of the knot projection diagram.
	/// \param erased The crossings that were erased during this move
	/// \return True, if a move was done.
	bool tryReidmeisterMove2(std::vector<int>* guassCode,
		std::vector<int>* erased);

	/// Perform Reidmeister move 3.
	/// \param [in,out] guassCode The gauss code of the knot projection diagram.
	/// \return True, if a move was done.
	bool tryReidmeisterMove3(std::vector<int>* guassCode);

	/// \param [in,out] guassCode The gauss code of the knot projection diagram.
	/// \param erased The crossings that were erased.
	void adjustGaussCodeForErasedCrossings(std::vector<int>* guassCode, std::vector<int>* erased);

	/// Identify the knot.
	/// \param [in,out] guassCode The reduced gauss code of the knot projection diagram.
	/// \return The knot id enum.
	int identifyKnot(const std::vector<int>& gaussCode);

	/// The fem1d within which the knot is checked for.
	std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> m_fem1d;

	/// The list of projection matrices to try.
	std::vector<SurgSim::Math::Matrix33d> m_projections;

	/// The name of the knot that was detected.
	std::string m_knotName;

	/// Tracking variables for Reidmeister move 3.
	std::vector<int> m_lastReidmeister3Code;
	size_t m_lastReidmeister3i;
	size_t m_lastReidmeister3iCount;
	size_t m_lastReidmeister3m;
	size_t m_lastReidmeister3n;

private:
	/// \param code The Gauss Code of the knot diagram
	/// \param i The current index in the code
	/// \return The next index in the code (cycle if end of list is reached)
	size_t next(const std::vector<int>& code, size_t i);

	/// \param code The Gauss Code of the knot diagram
	/// \param i The current index in the code
	/// \return The previous index in the code (cycle to end if start of list is reached)
	size_t prev(const std::vector<int>& code, size_t i);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j Indices of two entries in the code
	/// \return True, if the entries have the same sign
	bool isSameSign(const std::vector<int>& code, size_t i, size_t j);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j Indices of two entries in the code
	/// \return True, if the entries are the same cross (same id, different sign)
	bool isSameCross(const std::vector<int>& code, size_t i, size_t j);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j, k, l Indices of four entries in the code
	/// \return True, if segment from i->j overlaps segment k->l
	bool doesOverlap(const std::vector<int>& code, size_t i, size_t j, size_t k, size_t l);

	/// \param [in,out] code The Gauss Code of the knot diagram
	/// \param i, j Entries at these indices are erased
	void erase(std::vector<int>* code, size_t i, size_t j);

	/// \param [in,out] code The Gauss Code of the knot diagram
	/// \param i, j, k, l Entries at these indices are erased
	void erase(std::vector<int>* code, size_t i, size_t j, size_t k, size_t l);

	/// \param code The Gauss Code of the knot diagram
	/// \param i The current index in the code
	/// \return The next index in the code (cycle if end of list is reached)
	size_t getComplementCross(const std::vector<int>& code, size_t i);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j The two entries which are checked if they have a common cross as a neighbor
	/// \param [in,out] k, l The out-params to store the common neighbor for i and j. The also bring in the
	///					k, l from the last iteration for the same i, j. So, a different k, l is found (if it is possible)
	/// \return True, if a common neighbor was found
	bool hasCommonNeighbor(const std::vector<int>& code, size_t i, size_t j, size_t* k, size_t* l);

	/// \param code The Gauss Code of the knot diagram
	/// \param knot The knot code of a known knot, to which the firs parameter is compared to.
	bool isSame(const std::vector<int>& code, const std::vector<int>& knot);
};

} // namespace Blocks
} // namespace SurgSim

#endif // SURGSIM_BLOCKS_KNOTIDENTIFICATIONBEHAVIOR_H
