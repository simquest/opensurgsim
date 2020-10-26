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
/// https://en.wikipedia.org/wiki/Reidemeister_move explains the idea of Reidmeister moves.
/// The document below gives an overview of the algorithm used to detect the knot.
/// https://docs.google.com/document/d/1a8hCCvtFuOapYsj81enORBiN9pcoCS7HIDioquto6LA
/// This behavior may incorrectly identify knots, or fail to identify knots.
/// This code does not correctly handle when three segments projected into 2D all cross at one point, and
/// may fail when fem1d segments projected into 2D are degenerate.
/// It will always fail to identify knots with extra loops or crossings.
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

	/// Function to add a known knot code to the list.
	/// \param name The name of the knot
	/// \param code The gauss code of the knot
	/// \param signs The crossing signs of the knot. Orient the knot so that the crossing is in 2D and the 
	/// under-crossing is traveling upwards. The crossing sign is +1 if the over-crossing is traveling
	/// left-to-right, otherwise -1.
	void addKnownKnotCode(const std::string& name, const std::vector<int>& code, const std::vector<int>& signs);

	/// Function to clear the known knot code list.
	void clearKnownKnotCodes();

	/// struct to store a Crossing.
	/// Gauss code is not sufficient, also need the crossing sign (aka enhanced gauss code), see:
	/// Ellis, Graham, and Cedric Fragnaud. "Computing with knot quandles."
	/// Journal of Knot Theory and Its Ramifications 27.14 (2018): 1850074.
	struct Crossing
	{
		/// The gauss code. abs(id) indicates which crossing. sign(id) is positive for over and negative for under.
		int id;
		size_t segmentId;
		double segmentLocation;
		int sign; ///< also known as handedness. See documentation for addKnownKnotCode.
		Crossing(int id, size_t segmentId, double segmentLocation, int sign)
			: id(id), segmentId(segmentId), segmentLocation(segmentLocation), sign(sign) {}
	};

protected:
	/// Known knots, mapping the name to a vector of extended gauss codes.
	std::map<std::string, std::vector<std::vector<Crossing>>> m_knownLists;

	/// \param projection The projection matrix to be used.
	/// \return True, if a knot was detected.
	bool detectAndIdentifyKnot(const SurgSim::Math::Matrix33d& projection);

	/// \param projection The projection matrix to be used.
	/// \return The gauss code of the knot projection diagram.
	std::vector<Crossing> getGaussCode(const SurgSim::Math::Matrix33d& projection);

	/// Build the node data needed to setup the knot identification.
	/// \param projectionX The x-axis of the projection matrix.
	/// \param projectionY The y-axis of the projection matrix.
	/// \param nodes3d [out] The 3d positions of the nodes.
	/// \param nodes2d [out] The 2d (projected) positions of the nodes.
	/// \param segments3d [out] The 3d segments between the nodes.
	void buildNodeData(const Math::Vector3d& projectionX,
		const Math::Vector3d& projectionY,
		std::vector<SurgSim::Math::Vector3d>* nodes3d,
		std::vector<SurgSim::Math::Vector2d>* nodes2d,
		std::vector<SurgSim::Math::Vector3d>* segments3d);

	/// Calculate the crossings from the node data.
	/// \param projectionZ The z-axis of the projection matrix.
	/// \param nodes3d The 3d positions of the nodes.
	/// \param nodes2d The 2d (projected) positions of the nodes.
	/// \param segments3d  The 3d segments between the nodes.
	/// \return The crossings in the fem1d projection.
	std::vector<Crossing> calculateCrossings(const Math::Vector3d& projectionZ,
		const std::vector<SurgSim::Math::Vector3d>& nodes3d,
		const std::vector<SurgSim::Math::Vector2d>& nodes2d,
		const std::vector<SurgSim::Math::Vector3d>& segments3d);

	/// Perform Reidmeister moves.
	/// \param [in,out] gaussCode The gauss code of the knot projection diagram.
	void performReidmeisterMoves(std::vector<Crossing>* gaussCode);

	/// Perform Reidmeister move 1.
	/// \param [in,out] gaussCode The gauss code of the knot projection diagram.
	/// \param erased The crossings that were erased during this move
	/// \return True, if a move was done.
	bool tryReidmeisterMove1(std::vector<Crossing>* gaussCode, std::vector<int>* erased);

	/// Perform Reidmeister move 2.
	/// \param [in,out] gaussCode The gauss code of the knot projection diagram.
	/// \param erased The crossings that were erased during this move
	/// \return True, if a move was done.
	bool tryReidmeisterMove2(std::vector<Crossing>* gaussCode, std::vector<int>* erased);

	/// struct with variables for tracking Reidmeister move 3.
	struct ReidmeisterMove3Data
	{
		std::vector<Crossing> code;
		size_t i;
		size_t iCount;
		size_t m;
		size_t n;
		ReidmeisterMove3Data()
			: i(std::numeric_limits<size_t>::max()), iCount(0),
			  m(std::numeric_limits<size_t>::max()), n(std::numeric_limits<size_t>::max()) {}
	};

	/// Perform Reidmeister move 3.
	/// \param [in,out] gaussCode The gauss code of the knot projection diagram.
	/// \param data [in,out] The tracking data for this move.
	/// \return True, if a move was done.
	bool tryReidmeisterMove3(std::vector<Crossing>* gaussCode, ReidmeisterMove3Data* data);

	/// Squeezes together the ids so that their absolute values are 1, 2, 3, etc. without gaps.
	/// \param [in,out] gaussCode The gauss code of the knot projection diagram.
	void adjustGaussCodeForErasedCrossings(std::vector<Crossing>* gaussCode);

	/// Identify the knot.
	/// \param [in,out] gaussCode The reduced gauss code of the knot projection diagram.
	/// \return True, if a knot was detected.
	bool identifyKnot(const std::vector<Crossing>& gaussCode);

	/// The fem1d within which the knot is checked for.
	std::shared_ptr<SurgSim::Physics::Fem1DRepresentation> m_fem1d;

	/// The list of projection matrices to try.
	std::vector<SurgSim::Math::Matrix33d> m_projections;

	/// The name of the knot that was detected.
	std::string m_knotName;

private:
	/// \param code The Gauss Code of the knot diagram
	/// \param i The current index in the code
	/// \return The next index in the code (cycle if end of list is reached)
	size_t nextIndex(const std::vector<Crossing>& code, size_t i);

	/// \param code The Gauss Code of the knot diagram
	/// \param i The current index in the code
	/// \return The previous index in the code (cycle to end if start of list is reached)
	size_t prevIndex(const std::vector<Crossing>& code, size_t i);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j Indices of two entries in the code
	/// \return True, if the entries have the same sign
	bool isSameSign(const std::vector<Crossing>& code, size_t i, size_t j);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j Indices of two entries in the code
	/// \return True, if the entries are the same cross (same id, different sign)
	bool isSameCross(const std::vector<Crossing>& code, size_t i, size_t j);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j, k, l Indices of four entries in the code
	/// \return True, if segment from i->j overlaps segment k->l
	bool doesOverlap(const std::vector<Crossing>& code, size_t i, size_t j, size_t k, size_t l);

	/// \param [in,out] code The Gauss Code of the knot diagram
	/// \param i, j Entries at these indices are erased
	void erase(std::vector<Crossing>* code, size_t i, size_t j);

	/// \param [in,out] code The Gauss Code of the knot diagram
	/// \param i, j, k, l Entries at these indices are erased
	void erase(std::vector<Crossing>* code, size_t i, size_t j, size_t k, size_t l);

	/// \param code The Gauss Code of the knot diagram
	/// \param i The current index in the code
	/// \return The next index in the code (cycle if end of list is reached)
	size_t getComplementCross(const std::vector<Crossing>& code, size_t i);

	/// \param code The Gauss Code of the knot diagram
	/// \param i, j The two entries which are checked if they have a common cross as a neighbor
	/// \param [in,out] k, l The out-params to store the common neighbor for i and j. The also bring in the k, l
	///					from the last iteration for the same i, j. So, a different k, l is found (if it is possible)
	/// \return True, if a common neighbor was found
	bool hasCommonNeighbor(const std::vector<Crossing>& code, size_t i, size_t j, size_t* k, size_t* l);

	/// \param code The Gauss Code of the knot diagram
	/// \param knot The knot code of a known knot, to which the first parameter is compared to.
	bool isSameCode(const std::vector<Crossing>& code, const std::vector<Crossing>& knot);
};

} // namespace Blocks
} // namespace SurgSim

#endif // SURGSIM_BLOCKS_KNOTIDENTIFICATIONBEHAVIOR_H
