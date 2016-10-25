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

#include "SurgSim/Blocks/KnotIdentificationBehavior.h"

#include <boost/math/special_functions/sign.hpp>
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgTextRepresentation.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector2d;

namespace
{
// The list of knots that can be identified here.
const int knot3[] = {1, -2, 3, -1, 2, -3};

struct Knot
{
	std::string name;
	std::vector<int> code;
	Knot(const std::string& name, std::vector<int> code)
		: name(name), code(code)
	{}
};

std::array<Knot, 2> knots = {
	Knot("No Knot", std::vector<int>()),
	Knot("Trefoil Knot", std::vector<int>(knot3, knot3 + sizeof(knot3) / sizeof(int)))
};
}

namespace SurgSim
{
namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::KnotIdentificationBehavior,
				 KnotIdentificationBehavior);

KnotIdentificationBehavior::KnotIdentificationBehavior(const std::string& name) :
	Framework::Behavior(name), m_knotName("Calculating..."), m_lastReidmeister3iCount(0)
{
}

void KnotIdentificationBehavior::setFem1d(const std::shared_ptr<Framework::Component>& fem1d)
{
	m_fem1d = Framework::checkAndConvert<Physics::Fem1DRepresentation>(fem1d,
		"Physics::Fem1DRepresentation");
}

const std::shared_ptr<Physics::Fem1DRepresentation>& KnotIdentificationBehavior::getFem1d() const
{
	return m_fem1d;
}

const std::string& KnotIdentificationBehavior::getKnotName()
{
	return m_knotName;
}

void KnotIdentificationBehavior::update(double dt)
{
	if (m_fem1d == nullptr)
	{
		return;
	}

	bool knotDetected = false;
	auto projection = m_projections.begin();
	do 
	{
		knotDetected = detectAndIdentifyKnot(*projection);
		++projection;
	} while (!knotDetected && projection != m_projections.end());
}

int KnotIdentificationBehavior::getTargetManagerType() const
{
	return Framework::MANAGER_TYPE_PHYSICS;
}

bool KnotIdentificationBehavior::doInitialize()
{
	using Math::makeRigidTransform;
	Math::Matrix33d projection;
	m_projections.push_back(Math::Matrix33d::Identity() * 100.0);
	{
		projection = makeRigidTransform(Vector3d(0,0,0), Vector3d(0,0,1), Vector3d(0,1,0)).linear();
		m_projections.push_back(projection * 100.0);
	}
	{
		projection = makeRigidTransform(Vector3d(0,0,0), Vector3d(0,1,0), Vector3d(0,0,1)).linear();
		m_projections.push_back(projection * 100.0);
	}

	return true;
}

bool KnotIdentificationBehavior::doWakeUp()
{
	return true;
}

bool KnotIdentificationBehavior::detectAndIdentifyKnot(
	const Math::Matrix33d& projection)
{
	auto gaussCode = getGaussCode(projection);

	// Perform reidmeister moves.
	performReidmeisterMoves(&gaussCode);

	// Identify the knot.
	return identifyKnot(gaussCode);
}

std::vector<int> KnotIdentificationBehavior::getGaussCode(
	const Math::Matrix33d& projection)
{
	std::vector<Vector3d> nodes3d;
	std::vector<Vector2d> nodes2d;
	std::vector<Vector3d> segments3d;

	buildNodeData(projection.col(0), projection.col(1), &nodes3d, &nodes2d, &segments3d);

	auto crossings = calculateCrossings(projection.col(2), nodes3d, nodes2d, segments3d);

	if (crossings.empty())
	{
		return std::vector<int>();
	}

	crossings.sort([](const Crossing& i, const Crossing& j)
	{
		return (i.segmentId == j.segmentId) ? (i.segmentLocation < j.segmentLocation) : (i.segmentId < j.segmentId);
	});

	std::vector<int> gaussCode;
	gaussCode.reserve(crossings.size());
	for (const auto& cross : crossings)
	{
		gaussCode.push_back(cross.id);
	}

	return gaussCode;
}

void KnotIdentificationBehavior::buildNodeData(
	const Vector3d& projectionX,
	const Vector3d& projectionY,
	std::vector<Vector3d>* nodes3d,
	std::vector<Vector2d>* nodes2d,
	std::vector<Vector3d>* segments3d)
{
	// Reserve the memory
	nodes3d->reserve(m_fem1d->getNumFemElements() + 1);
	nodes2d->reserve(m_fem1d->getNumFemElements() + 1);
	segments3d->reserve(m_fem1d->getNumFemElements());

	const auto& finalState = m_fem1d->getFinalState();

	Vector3d node3d;
	Vector2d node2d;
	for (size_t i = 0; i < m_fem1d->getNumFemElements(); ++i)
	{	
		if (i == 0)
		{
			node3d = finalState->getPosition(m_fem1d->getFemElement(i)->getNodeIds()[0]);
			node2d[0] = projectionX.dot(node3d);
			node2d[1] = projectionY.dot(node3d);
			nodes3d->push_back(node3d);
			nodes2d->push_back(node2d);
		}

		node3d = finalState->getPosition(m_fem1d->getFemElement(i)->getNodeIds()[1]);
		node2d[0] = projectionX.dot(node3d);
		node2d[1] = projectionY.dot(node3d);

		segments3d->push_back(node3d - nodes3d->back());

		nodes3d->push_back(node3d);
		nodes2d->push_back(node2d);
	}
}

std::list<KnotIdentificationBehavior::Crossing> KnotIdentificationBehavior::calculateCrossings(
	const Math::Vector3d& projectionZ,
	const std::vector<SurgSim::Math::Vector3d>& nodes3d,
	const std::vector<SurgSim::Math::Vector2d>& nodes2d,
	const std::vector<SurgSim::Math::Vector3d>& segments3d)
{
	std::list<Crossing> crossings;
	int crossingId = 1;
	double s, t;
	size_t numSegments = segments3d.size();
	for (size_t i = 0; i < numSegments - 2; ++i)
	{
		for (size_t j = i + 2; j < numSegments; ++j)
		{
			if (Math::doesIntersectSegmentSegment(nodes2d[i], nodes2d[i + 1], nodes2d[j], nodes2d[j + 1], &s, &t))
			{
				int sign =
					((nodes3d[i] + segments3d[i] * s) - (nodes3d[j] + segments3d[j] * t)).dot(projectionZ) > 0.0
					? 1 : -1;
				crossings.push_back(Crossing(crossingId * sign, i, s));
				crossings.push_back(Crossing(crossingId * -sign, j, t));
				crossingId++;
			}
		}
	}

	return crossings;
}

void KnotIdentificationBehavior::performReidmeisterMoves(std::vector<int>* gaussCode)
{
	if (gaussCode->empty())
	{
		return;
	}

	// Setup some variables for Reidmeister Move 3.
	m_lastReidmeister3Code.clear();

	// Perform Reidmeister moves.
	std::vector<int> erased;
	bool reidmeisterMovePerformed = false;
	do
	{
		reidmeisterMovePerformed =
			tryReidmeisterMove1(gaussCode, &erased) ||
			tryReidmeisterMove2(gaussCode, &erased) ||
			tryReidmeisterMove3(gaussCode);
	} while (reidmeisterMovePerformed && !gaussCode->empty());

	adjustGaussCodeForErasedCrossings(gaussCode, &erased);
}

bool KnotIdentificationBehavior::tryReidmeisterMove1(std::vector<int>* gaussCode,
													 std::vector<int>* erased)
{
	bool movePerformed = false;
	size_t i = 0;
	size_t j;
	while (i < gaussCode->size())
	{
		j = nextIndex(*gaussCode, i);
		if (isSameCross(*gaussCode, i, j))
		{
			erased->push_back(std::abs((*gaussCode)[i]));
			erase(gaussCode, i, j);
			movePerformed = true;
		}
		else
		{
			++i;
		}
	}
	return movePerformed;
}

bool KnotIdentificationBehavior::tryReidmeisterMove2(std::vector<int>* gaussCode,
													 std::vector<int>* erased)
{
	size_t i = 0;
	size_t j;
	size_t k;
	size_t l;
	while (i < gaussCode->size())
	{
		j = nextIndex(*gaussCode, i);
		if (isSameSign(*gaussCode, i, j))
		{
			k = nextIndex(*gaussCode, j);
			while (k != i)
			{
				l = nextIndex(*gaussCode, k);
				if (doesOverlap(*gaussCode, i, j, k, l))
				{
					erased->push_back(std::abs((*gaussCode)[i]));
					erased->push_back(std::abs((*gaussCode)[j]));
					erase(gaussCode, i, j, k, l);
					return true;
				}
				k = nextIndex(*gaussCode, k);
			}
		}
		++i;
	}
	return false;
}

bool KnotIdentificationBehavior::tryReidmeisterMove3(std::vector<int>* gaussCode)
{
	size_t i = 0;
	size_t j;
	size_t k;
	size_t l;
	size_t m;
	size_t n;
	if (m_lastReidmeister3Code.empty() || m_lastReidmeister3Code.size() > gaussCode->size())
	{
		m_lastReidmeister3Code = *gaussCode;
		m_lastReidmeister3i = std::numeric_limits<size_t>::max();
		m_lastReidmeister3iCount = 0;
		m_lastReidmeister3m = std::numeric_limits<size_t>::max();
		m_lastReidmeister3n = std::numeric_limits<size_t>::max();
	}
	else
	{
		*gaussCode = m_lastReidmeister3Code;
		i = m_lastReidmeister3i;
	}

	while (i < gaussCode->size())
	{
		j = nextIndex(*gaussCode, i);
		if (isSameSign(*gaussCode, i, j) && m_lastReidmeister3iCount < 2)
		{
			k = getComplementCross(*gaussCode, i);
			l = getComplementCross(*gaussCode, j);
			m = (i == m_lastReidmeister3i) ? m_lastReidmeister3m : std::numeric_limits<size_t>::max();
			n = (i == m_lastReidmeister3i) ? m_lastReidmeister3n : std::numeric_limits<size_t>::max();
			if (hasCommonNeighbor(*gaussCode, k, l, &m, &n))
			{
				m_lastReidmeister3iCount = (i == m_lastReidmeister3i) ? (m_lastReidmeister3iCount + 1) : 1;
				m_lastReidmeister3i = i;
				m_lastReidmeister3m = m;
				m_lastReidmeister3n = n;
				std::swap(gaussCode->at(k), gaussCode->at(l));
				std::swap(gaussCode->at(k), gaussCode->at(m));
				std::swap(gaussCode->at(l), gaussCode->at(n));
				return true;
			}
		}
		++i;
	}
	return false;
}

void KnotIdentificationBehavior::adjustGaussCodeForErasedCrossings(std::vector<int>* gaussCode,
																   std::vector<int>* erased)
{
	using boost::math::sign;

	if (!gaussCode->empty() && !erased->empty())
	{
		std::sort(erased->begin(), erased->end(), std::greater<int>());
		for (const auto& id : *erased)
		{
			for (auto& code : *gaussCode)
			{
				if (std::abs(code) > id)
				{
					code = sign(code) * (std::abs(code) - 1);
				}
			}
		}
	}
}

bool KnotIdentificationBehavior::identifyKnot(const std::vector<int>& gaussCode)
{
	if (gaussCode.empty())
	{
		m_knotName = "No Knot";
		return false;
	}

	for (const auto& knot : knots)
	{
		if (isSameCode(gaussCode, knot.code))
		{
			m_knotName = knot.name;
			return true;
		}
	}

	m_knotName = "Unknown Knot";
	return false;
}

size_t KnotIdentificationBehavior::nextIndex(const std::vector<int>& code, size_t i)
{
	return (++i) % code.size();
}

size_t KnotIdentificationBehavior::prevIndex(const std::vector<int>& code, size_t i)
{
	return (i + code.size() - 1) % code.size();
}

bool KnotIdentificationBehavior::isSameSign(const std::vector<int>& code, size_t i, size_t j)
{
	using boost::math::sign;
	return sign(code[i]) == sign(code[j]);
}

bool KnotIdentificationBehavior::isSameCross(const std::vector<int>& code, size_t i, size_t j)
{
	return code[i] == -code[j];
}

bool KnotIdentificationBehavior::doesOverlap(const std::vector<int>& code, size_t i, size_t j, size_t k, size_t l)
{
	return isSameSign(code, i, j) && isSameSign(code, k, l) && !isSameSign(code, i, k) &&
		   ((isSameCross(code, i, k) && isSameCross(code, j, l)) ||
		    (isSameCross(code, i, l) && isSameCross(code, j, k)));
}

void KnotIdentificationBehavior::erase(std::vector<int>* code, size_t i, size_t j)
{
	code->at(i) = 0;
	code->at(j) = 0;
	code->erase(std::remove(code->begin(), code->end(), 0), code->end());
}

void KnotIdentificationBehavior::erase(std::vector<int>* code, size_t i, size_t j, size_t k, size_t l)
{
	code->at(i) = 0;
	code->at(j) = 0;
	code->at(k) = 0;
	code->at(l) = 0;
	code->erase(std::remove(code->begin(), code->end(), 0), code->end());
}

size_t KnotIdentificationBehavior::getComplementCross(const std::vector<int>& code, size_t i)
{
	for (size_t j = 0; j < code.size(); ++j)
	{
		if (code[j] == -code[i])
		{
			return j;
		}
	}
	SURGSIM_FAILURE() << "Cannot find the complement cross for " << i;
	return std::numeric_limits<size_t>::max();
}

bool KnotIdentificationBehavior::hasCommonNeighbor(const std::vector<int>& code, size_t i, size_t j,
												   size_t* k, size_t* l)
{
	size_t iPrev = prevIndex(code, i);
	size_t iNext = nextIndex(code, i);
	size_t jPrev = prevIndex(code, j);
	size_t jNext = nextIndex(code, j);

	if (isSameCross(code, iPrev, jPrev) && !(*k == iPrev && *l == jPrev))
	{
		*k = iPrev;
		*l = jPrev;
		return true;
	}
	if (isSameCross(code, iPrev, jNext) && !(*k == iPrev && *l == jNext))
	{
		*k = iPrev;
		*l = jNext;
		return true;
	}
	if (isSameCross(code, iNext, jPrev) && !(*k == iNext && *l == jPrev))
	{
		*k = iNext;
		*l = jPrev;
		return true;
	}
	if (isSameCross(code, iNext, jNext) && !(*k == iNext && *l == jNext))
	{
		*k = iNext;
		*l = jNext;
		return true;
	}

	return false;
}

bool KnotIdentificationBehavior::isSameCode(const std::vector<int>& code, const std::vector<int>& knot)
{
	if (code.size() != knot.size())
	{
		return false;
	}

	size_t start = 0;
	while (start < code.size() && code[0] != knot[start])
	{
		++start;
	}

	if (start == code.size())
	{
		return false;
	}

	bool isSame = true;
	size_t j = start;
	for (size_t i = 0; i < code.size() && isSame; ++i)
	{
		isSame = code[i] == knot[j];
		j = nextIndex(knot, j);
	}

	return isSame;
}

} //namespace Blocks
} //namespace SurgSim
