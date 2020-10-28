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

#include <boost/assign/list_of.hpp>
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

namespace SurgSim
{
namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::KnotIdentificationBehavior,
				 KnotIdentificationBehavior);

KnotIdentificationBehavior::KnotIdentificationBehavior(const std::string& name) :
	Framework::Behavior(name), m_knotName("Undetermined")
{
	using boost::assign::list_of;

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(KnotIdentificationBehavior,
		std::shared_ptr<SurgSim::Physics::Fem1DRepresentation>, Fem1d, getFem1d, setFem1d);

	addKnownKnotCode("Trefoil Knot", boost::assign::list_of(1)(-2)(3)(-1)(2)(-3),
		boost::assign::list_of(1)(1)(1)(1)(1)(1));
	addKnownKnotCode("Granny Knot", boost::assign::list_of(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6)(-1)(2)(-3),
		boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1));
	addKnownKnotCode("Granny Knot", boost::assign::list_of(1)(-2)(3)(4)(-5)(6)(-4)(5)(-6)(-1)(2)(-3),
		boost::assign::list_of(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1)(-1));
	addKnownKnotCode("Granny Knot", boost::assign::list_of(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6)(-1)(2)(-3),
		boost::assign::list_of(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1));
	addKnownKnotCode("Granny Knot", boost::assign::list_of(1)(-2)(3)(4)(-5)(6)(-4)(5)(-6)(-1)(2)(-3),
		boost::assign::list_of(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1)(1));
	addKnownKnotCode("Square Knot", boost::assign::list_of(1)(-2)(3)(4)(-5)(6)(-4)(5)(-6)(-1)(2)(-3),
		boost::assign::list_of(-1)(-1)(-1)(1)(1)(1)(1)(1)(1)(-1)(-1)(-1));
	addKnownKnotCode("Square Knot", boost::assign::list_of(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6)(-1)(2)(-3),
		boost::assign::list_of(-1)(-1)(-1)(1)(1)(1)(1)(1)(1)(-1)(-1)(-1));
	addKnownKnotCode("Square Knot", boost::assign::list_of(1)(-2)(3)(4)(-5)(6)(-4)(5)(-6)(-1)(2)(-3),
		boost::assign::list_of(1)(1)(1)(-1)(-1)(-1)(-1)(-1)(-1)(1)(1)(1));
	addKnownKnotCode("Square Knot", boost::assign::list_of(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6)(-1)(2)(-3),
		boost::assign::list_of(1)(1)(1)(-1)(-1)(-1)(-1)(-1)(-1)(1)(1)(1));
}

void KnotIdentificationBehavior::setFem1d(const std::shared_ptr<Framework::Component>& fem1d)
{
	m_fem1d = Framework::checkAndConvert<Physics::Fem1DRepresentation>(fem1d,
		"SurgSim::Physics::Fem1DRepresentation");
}

const std::shared_ptr<Physics::Fem1DRepresentation>& KnotIdentificationBehavior::getFem1d() const
{
	return m_fem1d;
}

const std::string& KnotIdentificationBehavior::getKnotName()
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	return m_knotName;
}

void KnotIdentificationBehavior::update(double dt)
{
	std::vector<std::string> results;
	for (const auto& projection : m_projections)
	{
		const auto& knotName = detectAndIdentifyKnot(projection);
		if (knotName != "No Knot")
		{
			results.push_back(knotName);
		}
	}
	std::string knotName = "No Knot";
	if (results.size() > 0)
	{
		knotName = results.back();
		if (results.size() > 1)
		{
			std::vector<std::string> known;
			for (const auto& r : results)
			{
				if (r != "Unknown Knot")
				{
					known.push_back(r);
				}
			}
			ptrdiff_t maxCount = 0;
			for (const auto& k : known)
			{
				auto count = std::count(known.begin(), known.end(), k);
				if (count > maxCount)
				{
					maxCount = count;
					knotName = k;
				}
			}
		}
	}
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_knotName = knotName;
}

int KnotIdentificationBehavior::getTargetManagerType() const
{
	return Framework::MANAGER_TYPE_PHYSICS;
}

bool KnotIdentificationBehavior::doInitialize()
{
	using Math::makeRigidTransform;
	Math::Matrix33d projection;
	m_projections.clear();
	m_projections.push_back(Math::Matrix33d::Identity() * 100.0);
	{
		projection <<
			1.0, 0.0, 0.0,
			0.0, 0.0, -1.0,
			0.0, 1.0, 0.0;
		m_projections.push_back(projection * 100.0);
	}
	{
		projection <<
			0.0, 0.0, -1.0,
			0.0, 1.0, 0.0,
			1.0, 0.0, 0.0;
		m_projections.push_back(projection * 100.0);
	}
	{
		projection <<
			0.0, -0.707107, -0.707107,
			0.0, 0.707107, -0.707107,
			1.0, 0.0, 0.0;
		m_projections.push_back(projection * 100.0);
	}
	{
		projection <<
			-0.707107, 0.0, -0.707107,
			0.0, 1.0, -0.0,
			0.707107, -0.0, -0.707107;
		m_projections.push_back(projection * 100.0);
	}
	{
		projection <<
			-1.0, 0.0, -0.0,
			0.0, 0.707107, -0.707107,
			0.0, -0.707107, -0.707107;
		m_projections.push_back(projection * 100.0);
	}
	return true;
}

bool KnotIdentificationBehavior::doWakeUp()
{
	if (m_fem1d == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
			getName() + "' must have a Fem1D Representation to detect knots in it.";
		return false;
	}
	return true;
}

void KnotIdentificationBehavior::addKnownKnotCode(const std::string& name, const std::vector<int>& code,
	const std::vector<int>& signs)
{
	std::vector<Crossing> enhancedGaussCode;
	auto codeLength = code.size();
	SURGSIM_ASSERT(codeLength == signs.size()) << " " <<  getClassName() << "::" << __func__ <<
		" passed a vector of signs that is not the same length as the code.";
	for (size_t id = 0; id < codeLength; ++id)
	{
		enhancedGaussCode.push_back(Crossing(code[id], 0, 0.0, signs[id]));
	}
	if (m_knownLists.find(name) == m_knownLists.end())
	{
		
		auto v = std::vector<std::vector<Crossing>>();
		v.push_back(enhancedGaussCode);
		m_knownLists[name] = v;
	}
	else
	{
		m_knownLists[name].push_back(enhancedGaussCode);
	}
}

void KnotIdentificationBehavior::clearKnownKnotCodes()
{
	m_knownLists.clear();
}

std::string KnotIdentificationBehavior::detectAndIdentifyKnot(
	const Math::Matrix33d& projection)
{
	auto gaussCode = getGaussCode(projection);

	// Perform reidmeister moves.
	performReidmeisterMoves(&gaussCode);

	// Identify the knot.
	return identifyKnot(gaussCode);
}

std::vector<KnotIdentificationBehavior::Crossing> KnotIdentificationBehavior::getGaussCode(
	const Math::Matrix33d& projection)
{
	std::vector<Vector3d> nodes3d;
	std::vector<Vector2d> nodes2d;
	std::vector<Vector3d> segments3d;

	buildNodeData(projection.col(0), projection.col(1), &nodes3d, &nodes2d, &segments3d);

	auto crossings = calculateCrossings(projection.col(2), nodes3d, nodes2d, segments3d);

	if (crossings.empty())
	{
		return std::vector<Crossing>();
	}

	std::sort(crossings.begin(), crossings.end(), [](const Crossing& i, const Crossing& j)
	{
		return (i.segmentId == j.segmentId) ? (i.segmentLocation < j.segmentLocation) : (i.segmentId < j.segmentId);
	});
	return crossings;
}

void KnotIdentificationBehavior::buildNodeData(
	const Vector3d& projectionX,
	const Vector3d& projectionY,
	std::vector<Vector3d>* nodes3d,
	std::vector<Vector2d>* nodes2d,
	std::vector<Vector3d>* segments3d)
{
	// Reserve the memory
	nodes3d->reserve(m_fem1d->getNumFemElements() + 2);
	nodes2d->reserve(m_fem1d->getNumFemElements() + 2);
	segments3d->reserve(m_fem1d->getNumFemElements() + 1);

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
	const auto& lastNode = 
		finalState->getPosition(m_fem1d->getFemElement(m_fem1d->getNumFemElements() - 1)->getNodeIds()[1]);
	node3d = lastNode + 0.99 * (finalState->getPosition(m_fem1d->getFemElement(0)->getNodeIds()[0]) - lastNode);
	node2d[0] = projectionX.dot(node3d);
	node2d[1] = projectionY.dot(node3d);
	segments3d->push_back(node3d - nodes3d->back());
	nodes3d->push_back(node3d);
	nodes2d->push_back(node2d);
}

std::vector<KnotIdentificationBehavior::Crossing> KnotIdentificationBehavior::calculateCrossings(
	const Math::Vector3d& projectionZ,
	const std::vector<SurgSim::Math::Vector3d>& nodes3d,
	const std::vector<SurgSim::Math::Vector2d>& nodes2d,
	const std::vector<SurgSim::Math::Vector3d>& segments3d)
{
	std::vector<Crossing> crossings;
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
				auto under = i;
				auto over = j;
				if (sign > 0)
				{
					under = j;
					over = i;
				}
				SurgSim::Math::Vector2d segments2dUnder = nodes2d[under + 1] - nodes2d[under];
				SurgSim::Math::Vector2d segments2dOver = nodes2d[over + 1] - nodes2d[over];
				double dot = segments2dUnder.dot(segments2dOver);
				double determinant = segments2dUnder.x() * segments2dOver.y() -
					segments2dUnder.y() * segments2dOver.x();
				int handedness = boost::math::sign(std::atan2(determinant, dot));
				crossings.push_back(Crossing(crossingId * sign, i, s, handedness));
				crossings.push_back(Crossing(crossingId * -sign, j, t, handedness));
				crossingId++;
			}
		}
	}

	return crossings;
}

void KnotIdentificationBehavior::performReidmeisterMoves(std::vector<Crossing>* gaussCode)
{
	if (gaussCode->empty())
	{
		return;
	}

	// Setup some variables for Reidmeister Move 3.
	ReidmeisterMove3Data move3Data;

	// Perform Reidmeister moves.
	std::vector<int> erased;
	bool reidmeisterMovePerformed = false;
	do
	{
		reidmeisterMovePerformed =
			tryReidmeisterMove1(gaussCode, &erased) ||
			tryReidmeisterMove2(gaussCode, &erased) ||
			tryReidmeisterMove3(gaussCode, &move3Data);
	} while (reidmeisterMovePerformed && !gaussCode->empty());

	adjustGaussCodeForErasedCrossings(gaussCode);
}

bool KnotIdentificationBehavior::tryReidmeisterMove1(std::vector<Crossing>* gaussCode,
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
			erased->push_back(std::abs((*gaussCode)[i].id));
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

bool KnotIdentificationBehavior::tryReidmeisterMove2(std::vector<Crossing>* gaussCode,
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
			l = nextIndex(*gaussCode, k);
			// If the gaussCode is correctly formed, then it has to have more than 2 crossings by the time this gets
			// executed, so we don't need to check the case that k == i before l == i.
			while (l != i)
			{
				if (doesOverlap(*gaussCode, i, j, k, l))
				{
					erased->push_back(std::abs((*gaussCode)[i].id));
					erased->push_back(std::abs((*gaussCode)[j].id));
					erase(gaussCode, i, j, k, l);
					return true;
				}
				k = l;
				l = nextIndex(*gaussCode, k);
			}
		}
		++i;
	}
	return false;
}

bool KnotIdentificationBehavior::tryReidmeisterMove3(std::vector<Crossing>* gaussCode, ReidmeisterMove3Data* data)
{
	size_t i = 0;
	size_t j;
	size_t k;
	size_t l;
	size_t m;
	size_t n;
	if (data->code.empty() || data->code.size() > gaussCode->size())
	{
		data->code = *gaussCode;
		data->i = std::numeric_limits<size_t>::max();
		data->iCount = 0;
		data->m = std::numeric_limits<size_t>::max();
		data->n = std::numeric_limits<size_t>::max();
	}
	else
	{
		*gaussCode = data->code;
		i = data->i;
	}

	while (i < gaussCode->size())
	{
		j = nextIndex(*gaussCode, i);
		if (isSameSign(*gaussCode, i, j) && data->iCount < 2)
		{
			k = getComplementCross(*gaussCode, i);
			l = getComplementCross(*gaussCode, j);
			m = (i == data->i) ? data->m : std::numeric_limits<size_t>::max();
			n = (i == data->i) ? data->n : std::numeric_limits<size_t>::max();
			if (hasCommonNeighbor(*gaussCode, k, l, &m, &n))
			{
				data->iCount = (i == data->i) ? (data->iCount + 1) : 1;
				data->i = i;
				data->m = m;
				data->n = n;
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

void KnotIdentificationBehavior::adjustGaussCodeForErasedCrossings(std::vector<Crossing>* gaussCode)
{
	using boost::math::sign;
	std::map<int, int> oldToNew;
	int next = 1;
	for (auto& code : *gaussCode)
	{
		const auto id = std::abs(code.id);
		int newId;
		if (oldToNew.find(id) == oldToNew.end())
		{
			newId = next;
			++next;
			oldToNew[id] = newId;
		}
		else
		{
			newId = oldToNew[id];
		}
		code.id = sign(code.id) * newId;
	}
}

std::string KnotIdentificationBehavior::identifyKnot(const std::vector<Crossing>& gaussCode)
{
	if (gaussCode.empty())
	{
		return "No Knot";
	}

	for (const auto& knotVector : m_knownLists)
	{
		for (const auto& knot : knotVector.second)
		{
			if (isSameCode(gaussCode, knot))
			{
				return knotVector.first;
			}
		}
	}

	return "Unknown Knot";
}

size_t KnotIdentificationBehavior::nextIndex(const std::vector<Crossing>& code, size_t i)
{
	return (++i) % code.size();
}

size_t KnotIdentificationBehavior::prevIndex(const std::vector<Crossing>& code, size_t i)
{
	return (i + code.size() - 1) % code.size();
}

bool KnotIdentificationBehavior::isSameSign(const std::vector<Crossing>& code, size_t i, size_t j)
{
	using boost::math::sign;
	return sign(code[i].id) == sign(code[j].id);
}

bool KnotIdentificationBehavior::isSameCross(const std::vector<Crossing>& code, size_t i, size_t j)
{
	return code[i].id == -code[j].id;
}

bool KnotIdentificationBehavior::doesOverlap(const std::vector<Crossing>& code, size_t i, size_t j, size_t k, size_t l)
{
	return isSameSign(code, i, j) && isSameSign(code, k, l) && !isSameSign(code, i, k) &&
		   ((isSameCross(code, i, k) && isSameCross(code, j, l)) ||
			(isSameCross(code, i, l) && isSameCross(code, j, k)));
}

void KnotIdentificationBehavior::erase(std::vector<Crossing>* code, size_t i, size_t j)
{
	code->erase(code->begin() + std::max(i, j));
	code->erase(code->begin() + std::min(i, j));
}

void KnotIdentificationBehavior::erase(std::vector<Crossing>* code, size_t i, size_t j, size_t k, size_t l)
{
	std::vector<size_t> list = { i, j, k, l };
	std::sort(list.begin(), list.end(), std::greater<size_t>());
	code->erase(code->begin() + list[0]);
	code->erase(code->begin() + list[1]);
	code->erase(code->begin() + list[2]);
	code->erase(code->begin() + list[3]);
}

size_t KnotIdentificationBehavior::getComplementCross(const std::vector<Crossing>& code, size_t i)
{
	for (size_t j = 0; j < code.size(); ++j)
	{
		if (code[j].id == -code[i].id)
		{
			return j;
		}
	}
	SURGSIM_FAILURE() << "Cannot find the complement cross for " << i;
	return std::numeric_limits<size_t>::max();
}

bool KnotIdentificationBehavior::hasCommonNeighbor(const std::vector<Crossing>& code, size_t i, size_t j,
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

bool KnotIdentificationBehavior::isSameCode(const std::vector<Crossing>& code, const std::vector<Crossing>& knot)
{
	if (code.size() != knot.size())
	{
		return false;
	}

	bool isSame = true;
	for (size_t i = 0; i < code.size() && isSame; ++i)
	{
		isSame = (code[i].id == knot[i].id) && (code[i].sign == knot[i].sign);
	}

	if (!isSame)
	{
		// Check if (code * -1) is the same as knot.
		isSame = true;
		for (size_t i = 0; i < code.size() && isSame; ++i)
		{
			isSame = (-code[i].id == knot[i].id) && (code[i].sign == knot[i].sign);
		}
	}

	return isSame;
}

} //namespace Blocks
} //namespace SurgSim
