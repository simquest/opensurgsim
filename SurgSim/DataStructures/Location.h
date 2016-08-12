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

#ifndef SURGSIM_DATASTRUCTURES_LOCATION_H
#define SURGSIM_DATASTRUCTURES_LOCATION_H

#include <vector>

#include "SurgSim/DataStructures/OptionalValue.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace DataStructures
{

/// A Location defines a local position w.r.t. any shape. Depending on the type of shape,
/// different data is needed to specify a location on it. This structure supports:
/// - Any rigid shape (location identified by a local position)
/// - Octree (location identified by an octree path)
/// - Index (location indentified by an index)
/// - A triangle mesh (location identified by the triangle id, and the barycentric coordinate of a point in it)
/// - A node in a mesh (location identified by the node id)
/// - An element in a mesh (location identified by the element id, and the barycentric coordinate of a point in it)
struct Location
{
public:
	enum Type {TRIANGLE, ELEMENT};

	/// Default constructor
	Location()
	{
	}

	/// Copy constructor
	/// \param other The location to be copied while constructing.
	Location(const Location& other)
		: rigidLocalPosition(other.rigidLocalPosition),
		  octreeNodePath(other.octreeNodePath),
		  index(other.index),
		  triangleMeshLocalCoordinate(other.triangleMeshLocalCoordinate),
		  elementMeshLocalCoordinate(other.elementMeshLocalCoordinate)
	{}

	/// Constructor for rigid local position
	/// \param localPosition The 3D local position to set this location to
	explicit Location(const SurgSim::Math::Vector3d& localPosition)
	{
		rigidLocalPosition.setValue(localPosition);
	}

	/// Constructor for octree node path
	/// \param nodePath The octree node path to set this location to
	explicit Location(const SurgSim::DataStructures::OctreePath& nodePath)
	{
		octreeNodePath.setValue(nodePath);
	}

	/// Constructor for an index
	/// \param val The index to set this location to
	explicit Location(const size_t val)
	{
		index.setValue(val);
	}

	/// Constructor for mesh-based location
	/// \param localCoordinate index-based local coordinate
	/// \param meshType the type of location (a node, a triangle or an element)
	Location(const SurgSim::DataStructures::IndexedLocalCoordinate& localCoordinate, Type meshType)
	{
		switch (meshType)
		{
			case TRIANGLE:
				triangleMeshLocalCoordinate.setValue(localCoordinate);
				break;
			case ELEMENT:
				elementMeshLocalCoordinate.setValue(localCoordinate);
				break;
			default:
				SURGSIM_FAILURE() << "Unknown location";
				break;
		}
	}

	/// Gives access to the coordinates via enum rather than  '.' public member access, this can reduce complexity
	/// in the calling code
	/// \param meshType (TRIANGLE or ELEMENT)
	/// \return the appropriate coordinate for the type of mesh
	const SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::IndexedLocalCoordinate>& get(Type meshType)
	{
		static SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::IndexedLocalCoordinate> null;
		switch (meshType)
		{
			case TRIANGLE:
				return triangleMeshLocalCoordinate;
				break;
			case ELEMENT:
				return elementMeshLocalCoordinate;
				break;
			default:
				SURGSIM_FAILURE() << "Unknown location";
				break;
		}
		return null;
	}

	bool isApprox(const Location& other, double precision = std::numeric_limits<double>::epsilon()) const
	{
		bool result = false;

		if (rigidLocalPosition.hasValue() && other.rigidLocalPosition.hasValue())
		{
			auto const& vector1 = rigidLocalPosition.getValue();
			auto const& vector2 = other.rigidLocalPosition.getValue();

			result = (vector1.isZero(precision) && vector2.isZero(precision)) || vector1.isApprox(vector2, precision);
		}
		else if (octreeNodePath.hasValue() && other.octreeNodePath.hasValue())
		{
			result = (octreeNodePath.getValue() == other.octreeNodePath.getValue());
		}
		else if (index.hasValue() && other.index.hasValue())
		{
			result = (index.getValue() == other.index.getValue());
		}
		else if (triangleMeshLocalCoordinate.hasValue() && other.triangleMeshLocalCoordinate.hasValue())
		{
			result = triangleMeshLocalCoordinate.getValue().isApprox(other.triangleMeshLocalCoordinate.getValue());
		}
		else if (elementMeshLocalCoordinate.hasValue() && other.elementMeshLocalCoordinate.hasValue())
		{
			result = elementMeshLocalCoordinate.getValue().isApprox(other.elementMeshLocalCoordinate.getValue());
		}
		else
		{
			SURGSIM_FAILURE() << "Invalid location type";
		}

		return result;
	}

	SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d> rigidLocalPosition;
	SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::OctreePath> octreeNodePath;
	SurgSim::DataStructures::OptionalValue<size_t> index;
	SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::IndexedLocalCoordinate> triangleMeshLocalCoordinate;
	SurgSim::DataStructures::OptionalValue<SurgSim::DataStructures::IndexedLocalCoordinate> elementMeshLocalCoordinate;
};


template <typename charT, typename traits, typename T>
std::basic_ostream<charT, traits>& operator << (std::basic_ostream<charT, traits>& out,
		const SurgSim::DataStructures::OptionalValue<T>& val)
{
	if (val.hasValue())
	{
		out << val.getValue();
	}
	else
	{
		out << "<empty>";
	}
	return out;
}

template <typename charT, typename traits>
std::basic_ostream<charT, traits>& operator << (std::basic_ostream<charT, traits>& out,
		const SurgSim::DataStructures::OptionalValue<SurgSim::Math::Vector3d>& val)
{
	if (val.hasValue())
	{
		out << val.getValue().transpose();
	}
	else
	{
		out << "<empty>";
	}
	return out;
}


template <typename charT, typename traits>
std::basic_ostream<charT, traits>& operator << (std::basic_ostream<charT, traits>& out,
		const SurgSim::DataStructures::IndexedLocalCoordinate& val)
{
	out << "[ " << val.index << " : " << val.coordinate.transpose() << " ]";
	return out;
}


template <typename charT, typename traits>
std::basic_ostream<charT, traits>& operator << (std::basic_ostream<charT, traits>& out,
		const Location& loc)
{
	out << "ElementMesh: " << loc.elementMeshLocalCoordinate << std::endl;
	out << "Index: " << loc.index << std::endl;
	out << "RigidLocal: " << loc.rigidLocalPosition << std::endl;
	out << "TriangleMeshLocal: " << loc.triangleMeshLocalCoordinate << std::endl;
	return out;
}

}; // namespace DataStructures
}; // namespace SurgSim

#endif // SURGSIM_DATASTRUCTURES_LOCATION_H
