// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_DATAGROUPCOPIER_H
#define SURGSIM_DATASTRUCTURES_DATAGROUPCOPIER_H

#include <array>
#include <memory>
#include <unordered_map>

#include "SurgSim/DataStructures/NamedData.h"

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
class IndexDirectory;

/// The type used for copying values between two DataGroups that cannot assign to each other.
typedef std::array<NamedDataCopyMap, 9> DataGroupCopyMap;

/// A class that assists in copying from one DataGroup to another, when assignment is not possible.
/// \sa SurgSim::DataStructures::DataGroup
class DataGroupCopier
{
public:
	/// Construct a copier.
	/// \param source The source DataGroup.
	/// \param target The target DataGroup.
	DataGroupCopier(const DataGroup& source, DataGroup* target);

	/// Copies the NamedData entries with the same names.  Resets entries in the target that are reset in the source.
	/// The source and target IndexDirectories are assumed to be the same as the source and target
	/// used in the constructor.
	/// \param source The source DataGroup.
	/// \param target The target DataGroup.
	void copy(const DataGroup& source, DataGroup* target);

private:
	/// Find the entries (by name) from the source to target IndexDirectories, and return the matching entries.
	/// \param source The source IndexDirectory.
	/// \param target The target IndexDirectory.
	/// \return The map from source to target indices.
	NamedDataCopyMap findMap(std::shared_ptr<const IndexDirectory> source,
		std::shared_ptr<const IndexDirectory> target) const;

	/// The map from source to target.
	DataGroupCopyMap m_map;
};

};  // namespace DataStructures
};  // namespace SurgSim


#endif  // SURGSIM_DATASTRUCTURES_DATAGROUPCOPIER_H
