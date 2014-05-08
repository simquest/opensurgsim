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

#include "SurgSim/DataStructures/DataGroup.h"

namespace SurgSim
{
namespace DataStructures
{
/// A class that assists in copying from one DataGroup to another, when assignment is not possible.
/// \sa SurgSim::DataStructures::DataGroup
class DataGroupCopier
{
public:

	/// Construct a copier.
	/// \param from The source DataGroup.
	/// \param to The target DataGroup.
	DataGroupCopier(const DataGroup& from, DataGroup& to);

	/// Copies the NamedData entries with the same names.  Resets entries in the target that are reset in the source.
	void copy();

private:
	/// The source DataGroup.
	const DataGroup& m_from;

	/// The target DataGroup.
	DataGroup& m_to;

	/// The map from source to target.
	SurgSim::DataStructures::DataGroupCopyMap m_map;
};

};  // namespace DataStructures
};  // namespace SurgSim


#endif  // SURGSIM_DATASTRUCTURES_DATAGROUPCOPIER_H
