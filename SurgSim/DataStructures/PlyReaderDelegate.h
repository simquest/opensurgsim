// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_PLYREADERDELEGATE_H
#define SURGSIM_DATASTRUCTURES_PLYREADERDELEGATE_H



namespace SurgSim
{
namespace DataStructures
{

class PlyReader;

/// PlyReaderDelegate abstract class.
/// The purpose of this class is to customize the parsing and contain the callback functions that
/// are being used in the parsing process.
class PlyReaderDelegate
{
public:

	/// Virtual destructor.
	virtual ~PlyReaderDelegate()
	{
	}

	/// Registers the delegate with the reader.
	/// \param [out] reader The reader that should be used by the delegate.
	/// \return true usually if the reader is valid and fileIsAcceptable() is true.
	virtual bool registerDelegate(PlyReader* reader) = 0;

	/// Check whether the file in the reader can be used with this delegate,
	/// this gives the delegate a chance to make sure that all the elements and 
	/// properties that are required are available in the file encapsulated by
	/// the reader
	virtual bool fileIsAcceptable(const PlyReader& reader) = 0;
};

}
}

#endif
