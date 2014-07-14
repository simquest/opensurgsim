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

#ifndef SURGSIM_FRAMEWORK_ASSET_H
#define SURGSIM_FRAMEWORK_ASSET_H

#include <string>

namespace SurgSim
{
namespace Framework
{
class AssetTest;
class ApplicationData;

/// This class stores a relative file name and once a file name is set, it will try to load
/// the file using the static ApplicationData in SurgSim::Framework::Runtime.
/// Classes not in SurgSim::Framework::Component hierarchy should inherit this class in
/// order to load a file.
class Asset
{
	friend AssetTest;
public:
	/// Constructor
	Asset();

	/// Destructor
	virtual ~Asset();

	/// Set the file name to be loaded.
	/// \note Asset::setFileName() will try to load the file right after the file name is set.
	/// \param fileName Name of the file to be loaded.
	void setFileName(const std::string& fileName);

	/// Return the name of file loaded by this class.
	/// \return Name of the file loaded by this class.
	std::string getFileName() const;

	/// Check to see if an attempt has been made to load the file.
	/// \return true if initialization succeeded, false otherwise
	bool isInitialized() const;

protected:
	/// Check for existence of the resolved filename, return false if not found.
	/// If found, it then calls 'doInitialize()' to load the file. Return 'false' if 'doInitialize()' fails.
	/// It asserts on double calls.
	/// \param data Gives the locations to search for the file.
	/// \return true if file is found and loaded successfully; false otherwise.
	bool initialize(const ApplicationData& data);

	/// Derived classes will overwrite this method to do actual loading.
	/// \note This method is not required to do any check on the validity or the existence of the file.
	/// \return false if loading failed, assume filename != ""
	virtual bool doInitialize(const std::string& fileName) = 0;
	
private:
	/// Indicates if an attempt to load the file has been made.
	bool m_didInit;

	/// Indicates if load is successful.
	bool m_isInitialized;

	/// Name of the file to be loaded.
	std::string m_fileName;
};

} // namespace Framework
} // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_ASSET_H