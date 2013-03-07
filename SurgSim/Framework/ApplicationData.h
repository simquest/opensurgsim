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

#ifndef SURGSIM_FRAMEWORK_APPLICATIONDATA_H
#define SURGSIM_FRAMEWORK_APPLICATIONDATA_H

#include <string>
#include <vector>

namespace boost
{
namespace filesystem
{
	class path;
}
}
namespace SurgSim
{
namespace Framework
{

/// Enable searching for files in a given list of paths, give access to the current directory and
/// wrap boost::filesystem functionality
class ApplicationData 
{
public:

	/// Constructor, class is immutable, pass a list of paths to be used for searching duplicate
	/// paths will be eliminated, invalid paths will be eliminated, the order of the paths given here
	/// is the order that will be used in searching, the first occurrance of a file within a given
	/// path will be used.
	/// \param	paths	The list of search paths.
	explicit ApplicationData(const std::vector<std::string>& paths);

	/// Reads the search paths from a given configuration file
	/// \param	configFile	The configuration file for reading the search paths.
	explicit ApplicationData(const std::string& configFile);

	~ApplicationData();

	/// Gets the search paths.
	/// \return	The paths, passed in the constructor with duplicates and invalid paths
	/// 		removed. All paths returned will be absolute and in system format.
	std::vector<std::string> getPaths() const;

	/// Searches for the first occurrence of fileName amongst the given paths, the
	/// search is shallow, only direct content of the directories in the path list 
	/// will be used as search candidates. 
	/// \param	fileName	Filename of the file.
	/// \return	The absolute path to the file in system format i.e c:\\xxx\\yyy\\file.txt for 
	/// 		windows and /xxx/yyy/file.txt for all other systems. An empty string will be
	/// 		returned if the file cannot be found.
	std::string findFile(const std::string& fileName) const;

	/// Checks if the filename is acceptable
	/// \param fileName		Filename to be checked.
	/// \return true if the name is valid, false otherwise.
	bool isValidFilename(const std::string& fileName) const;

	/// Remove all occurances of '\' and replace them with '/'
	/// \param fileName	The file name to be changed.
	/// \return A useable file name
	std::string makeValid(const std::string& fileName) const;

private:

	/// Adds a single path to the list of search paths.
	/// \param	path	Full pathname.
	/// \return	true if it succeeds, false if the given path does not exist or if it is
	/// 		already in the list of paths.
	bool addPath(std::string pathName);

	/// Sets the list of search paths to be used for finding the location of files.
	/// Eliminates duplicate paths and paths that do not exist
	/// \param	paths	The paths to be used for finding files.
	/// \return	true if it succeeds, false if one or more of the paths 
	/// 		do not exist in the system or are duplicated.
	bool setPaths(const std::vector<std::string>& paths);

	std::vector<boost::filesystem::path> m_paths;
};

}; // Framework
}; // SurgSim

#endif
