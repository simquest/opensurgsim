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

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Log.h"

#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

using boost::filesystem::path;

namespace SurgSim
{
namespace Framework
{

ApplicationData::ApplicationData(const std::vector<std::string>& paths)
{
	setPaths(paths);
}

ApplicationData::ApplicationData(const std::string& configurationFileName)
{
	path filePath(configurationFileName);
	SURGSIM_ASSERT(boost::filesystem::exists(filePath)) <<
			"ApplicationdData could not find configuration file " << configurationFileName << " " <<
			"the application is probably not going to be able to find it's data files";

	std::vector<std::string> paths;
	boost::filesystem::ifstream in(filePath);
	std::string line;
	while (! in.eof())
	{
		getline(in, line);
		paths.push_back(line);
		// Skip possible Trailing newlines
		in >> std::ws;
	}
	setPaths(paths);
}

ApplicationData::~ApplicationData()
{
}

std::string ApplicationData::findFile(const std::string& fileName) const
{
	std::string result;

	if (!isValidFilename(fileName))
	{
		return "";
	}

	path file(fileName);
	if (file.is_absolute() && boost::filesystem::exists(file))
	{
		result = file.make_preferred().string();
	}
	else
	{
		for (auto it = m_paths.cbegin(); it != m_paths.cend(); ++it)
		{
			path filePath(*it);
			filePath /= fileName;
			if (boost::filesystem::exists(filePath))
			{
				result = filePath.make_preferred().string();
				break;
			}
		}
	}
	return result;
}


bool ApplicationData::tryFindFile(const std::string& fileName, std::string* target) const
{
	bool result = false;
	std::string resultName = findFile(fileName);
	if (resultName != "")
	{
		*target = std::move(resultName);
		result = true;
	}
	return result;
}


bool ApplicationData::setPaths(const std::vector<std::string>& paths)
{
	bool result = true;
	m_paths.clear();
	for (auto it = paths.cbegin(); it != paths.cend(); ++it)
	{
		result = addPath(*it) && result;
	}
	return result;
}

bool ApplicationData::addPath(const std::string& pathName)
{
	bool result = false;

	if (! isValidFilename(pathName))
	{
		return false;
	}

	path newPath(pathName);
	if (boost::filesystem::exists(newPath) && boost::filesystem::is_directory(newPath))
	{
		newPath = boost::filesystem::canonical(newPath).make_preferred();
		if (std::find(m_paths.cbegin(), m_paths.cend(), newPath) == m_paths.cend())
		{
			m_paths.push_back(newPath);
			result = true;
		}
		else
		{
			SURGSIM_LOG_INFO(Logger::getDefaultLogger()) <<
					"ApplicationsData::addPath: Trying to add duplicate path " << pathName;
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) <<
				"ApplicationData, trying to add nonexistent or non directory path to search list " << newPath;
	}
	return result;
}

std::vector<std::string> ApplicationData::getPaths() const
{
	std::vector<std::string> result;
	for (auto it = m_paths.cbegin(); it != m_paths.cend(); ++it)
	{
		result.push_back(it->string());
	}
	return result;
}

bool ApplicationData::isValidFilename(const std::string& fileName) const
{
	bool result = true;

	if (fileName.empty())
	{
		result = false;
	}

	size_t index = fileName.find("\\");
	if (index != std::string::npos)
	{
		SURGSIM_LOG_WARNING(Logger::getDefaultLogger()) << __FUNCTION__ <<
				" Backslashes encountered in the path, this path cannot be used " << fileName <<
				" to be useful it needs to be rewritten using '/'.";
		result = false;
	}
	return result;
}

}; // Framework
}; // SurgSim
