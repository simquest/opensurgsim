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

#ifndef SURGSIM_DATASTRUCTURE_PLYREADER_H
#define SURGSIM_DATASTRUCTURE_PLYREADER_H

#include <string>
#include <vector>

#include <SurgSim/DataStructures/TriangleMesh.h>

struct PlyFile;

namespace SurgSim
{
namespace DataStructures
{

class PlyReader 
{
public:

	enum Type
	{
		TYPE_INVALID = 0,
		TYPE_CHAR,
		TYPE_SHORT,
		TYPE_INT,
		TYPE_UNSIGNED_CHAR,
		TYPE_UNSIGNED_SHORT,
		TYPE_UNSIGNED_INT,
		TYPE_FLOAT,
		TYPE_DOUBLE,
		TYPE_COUNT
	};

	explicit PlyReader(std::string filename);
	
	virtual ~PlyReader();

	bool isValid();

	bool requestElement(std::string elementName, 
		std::function<void* (const std::string&, size_t)> startElementCallback,
		std::function<void (const std::string&)> processElementCallback,
		std::function<void (const std::string&)> endElementCallback);

	bool requestProperty(std::string elementName, std::string propertyName, int targetType, int dataOffset);
	bool requestProperty(std::string elementName, std::string propertyName, int targetType, int dataOffset, int countType, int countOffset);

	bool hasElement(std::string elemenetName);

	bool hasProperty(std::string elementName, std::string propertyName);

	bool isScalar(std::string elementName, std::string propertyName);

	void parseFile();
private:
	std::string m_filename;

	struct PropertyInfo
	{
		std::string propertyName;
		int targetType;
		int dataOffset;
		int countType;
		int countOffset;
	};

	struct ElementInfo
	{
		std::string name;
		std::function<void* (const std::string&, size_t)> startElementCallback;
		std::function<void (const std::string&)> processElementCallback;
		std::function<void (const std::string&)> endElementCallback;
		std::vector<PropertyInfo> requestedProperties;
	};

	std::unordered_map<std::string, ElementInfo> m_requestedElements;

	struct Data;
	std::unique_ptr<Data> m_data;
};


}
}

#endif
