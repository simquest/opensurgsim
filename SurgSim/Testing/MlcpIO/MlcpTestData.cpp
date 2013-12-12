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

#include <iomanip>

#include "SurgSim/Testing/MlcpIO/MlcpTestData.h"
#include "SurgSim/Testing/MlcpIO/ReadText.h"

std::shared_ptr<MlcpTestData> loadTestData(const std::string& fileName)
{
	std::shared_ptr<MlcpTestData> data = std::make_shared<MlcpTestData>();
	if (! readMlcpTestDataAsText("MlcpTestData/" + fileName, data.get()))
	{
		data.reset();
	}
	return data;
}

std::string getTestFileName(const std::string& prefix, int index, const std::string& suffix)
{
	std::ostringstream stream;
	stream << prefix << std::setfill('0') << std::setw(3) << index << suffix;
	return stream.str();
}
