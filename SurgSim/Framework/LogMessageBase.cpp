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

#include "SurgSim/Framework/LogMessageBase.h"

#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Framework
{

LogMessageBase::LogMessageBase(Logger* logger, int level)
	: m_stream(), m_logger(logger)
{
	SURGSIM_ASSERT(logger) << "logger should not be a null pointer";
	static std::string levelNames[5] = {"DEBUG   ", "INFO    ", "WARNING ", "SEVERE  ", "CRITICAL"};
	std::time_t timeStamp;
	std::time(&timeStamp);
	::tm tm;
#ifdef _MSC_VER
	localtime_s(&tm, &timeStamp);
#else
	localtime_r(&timeStamp, &tm);
#endif
	std::string levelName("NONE    ");
	if (level >= 0 && level <= LOG_LEVEL_CRITICAL)
	{
		levelName = levelNames[level];
	}
	char fillChar = m_stream.fill();
	m_stream << std::setfill('0') <<
			 std::setw(2) << 1 + tm.tm_mon << "." <<
			 std::setw(2) << tm.tm_mday << ' ' <<
			 std::setw(2) << tm.tm_hour << ':' <<
			 std::setw(2) << tm.tm_min << ':' <<
			 std::setw(2) << tm.tm_sec << ' ' <<
			 std::setfill(fillChar) <<
			 levelName << " " <<
			 m_logger->getName() << " ";
}

}; // namespace Framework
}; // namespace SurgSim
