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

#include "SurgSim/Framework/LogOutput.h"

#include <boost/thread/locks.hpp>
#include <fstream>

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Logger.h"


namespace SurgSim
{
namespace Framework
{

FileOutput::FileOutput(const std::string& filename) :
	m_filename(filename)
{
	if (! m_stream.is_open())
	{
		m_stream.open(m_filename,std::ios_base::app);
	}

	SURGSIM_ASSERT(! m_stream.fail()) << "Failed to open '" << m_filename << "'!";
}

bool SurgSim::Framework::FileOutput::writeMessage(const std::string& message)
{
	SURGSIM_ASSERT(m_stream.is_open() && ! m_stream.fail()) <<
		"Error writing to " << m_filename;
	{
		boost::mutex::scoped_lock lock(m_mutex);
		m_stream << message << std::endl;
	}
	return true;
}


StreamOutput::StreamOutput(std::ostream& ostream) : m_stream(ostream) //NOLINT
{
}

bool StreamOutput::writeMessage(const std::string& message)
{
	bool result = false;
	if (!m_stream.fail())
	{
		boost::mutex::scoped_lock lock(m_mutex);
		m_stream << message << std::endl;
		result = true;
	}
	else
	{
		//TODO(hscheirich) 2013-01-28: Still need to figure out default logging
		throw("Default logging not implemented yet");
	}
	return result;
}

}; // namespace Framework
}; // namespace SurgSim


