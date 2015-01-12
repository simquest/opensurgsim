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

#ifndef SURGSIM_FRAMEWORK_LOGOUTPUT_H
#define SURGSIM_FRAMEWORK_LOGOUTPUT_H

#include <fstream>
#include <boost/thread/mutex.hpp>

namespace SurgSim
{
namespace Framework
{

/// Virtual Base class to define an interface for outputting logging information
class LogOutput
{
public:
	LogOutput()
	{
	}

	virtual ~LogOutput()
	{
	}

	/// \param message to be written out
	/// \return true on success
	virtual bool writeMessage(const std::string& message) = 0;

};

class NullOutput : public LogOutput
{
public:
	virtual bool writeMessage(const std::string& message) {return true;}
};


/// Class to output logging information to a give file
class FileOutput : public LogOutput
{
public:

	/// Constructor
	/// \param filename The filename to be used for writing
	explicit FileOutput(const std::string& filename);

	virtual ~FileOutput();

	/// \param message to be written out
	/// \return true on success
	bool writeMessage(const std::string& message) override;

private:
	std::string m_filename;
	std::ofstream m_stream;
	boost::mutex m_mutex;
};

/// Class to output logging information to a stream that can be passed
/// into the constructor of the class
class StreamOutput : public LogOutput
{
public:

	/// Constructor
	/// \param ostream stream to be used for writing
	/// ostream parameter to be passed by non-const reference on purpose.
	explicit StreamOutput(std::ostream& ostream); //NOLINT
	virtual ~StreamOutput();

	/// Writes a message to the stream.
	/// \param	message	Message to be written to the stream
	/// \return	True on success
	bool writeMessage(const std::string& message) override;

private:
	std::ostream& m_stream;
	boost::mutex m_mutex;
};

}; // namespace Framework
}; // namespace SurgSim

#endif // SURGSIM_FRAMEWORK_LOGOUTPUT_H
