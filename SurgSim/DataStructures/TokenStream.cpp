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

#include "SurgSim/DataStructures/TokenStream.h"

namespace SurgSim
{
namespace DataStructures
{

TokenStream::TokenStream(std::stringstream* stream)
	: m_stream(*stream)
{}

bool TokenStream::parse(int* var)
{
	return m_stream.good() && !(m_stream >> *var).fail() && !m_stream.bad();
}

bool TokenStream::parse(std::string* var)
{
	return m_stream.good() && !(m_stream >> *var).fail() && !m_stream.bad();
}

bool TokenStream::parse(double* var)
{
	return m_stream.good() && !(m_stream >> *var).fail() && !m_stream.bad();
}

} // namespace DataStructures
} // namespace SurgSim