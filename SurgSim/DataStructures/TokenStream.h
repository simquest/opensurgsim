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

#ifndef SURGSIM_DATASTRUCTURES_TOKENSTREAM_H
#define SURGSIM_DATASTRUCTURES_TOKENSTREAM_H

#include <sstream>

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"

namespace SurgSim
{
namespace DataStructures
{

/// A class to parse std::stringstream into requested data type.
/// The class is initialized with a std::stringstream and can parse it into requested datatype.
class TokenStream
{
public:
	/// Constructor.
	/// \param stream The string stream from which the values are parsed.
	explicit TokenStream(std::stringstream* stream);

	/// \param [out] var Parse stream into this int.
	/// \return True, if parsing is successful.
	bool parse(int* var);

	/// \param [out] var Parse stream into this string.
	/// \return True, if parsing is successful.
	bool parse(std::string* var);

	/// \param [out] var Parse stream into this double.
	/// \return True, if parsing is successful.
	bool parse(double* var);

	/// \param [out] v Parse stream into this variable size Eigen vector.
	/// \return True, if parsing is successful.
	template <typename T, int Rows>
	bool parse(Eigen::Matrix<T, Rows, 1>* v);

	/// \param [out] q Parse stream into this variable size Eigen quaternion.
	/// \return True, if parsing is successful.
	template <typename T>
	bool parse(Eigen::Quaternion<T>* q);

protected:
	/// The string stream from where the values are parsed.
	std::stringstream& m_stream;
};

} // namespace DataStructures
} // namespace SurgSim

#include "SurgSim/DataStructures/TokenStream-inl.h"

#endif // SURGSIM_DATASTRUCTURES_TOKENSTREAM_H