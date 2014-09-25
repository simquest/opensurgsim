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

template <typename T, int Rows>
bool TokenStream::parse(Eigen::Matrix<T, Rows, 1>* v)
{
	bool success = true;
	for (int i = 0; success && i < Rows; ++i)
	{
		success &= parse(&(*v)[i]);
	}
	return success;
}

template <typename T>
bool TokenStream::parse(Eigen::Quaternion<T>* q)
{
	return parse(&q->x()) && parse(&q->y()) && parse(&q->z()) && parse(&q->w());
}

} // namespace DataStructures
} // namespace SurgSim