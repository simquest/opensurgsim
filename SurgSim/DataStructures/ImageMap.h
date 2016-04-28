// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DATASTRUCTURES_IMAGEMAP_H
#define SURGSIM_DATASTRUCTURES_IMAGEMAP_H

#include "SurgSim/DataStructures/ImageBase.h"


namespace SurgSim
{
namespace DataStructures
{

/// A class that behaves like an Image but maps an existing array of data
///
/// \tparam T the data type of the image data
template<class T>
class ImageMap : public ImageBase<T>
{
public:
	/// Constructor
	/// \param width the image width
	/// \param height the image height
	/// \param channels the number of channels in the image
	/// \param data pointer to the array to map
	ImageMap(size_t width, size_t height, size_t channels, T* data);

	T* const getData() override;

	const T* const getData() const override;

private:
	T* m_data;
};

};
};

#include "SurgSim/DataStructures/ImageMap-inl.h"

#endif //SURGSIM_DATASTRUCTURES_IMAGEMAP_H
