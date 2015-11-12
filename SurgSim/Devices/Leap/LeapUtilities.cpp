// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/Leap/LeapUtilities.h"


namespace SurgSim
{
namespace Devices
{

DataStructures::DataGroup::ImageType undistortLeapImage(const DataStructures::DataGroup::ImageType& image,
		const DataStructures::DataGroup::ImageType& distortion)
{
	DataStructures::DataGroup::ImageType result(image.getWidth(), image.getHeight(), image.getNumChannels());
	Eigen::Array2f distortionPosition;
	Eigen::Array2f weights;
	Eigen::Array2f size(result.getWidth(), result.getHeight());
	Eigen::Array2i corner;
	Eigen::RowVector2f preMultiply;
	Eigen::Vector2f postMultiply;
	Eigen::Array2f position;

	// Undistort the images using the Leap distortion map.
	// See Image::distortion() documentation for reference implementation:
	//   https://developer.leapmotion.com/documentation/cpp/api/Leap.Image.html
	for (size_t i = 0; i < result.getWidth(); i++)
	{
		for (size_t j = 0; j < result.getHeight(); j++)
		{
			distortionPosition << 63 * i / size[0], 62 * (1 - j / size[1]);
			corner = distortionPosition.template cast<int>();

			// Bilinear interpolation
			weights = distortionPosition - distortionPosition.unaryExpr(std::ptr_fun<float, float>(std::floor));
			preMultiply << 1 - weights[0], weights[0];
			postMultiply << 1 - weights[1], weights[1];
			position << preMultiply * distortion.getChannel(0).block<2, 2>(corner[0], corner[1]) * postMultiply,
						preMultiply * distortion.getChannel(1).block<2, 2>(corner[0], corner[1]) * postMultiply;

			if ((position >= 0).all() && (position <= 1.0).all())
			{
				position *= size;
				result(i, j) = image(static_cast<size_t>(position[0]), static_cast<size_t>(position[1]));
			}
			else
			{
				result(i, j).setConstant(0);
			}
		}
	}
	return result;
}

};
};


