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

/// \file
/// Tests for the ImageMap class.

#include "gtest/gtest.h"

#include "SurgSim/DataStructures/ImageMap.h"


namespace
{
double epsilon = 1e-10;
}

namespace SurgSim
{
namespace DataStructures
{

template <class T>
class ImageMapTests : public testing::Test
{
public:
	typedef T Scalar;
};

typedef ::testing::Types<unsigned char, char, unsigned int, int, float, double> ImageMapTestTypes;
TYPED_TEST_CASE(ImageMapTests, ImageMapTestTypes);

TYPED_TEST(ImageMapTests, Construct)
{
	typedef typename TestFixture::Scalar T;

	T array[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ASSERT_NO_THROW({ImageMap<T> image(3, 3, 1, array);});
}

TYPED_TEST(ImageMapTests, Copy)
{
	typedef typename TestFixture::Scalar T;
	T array[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ImageMap<T> image(3, 3, 1, array);
	ImageMap<T> newImage(image);
	EXPECT_EQ(image.getSize(), newImage.getSize());
	EXPECT_EQ(image.getData(), newImage.getData());
}

TYPED_TEST(ImageMapTests, Assign)
{
	typedef typename TestFixture::Scalar T;
	T array[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ImageMap<T> image(3, 3, 1, array);

	T newArray[] = {0, 1, 2, 3};
	ImageMap<T> newImage(2, 2, 1, newArray);

	EXPECT_NE(image.getSize(), newImage.getSize());
	EXPECT_NE(image.getData(), newImage.getData());
	newImage = image;
	EXPECT_EQ(image.getSize(), newImage.getSize());
	EXPECT_EQ(image.getData(), newImage.getData());
}

TYPED_TEST(ImageMapTests, Accessors)
{
	typedef typename TestFixture::Scalar T;

	T array[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
	ImageMap<T> image(3, 3, 1, array);
	EXPECT_EQ(3u, image.getWidth());
	EXPECT_EQ(3u, image.getHeight());
	EXPECT_EQ(1u, image.getNumChannels());

	std::array<size_t, 3> size = {3, 3, 1};
	EXPECT_EQ(size, image.getSize());

	for (int i = 0; i < 9; i++)
	{
		EXPECT_NEAR(array[i], image.getData()[i], epsilon);
	}
}

TYPED_TEST(ImageMapTests, Move)
{
	typedef typename TestFixture::Scalar T;

	{
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
		ImageMap<T> oldImage(3, 3, 1, array);
		T* const dataPtr = oldImage.getData();
		ImageMap<T> newImage = std::move(oldImage);

		EXPECT_EQ(dataPtr, newImage.getData());
		EXPECT_EQ(3u, newImage.getWidth());
		EXPECT_EQ(3u, newImage.getHeight());
		EXPECT_EQ(1u, newImage.getNumChannels());
	}
	{
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
		ImageMap<T> oldImage(3, 1, 3, array);
		T* const dataPtr = oldImage.getData();
		ImageMap<T> newImage(std::move(oldImage));

		EXPECT_EQ(dataPtr, newImage.getData());
		EXPECT_EQ(3u, newImage.getWidth());
		EXPECT_EQ(1u, newImage.getHeight());
		EXPECT_EQ(3u, newImage.getNumChannels());
	}
}

TYPED_TEST(ImageMapTests, ArrayMapping)
{
	typedef typename TestFixture::Scalar T;

	{
		SCOPED_TRACE("Modifying the array should modify the data in the ImageMap");
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7};
		ImageMap<T> image(2, 2, 2, array);
		array[2] = 20;
		EXPECT_NEAR(20, image.getData()[2], epsilon);
	}
	{
		SCOPED_TRACE("Modifying the ImageMap with getData should modify the array");
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7};
		ImageMap<T> image(2, 2, 2, array);
		image.getData()[3] = 30;
		EXPECT_NEAR(30, array[3], epsilon);
	}
	{
		SCOPED_TRACE("Modifying the ImageMap with operator() should modify the array");
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7};
		ImageMap<T> image(2, 2, 2, array);
		image(1, 1) << 100, 101;
		EXPECT_NEAR(100, array[6], epsilon);
		EXPECT_NEAR(101, array[7], epsilon);
	}
	{
		SCOPED_TRACE("Modifying the ImageMap with getChannel() should modify the array");
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7};
		ImageMap<T> image(2, 2, 2, array);
		image.getChannel(1) << 100, 101, 110, 111;
		EXPECT_NEAR(100, array[1], epsilon);
		EXPECT_NEAR(110, array[3], epsilon);
		EXPECT_NEAR(101, array[5], epsilon);
		EXPECT_NEAR(111, array[7], epsilon);
	}
	{
		SCOPED_TRACE("Modifying the ImageMap with getAsVector() should modify the array");
		T array[] = {0, 1, 2, 3, 4, 5, 6, 7};
		ImageMap<T> image(2, 2, 2, array);
		image.getAsVector() << 100, 101, 102, 103, 104, 105, 106, 107;
		EXPECT_NEAR(100, array[0], epsilon);
		EXPECT_NEAR(101, array[1], epsilon);
		EXPECT_NEAR(102, array[2], epsilon);
		EXPECT_NEAR(103, array[3], epsilon);
		EXPECT_NEAR(104, array[4], epsilon);
		EXPECT_NEAR(105, array[5], epsilon);
		EXPECT_NEAR(106, array[6], epsilon);
		EXPECT_NEAR(107, array[7], epsilon);
	}
}

};
};
