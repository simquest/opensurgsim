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

#ifndef SURGSIM_GRAPHICS_MOCKCAMERA_H
#define SURGSIM_GRAPHICS_MOCKCAMERA_H

#include <gmock/gmock.h>
#include <string>
#include <SurgSim/Graphics/Camera.h>
#include <SurgSim/Graphics/Group.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/Material.h>


namespace SurgSim
{
namespace Graphics
{

class GMockCamera : public Camera
{
public:
	GMockCamera() : Camera("GMockCamera"), Representation("GMockCamera") {}
	
	// Mock Representation Methods
	MOCK_METHOD1(setInitialPose, void(const SurgSim::Math::RigidTransform3d&));
	MOCK_CONST_METHOD0(getInitialPose, SurgSim::Math::RigidTransform3d&());
	MOCK_METHOD1(setVisible, void(bool));
	MOCK_CONST_METHOD0(isVisible, bool());

	// Mock Camera Methods
	MOCK_METHOD1(setViewMatrix, void(const SurgSim::Math::Matrix44d& matrix));
	MOCK_CONST_METHOD0(getViewMatrix, SurgSim::Math::Matrix44d&());
	MOCK_METHOD1(setProjectionMatrix, void(const SurgSim::Math::Matrix44d&));
	MOCK_CONST_METHOD0(getProjectionMatrix, SurgSim::Math::Matrix44d&());
	MOCK_METHOD1(setRenderTarget, void(std::shared_ptr<RenderTarget>));
	MOCK_CONST_METHOD0(getRenderTarget, std::shared_ptr<RenderTarget>());
	MOCK_METHOD1(setPose, void(const SurgSim::Math::RigidTransform3d&));
	MOCK_CONST_METHOD0(getPose, SurgSim::Math::RigidTransform3d&());
	MOCK_METHOD1(setMaterial, bool(std::shared_ptr<Material>));
	MOCK_CONST_METHOD0(getMaterial, std::shared_ptr<Material>());
	MOCK_METHOD0(clearMaterial, void());
	MOCK_METHOD1(update, void(double));
	MOCK_METHOD2(setRenderOrder, void(int, int));
};

class GMockGroup : public Group
{
public:
	GMockGroup() : Group("GMockGroup") {}

	MOCK_METHOD1(setVisible, void(bool));
	MOCK_CONST_METHOD0(isVisible, bool());
};

}; // Graphics
}; // SurgSim

#endif