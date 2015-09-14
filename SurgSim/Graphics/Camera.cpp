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

#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"

namespace SurgSim
{
namespace Graphics
{

Camera::Camera(const std::string& name) : Representation(name)
{
	typedef std::array<int, 2> ParamType;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Camera, SurgSim::Math::Matrix44d, ProjectionMatrix,
									  getProjectionMatrix, setProjectionMatrix);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Camera, std::string, RenderGroupReference,
									  getRenderGroupReference, setRenderGroupReference);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Camera, SurgSim::Math::Vector4d, AmbientColor,
									  getAmbientColor, setAmbientColor);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(Camera, ParamType, ViewportSize,
									  getViewportSize, setViewportSize);

	SURGSIM_ADD_RO_PROPERTY(Camera, SurgSim::Math::Matrix44d, ViewMatrix, getViewMatrix);
	SURGSIM_ADD_RO_PROPERTY(Camera, SurgSim::Math::Matrix44f, FloatViewMatrix, getViewMatrix);
	SURGSIM_ADD_RO_PROPERTY(Camera, SurgSim::Math::Matrix44f, FloatProjectionMatrix, getProjectionMatrix);
	SURGSIM_ADD_RO_PROPERTY(Camera, SurgSim::Math::Matrix44f, FloatInverseViewMatrix, getInverseViewMatrix);
	SURGSIM_ADD_RO_PROPERTY(Camera, SurgSim::Math::Matrix44f, FloatInverseProjectionMatrix, getInverseProjectionMatrix);

	{
		typedef std::array<double, 4> ParamType;
		SURGSIM_ADD_SETTER(Camera, ParamType, PerspectiveProjection, setPerspectiveProjection);
	}
	{
		typedef std::array<double, 6> ParamType;
		SURGSIM_ADD_SETTER(Camera, ParamType, OrthogonalProjection, setOrthogonalProjection);
	}
	{
		typedef std::array<int, 4> ParamType;

		// Deal with the overloaded function, by casting to explicit function type
		auto getter = (ParamType(Camera::*)(void) const)&Camera::getViewport;
		auto setter = (void(Camera::*)(ParamType))&Camera::setViewport;

		setAccessors("Viewport", std::bind(getter, this),
					 std::bind(setter, this, std::bind(SurgSim::Framework::convert<ParamType>, std::placeholders::_1)));

		setSerializable("Viewport",
						std::bind(&YAML::convert<ParamType>::encode, std::bind(getter, this)),
						std::bind(setter, this, std::bind(&YAML::Node::as<ParamType>, std::placeholders::_1)));
	}
}

void Camera::setRenderGroupReference(const std::string& name)
{
	removeGroupReference(name);
	m_renderGroupReference = name;
}

std::string Camera::getRenderGroupReference() const
{
	return m_renderGroupReference;
}

bool Camera::setRenderGroup(std::shared_ptr<Group> group)
{
	m_group = group;
	return true;
}

std::shared_ptr<Group> Camera::getRenderGroup() const
{
	return m_group;
}

bool Camera::addGroupReference(const std::string& name)
{
	bool result = false;
	if (name != m_renderGroupReference)
	{
		result = Representation::addGroupReference(name);
	}
	return result;
}

void Camera::setPerspectiveProjection(const std::array<double, 4>& val)
{
	setPerspectiveProjection(val[0], val[1], val[2], val[3]);
}

void Camera::setOrthogonalProjection(const std::array<double, 6>& val)
{

	setOrthogonalProjection(val[0], val[1], val[2], val[3], val[4], val[5]);
}

void Camera::setViewport(std::array<int, 4> val)
{
	setViewport(val[0], val[1], val[2], val[3]);
}

std::array<int, 4> Camera::getViewport() const
{
	std::array<int, 4> result;
	getViewport(&result[0], &result[1], &result[2], &result[3]);
	return result;
}

bool Camera::doInitialize()
{
	SURGSIM_ASSERT(m_renderGroupReference != "")
			<< "Can't have a camera without a RenderGroupReference.";
	return true;
}

}
}

