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

#include "SurgSim/Devices/Oculus/OculusDisplaySettings.h"

#include "SurgSim/Graphics/OsgMatrixConversions.h"

namespace SurgSim
{
namespace Devices
{

OculusDisplaySettings::OculusDisplaySettings() : m_leftEyeProjectionMatrix(osg::Matrixd()),
												 m_rightEyeProjectionMatrix(osg::Matrixd())
{
}

OculusDisplaySettings::OculusDisplaySettings(const osg::DisplaySettings* displaySettings) :
	osg::DisplaySettings(*displaySettings),
	m_leftEyeProjectionMatrix(osg::Matrixd()),
	m_rightEyeProjectionMatrix(osg::Matrixd())
{
}

void OculusDisplaySettings::setLeftEyeProjectionMatrix(const SurgSim::Math::Matrix44d& matrix)
{
	m_leftEyeProjectionMatrix = SurgSim::Graphics::toOsg(matrix);
}

SurgSim::Math::Matrix44d OculusDisplaySettings::getLeftEyeProjectionMatrix() const
{
	return SurgSim::Graphics::fromOsg(m_leftEyeProjectionMatrix);
}

void OculusDisplaySettings::setRightEyeProjectionMatrix(const SurgSim::Math::Matrix44d& matrix)
{
	m_rightEyeProjectionMatrix = SurgSim::Graphics::toOsg(matrix);
}

SurgSim::Math::Matrix44d OculusDisplaySettings::getRightEyeProjectionMatrix() const
{
	return SurgSim::Graphics::fromOsg(m_rightEyeProjectionMatrix);
}

osg::Matrixd OculusDisplaySettings::computeLeftEyeProjectionImplementation(const osg::Matrixd&) const
{
	return m_leftEyeProjectionMatrix;
}

osg::Matrixd OculusDisplaySettings::computeRightEyeProjectionImplementation(const osg::Matrixd&) const
{
	return m_rightEyeProjectionMatrix;
}

}; // namespace Devices
}; // namespace SurgSim