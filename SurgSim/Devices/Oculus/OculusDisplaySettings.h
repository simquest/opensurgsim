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

#ifndef SURGSIM_DEVICES_OCULUS_OCULUSDISPLAYSETTINGS_H
#define SURGSIM_DEVICES_OCULUS_OCULUSDISPLAYSETTINGS_H

#include <string>

#include <osg/DisplaySettings>
#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{
namespace Device
{

/// A customized osg::DisplaySettings, to be used with Oculus device.
/// It pulls the projection matrices from OculusSDK and pass them to OSG for rendering.
class OculusDisplaySettings : public osg::DisplaySettings
{
public:
	OculusDisplaySettings();

	/// Constructor
	/// \param displaySettings An instance of osg::DisplaySettings
	explicit OculusDisplaySettings(const osg::DisplaySettings* displaySettings);

	void setLeftEyeProjectionMatrix(const SurgSim::Math::Matrix44d& matrix);
	void setRightEyeProjectionMatrix(const SurgSim::Math::Matrix44d& matrix);

	/// This method returns the projection matrix for the left eye pulled from OculusSDK.
	/// The parameter passed in is NOT used.
	osg::Matrixd computeLeftEyeProjectionImplementation(const osg::Matrixd&) const override;

	/// This method returns the projection matrix for the right eye pulled from OculusSDK.
	/// The parameter passed in is NOT used.
	osg::Matrixd computeRightEyeProjectionImplementation(const osg::Matrixd&) const override;

private:
	osg::Matrixd m_leftEyeProjectionMatrix;
	osg::Matrixd m_rightEyeProjectionMatrix;
};
}; // namespace Device
}; // namespace SurgSim

#endif  // SURGSIM_DEVICES_OCULUS_OCULUSDISPLAYSETTINGS_H