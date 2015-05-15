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

#include <osg/DisplaySettings>

#include "SurgSim/Math/Matrix.h"

namespace SurgSim
{
namespace Device
{

/// A customized osg::DisplaySettings, to be used with Oculus device.
/// It passes customized projection matrices to OSG for rendering.
class OculusDisplaySettings : public osg::DisplaySettings
{
public:
	/// Constructor
	OculusDisplaySettings();

	/// Constructor
	/// \param displaySettings An instance of osg::DisplaySettings
	explicit OculusDisplaySettings(const osg::DisplaySettings* displaySettings);

	/// Set the projection matrix of the left eye
	/// \param matrix Projection matrix for left eye
	void setLeftEyeProjectionMatrix(const SurgSim::Math::Matrix44d& matrix);

	/// Get the projection matrix of the left eye
	/// \return Projection matrix for left eye
	SurgSim::Math::Matrix44d getLeftEyeProjectionMatrix() const;

	/// Set the projection matrix of the right eye
	/// \param matrix Projection matrix for right eye
	void setRightEyeProjectionMatrix(const SurgSim::Math::Matrix44d& matrix);

	/// Get the projection matrix of the right eye
	/// \return Projection matrix for right eye
	SurgSim::Math::Matrix44d getRightEyeProjectionMatrix() const;

	/// This method returns the projection matrix set by setLeftEyeProjectionMatrix() method.
	/// OSG calls this overriding function to get the left eye projection matrix to use.
	/// The parameter passed in is NOT used.
	osg::Matrixd computeLeftEyeProjectionImplementation(const osg::Matrixd&) const override;

	/// This method returns the projection matrix set by setRighttEyeProjectionMatrix() method.
	/// OSG calls this overiding function to get the right eye projection matrix to use.
	/// The parameter passed in is NOT used.
	osg::Matrixd computeRightEyeProjectionImplementation(const osg::Matrixd&) const override;

private:
	/// Left eye projection matrix
	osg::Matrixd m_leftEyeProjectionMatrix;

	/// Right eye projection matrix
	osg::Matrixd m_rightEyeProjectionMatrix;
};

}; // namespace Device
}; // namespace SurgSim

#endif  // SURGSIM_DEVICES_OCULUS_OCULUSDISPLAYSETTINGS_H