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

#ifndef SURGSIM_GRAPHICS_OSGTRACKBALLZOOMMANIPULATOR_H
#define SURGSIM_GRAPHICS_OSGTRACKBALLZOOMMANIPULATOR_H

#include <osgGA/TrackballManipulator>



namespace SurgSim
{
namespace Graphics
{

/// Trackball manipulator that uses the mouse wheel to control zoom amount.
/// The output matrices are view matrices and do not include the zoom.
/// To apply the zoom, get the value from getZoomFactor() and multiply
/// it by some base FOV to calculate the FOV for the current zoom level.
class OsgTrackballZoomManipulator : public osgGA::TrackballManipulator
{
public:
	/// Initializes the zoom parameters to default values
	OsgTrackballZoomManipulator();

	/// Sets the minimum zoom factor (zoomed out)
	/// \param factor Minimum zoom factor
	virtual void setMinZoomFactor(double factor);

	/// Gets the minimum zoom factor
	/// \return Minimum zoom factor
	double getMinZoomFactor() const;

	/// Sets the maximum zoom factor (zoomed out)
	/// \param factor Maximum zoom factor
	virtual void setMaxZoomFactor(double factor);

	/// Gets the maximum zoom factor
	/// \return Maximum zoom factor
	double getMaxZoomFactor() const;

	/// Sets the minimum amount to change the zoom factor in one step
	/// \param amount Minimum zoom amount
	virtual void setMinZoomAmount(double amount);

	/// Gets the minimum amount to change the zoom factor in one step
	/// \return Minimum zoom factor
	double getMinZoomAmount() const;//

	/// Sets the maximum amount to change the zoom factor in one step
	/// \param amount Maximum zoom amount
	virtual void setMaxZoomAmount(double amount);

	/// Gets the maximum amount to change the zoom factor in one step
	/// \return Maximum zoom factor
	double getMaxZoomAmount() const;

	/// Sets the current zoom factor
	/// \param factor Zoom factor
	virtual void setZoomFactor(double factor);

	/// Gets the current zoom factor
	/// \return Zoom factor
	double getZoomFactor() const;

	/// Sets the scale applied to the zoom factor before it is applied to the FOV
	/// \param factor Scale applied to the zoom factor
	virtual void setZoomFactorScale(double factor);

	/// Gets the current zoom factor
	/// \return Scale applied to the zoom factor
	double getZoomFactorScale() const;

	/// Zoom by a percent of the difference between the current zoom amount and minimum zoom factor
	/// \param zoomPercent Percent to zoom by: positive values zoom out, negative values zoom in
	virtual void zoom(double zoomPercent);

	/// Removes roll of the camera, so that the top of the view is towards the Y direction.
	virtual void makeUpright();

protected:

	/// Minimum zoom factor value (zoomed in)
	double m_minZoomFactor;
	/// Maximum zoom factor value (zoomed out)
	double m_maxZoomFactor;

	/// Minimum amount to change the zoom factor in one step
	/// This minimum prevents zooming by infinitely smaller amounts.
	double m_minZoomAmount;

	/// Maximum amount to change the zoom factor in one step
	double m_maxZoomAmount;

	/// Current zoom factor
	/// Larger values are zoomed out, smaller values are zoomed in.
	double m_zoomFactor;

	/// Scaling factor applied to the zoom factor before it is applied to the FOV
	double m_zoomFactorScale;

	/// Handle keyboard CTRL-U events to make the view upright
	/// \param eventAdapter Event adapter
	/// \param actionAdapter Action adapter
	/// \return true if the event was handled, false otherwise
	virtual bool handle(const osgGA::GUIEventAdapter& eventAdapter, osgGA::GUIActionAdapter& actionAdapter); //NOLINT

	/// Handle mouse wheel scrolling to zoom in or out
	/// \param eventAdapter Event adapter
	/// \param actionAdapter Action adapter
	/// \return true if the mouse wheel was handled, false otherwise
	virtual bool handleMouseWheel(const osgGA::GUIEventAdapter& eventAdapter,
								  osgGA::GUIActionAdapter& actionAdapter); //NOLINT

	void updateCamera(osg::Camera& camera) override;

};

}; // namespace Graphics
}; // namespace SurgSim


#endif
