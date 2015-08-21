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

#include "SurgSim/Graphics/OsgTrackballZoomManipulator.h"
#include <osgUtil/UpdateVisitor>

/// Calculate the key code value of Ctrl-{character}, given the uppercase character.
/// If a key is pressed while holding Ctrl, OSG "helpfully" gives you the key code of the control character
/// (i.e. ^A == 1) instead of the key itself ('A' == 65).
/// To cope with this, you can use CONTROL_CHAR_FROM_UPPERCASE('A') which is easier to read than
/// strange character ('\001') or integral (1) constants.
#define CONTROL_CHAR_FROM_UPPERCASE(uppercaseCharacter)   ((uppercaseCharacter) - ('A' - 1))

namespace SurgSim
{
namespace Graphics
{

OsgTrackballZoomManipulator::OsgTrackballZoomManipulator() :
	osgGA::TrackballManipulator(),
	m_minZoomFactor(0.05),
	m_maxZoomFactor(1.0),
	m_minZoomAmount(0.01),
	m_maxZoomAmount(1.0),
	m_zoomFactor(1.0),
	m_zoomFactorScale(1.0)
{
}

void OsgTrackballZoomManipulator::setMinZoomFactor(double factor)
{
	m_minZoomFactor = factor;
}
double OsgTrackballZoomManipulator::getMinZoomFactor() const
{
	return m_minZoomFactor;
}

void OsgTrackballZoomManipulator::setMaxZoomFactor(double factor)
{
	m_maxZoomFactor = factor;
}
double OsgTrackballZoomManipulator::getMaxZoomFactor() const
{
	return m_maxZoomFactor;
}

void OsgTrackballZoomManipulator::setMinZoomAmount(double amount)
{
	m_minZoomAmount = amount;
}
double OsgTrackballZoomManipulator::getMinZoomAmount() const
{
	return m_minZoomAmount;
}

void OsgTrackballZoomManipulator::setMaxZoomAmount(double amount)
{
	m_maxZoomAmount = amount;
}
double OsgTrackballZoomManipulator::getMaxZoomAmount() const
{
	return m_maxZoomAmount;
}

void OsgTrackballZoomManipulator::setZoomFactor(double factor)
{
	m_zoomFactor = factor;
}
double OsgTrackballZoomManipulator::getZoomFactor() const
{
	return m_zoomFactor;
}

void OsgTrackballZoomManipulator::setZoomFactorScale(double factor)
{
	m_zoomFactorScale = factor;
}
double OsgTrackballZoomManipulator::getZoomFactorScale() const
{
	return m_zoomFactorScale;
}

void OsgTrackballZoomManipulator::zoom(double zoomPercent)
{
	double difference = m_zoomFactor - m_minZoomFactor;

	if (difference < m_minZoomAmount)
	{
		difference = m_minZoomAmount;
	}
	else if (difference > m_maxZoomAmount)
	{
		difference = m_maxZoomAmount;
	}

	m_zoomFactor += difference * zoomPercent;

	if (m_zoomFactor < m_minZoomFactor)
	{
		m_zoomFactor = m_minZoomFactor;
	}
	else if (m_zoomFactor > m_maxZoomFactor)
	{
		m_zoomFactor = m_maxZoomFactor;
	}
}

void OsgTrackballZoomManipulator::makeUpright()
{
	osg::Matrixd rotationMatrix(getRotation());

	// Remove any roll from the camera pose.
	osg::Vec3d backwards(rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2));

	osg::Vec3d right = osg::Vec3d(0.0, 1.0, 0.0) ^ backwards;
	if (right.length2() < 1e-12)
	{
		right.set(0.0, 0.0, 1.0);  // arbitrary...
	}
	right.normalize();

	osg::Vec3d up = backwards ^ right; // shouldn't need to be normalized

	for (int c = 0; c < 3; ++c)
	{
		rotationMatrix(0, c) = right[c];
	}
	for (int c = 0; c < 3; ++c)
	{
		rotationMatrix(1, c) = up[c];
	}

	setRotation(rotationMatrix.getRotate());
}

bool OsgTrackballZoomManipulator::handle(
	const osgGA::GUIEventAdapter& eventAdapter,
	osgGA::GUIActionAdapter& actionAdapter)
{
	unsigned int mask = eventAdapter.getModKeyMask();

	switch (eventAdapter.getEventType())
	{
	case (osgGA::GUIEventAdapter::KEYDOWN):
		{
			int key = eventAdapter.getKey();
			switch (key)
			{
			case 'u':
			case 'U':
			case CONTROL_CHAR_FROM_UPPERCASE('U'):
				{
					// ctrl-U makes the camera upright, so that the up axis points up
					if ((mask & osgGA::GUIEventAdapter::MODKEY_CTRL) &&
						!(mask & osgGA::GUIEventAdapter::MODKEY_ALT) &&
						!(mask & osgGA::GUIEventAdapter::MODKEY_SHIFT))
					{
						makeUpright();
					}
				}
				break;
			}
		}
	default:
		break;
	}

	return osgGA::TrackballManipulator::handle(eventAdapter, actionAdapter);
}

bool OsgTrackballZoomManipulator::handleMouseWheel(
	const osgGA::GUIEventAdapter& eventAdapter,
	osgGA::GUIActionAdapter& actionAdapter)
{
	osgGA::GUIEventAdapter::ScrollingMotion scrollingMotion = eventAdapter.getScrollingMotion();

	switch (scrollingMotion)
	{
	// Scroll up to zoom in
	case osgGA::GUIEventAdapter::SCROLL_UP:
		{
			zoom(-_wheelZoomFactor);
			actionAdapter.requestRedraw();
			actionAdapter.requestContinuousUpdate(false);
			return true;
		}
	// Scroll down to zoom out
	case osgGA::GUIEventAdapter::SCROLL_DOWN:
		{
			zoom(_wheelZoomFactor);
			actionAdapter.requestRedraw();
			actionAdapter.requestContinuousUpdate(false);
			return true;
		}
	default:
		return false;
	}
}

void OsgTrackballZoomManipulator::updateCamera(osg::Camera& camera)
{
	return;
}

}; // namespace Graphics
}; // namespace SurgSim
