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

#include "SurgSim/Devices/Novint/NovintAncillaryThread.h"

#include <osg_interface.h>

#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{
NovintAncillaryThread::NovintAncillaryThread(NovintScaffold* scaffold) :
	BasicThread("Devices/NovintAncillaryThread"),
	m_scaffold(scaffold),
	m_left(true),
	m_right(true)
{
}

NovintAncillaryThread::~NovintAncillaryThread()
{
	if (m_left)
	{
		osgDisconnectFromGrip(OSG_GRIP_LEFT_HAND);
	}
	if (m_right)
	{
		osgDisconnectFromGrip(OSG_GRIP_RIGHT_HAND);
	}
}

bool NovintAncillaryThread::doInitialize()
{
	bool result = true;
	if (m_left)
	{
		SURGSIM_ASSERT(osgConnectToGrip(OSG_GRIP_LEFT_HAND, true, -1) == OSG_OK) <<
			"NovintAncillaryThread failed to find the left-hand grip.";
		m_grips.push_back(OSG_GRIP_LEFT_HAND);
	}
	if (m_right)
	{
		SURGSIM_ASSERT(osgConnectToGrip(OSG_GRIP_RIGHT_HAND, true, -1) == OSG_OK) <<
			"NovintAncillaryThread failed to find the right-hand grip.";
		m_grips.push_back(OSG_GRIP_RIGHT_HAND);
	}
	return result;
}

bool NovintAncillaryThread::doStartUp()
{
	return true;
}

bool NovintAncillaryThread::doUpdate(double dt)
{
	for (int grip : m_grips)
	{
		double roll = -1.0;
		double toolDof = -1.0;
		if ((osgGetRollAngleInRadians(grip, roll) == OSG_OK) &&
			(osgGetGrasperAngleInRadians(grip, toolDof) == OSG_OK))
		{
			m_scaffold->setAncillary(grip, roll, toolDof);
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "osgGetRollAngleInRadians reported error for grip " << grip;
			return false;
		}
	}
	return true;
}

};  // namespace Devices
};  // namespace SurgSim
