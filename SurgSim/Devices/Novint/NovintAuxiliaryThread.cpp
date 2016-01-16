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

#include "SurgSim/Devices/Novint/NovintAuxiliaryThread.h"

#include <osg_interface.h>

#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{
NovintAuxiliaryThread::NovintAuxiliaryThread(NovintScaffold* scaffold) :
	BasicThread("Devices/NovintAuxiliaryThread"),
	m_scaffold(scaffold),
	m_left(true),
	m_right(true)
{
}

NovintAuxiliaryThread::~NovintAuxiliaryThread()
{
}

bool NovintAuxiliaryThread::doInitialize()
{
	bool result = true;
	if (m_left)
	{
		SURGSIM_ASSERT(osgConnectToGrip(OSG_GRIP_LEFT_HAND) == OSG_OK) << "NovintAuxiliaryThread failed to find the left-hand grip.";
		m_grips.push_back(OSG_GRIP_LEFT_HAND);
	}
	if (m_right)
	{
		SURGSIM_ASSERT(osgConnectToGrip(OSG_GRIP_RIGHT_HAND) == OSG_OK) << "NovintAuxiliaryThread failed to find the right-hand grip.";
		m_grips.push_back(OSG_GRIP_RIGHT_HAND);
	}
	return result;
}

bool NovintAuxiliaryThread::doStartUp()
{
	return true;
}

bool NovintAuxiliaryThread::doUpdate(double dt)
{
	for (int grip : m_grips)
	{
		double roll = -1.0;
		double toolDof = -1.0;
		if ((osgGetRollAngleInRadians(grip, roll) == OSG_OK) &&
			(osgGetGrasperAngleInRadians(grip, toolDof) == OSG_OK))
		{
			m_scaffold->setAuxiliary(grip, roll, toolDof);
		}
		else
		{
			SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Devices/Novint/AuxiliaryThread")) << "osgGetRollAngleInRadians reported error for grip " << grip;
			return false;
		}
	}
	return true;
}

};  // namespace Devices
};  // namespace SurgSim
