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

#ifndef SURGSIM_DEVICES_SIXENSE_SIXENSETHREAD_H
#define SURGSIM_DEVICES_SIXENSE_SIXENSETHREAD_H

#include <memory>
#include <string>

#include "SurgSim/Framework/BasicThread.h"

namespace SurgSim
{
namespace Device
{

class SixenseScaffold;

/// A class implementing the thread context for sampling Sixense devices.
/// \sa SurgSim::Device::SixenseScaffold
class SixenseThread : public SurgSim::Framework::BasicThread
{
public:
	explicit SixenseThread(SixenseScaffold* scaffold) :
		BasicThread("Sixense thread"),
		m_scaffold(scaffold)
	{
		setRate(120.0);  // The Hydra runs at 60Hz, but running at 60Hz here could introduce up to 16.6ms extra latency
	}

	virtual ~SixenseThread();

protected:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

private:
	SixenseScaffold* m_scaffold;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_SIXENSE_SIXENSETHREAD_H
