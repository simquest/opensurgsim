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

#ifndef SURGSIM_DEVICE_SIXENSETHREAD_H
#define SURGSIM_DEVICE_SIXENSETHREAD_H

#include <memory>
#include <string>

#include <SurgSim/Framework/BasicThread.h>

namespace SurgSim
{
namespace Device
{

class SixenseManager;

/// A class implementing the thread context for sampling Sixense devices.
/// \sa SurgSim::Device::SixenseManager
class SixenseThread : public SurgSim::Framework::BasicThread
{
public:
	explicit SixenseThread(SixenseManager* manager) :
		BasicThread("Sixense thread"),
		m_manager(manager)
	{
		setRate(120.0);  // The Hydra runs at 60Hz, but running at 60Hz here could introduce up to 16.6ms extra latency
	}

	virtual ~SixenseThread();

	/// Starts the thread execution.
	void start()
	{
		// Start without waiting on a barrier.
		BasicThread::start(std::shared_ptr<SurgSim::Framework::Barrier>());
	}

protected:
	virtual bool doInitialize();
	virtual bool doStartUp();
	virtual bool doUpdate(double dt);

private:
	SixenseManager* m_manager;
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_SIXENSETHREAD_H
