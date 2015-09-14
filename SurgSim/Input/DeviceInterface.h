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

#ifndef SURGSIM_INPUT_DEVICEINTERFACE_H
#define SURGSIM_INPUT_DEVICEINTERFACE_H

#include <memory>
#include <string>

#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Framework/Accessible.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{
namespace Input
{


/// Interface used to communicate with user-interface hardware devices.
///
/// Classes that implement communication with a hardware device implement this interface.  This includes
/// input/output devices (haptic devices are the most obvious example, but many other devices can, for example,
/// display visual indicators such as LEDs, etc.), as well as input-only and output-only devices.
///
/// Derived classes will likely want to hide their constructor and only
/// allow creation through a manager object for that type of device.
class DeviceInterface : public Framework::Accessible, public Framework::FactoryBase1<DeviceInterface, std::string>
{
public:
	/// Virtual destructor (empty).
	virtual ~DeviceInterface()
	{
	}

	/// Return a (hopefully unique) device name.
	virtual std::string getName() const = 0;

	/// The class name for this class
	/// \note Use the SURGSIM_CLASSNAME macro in derived classes.
	/// \return The fully namespace qualified name of this class.
	virtual std::string getClassName() const = 0;

	/// Fully initialize the device.
	///
	/// When the manager object creates the device, the internal state of the device usually isn't fully
	/// initialized yet.  This method performs any needed initialization.
	virtual bool initialize() = 0;

	/// Adds an input consumer that will be notified when the application input state is updated.
	///
	/// \param inputConsumer The input consumer to be added.
	/// \return true on success, false on failure.
	virtual bool addInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer) = 0;

	/// Removes an input consumer previously added via \ref addInputConsumer.
	/// \param inputConsumer The input consumer to be removed.
	virtual bool removeInputConsumer(std::shared_ptr<InputConsumerInterface> inputConsumer) = 0;

	/// Removes all InputConsumers.
	virtual void clearInputConsumers() = 0;

	/// Sets an output producer that will be asked for application output state when the device needs it.
	/// Any previously set output producer will be removed.
	///
	/// \param outputProducer The output producer to be added.
	/// \return true on success, false on failure.
	virtual bool setOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer) = 0;

	/// Removes an output producer previously added via \ref setOutputProducer.
	/// \param outputProducer The output producer to be removed.
	virtual bool removeOutputProducer(std::shared_ptr<OutputProducerInterface> outputProducer) = 0;

	/// Query if this object has output producer.
	/// \return	true if there is an output producer, false if not.
	virtual bool hasOutputProducer() = 0;

	/// Removes any OutputProducer
	virtual void clearOutputProducer() = 0;

protected:
	/// Finalize (de-initialize) the device.
	virtual bool finalize() = 0;
};


};  // namespace Input
};  // namespace SurgSim

#endif // SURGSIM_INPUT_DEVICEINTERFACE_H
