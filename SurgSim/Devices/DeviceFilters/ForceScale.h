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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_FORCESCALE_H
#define SURGSIM_DEVICES_DEVICEFILTERS_FORCESCALE_H

#include <boost/thread/mutex.hpp>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"

namespace SurgSim
{

namespace Device
{
/// An output device filter that scales forces and/or torques.  Any other entries in the DataGroup are passed through.
/// For convenience, it is also an InputConsumerInterface that does no filtering of the input data.  Thus it can be
/// added as an input consumer to the raw device, and set as the output producer for the raw device, after which other
/// device filters, input components, and output components only need access to the ForceScale instance, not the raw
/// device.
class ForceScale : public SurgSim::Input::CommonDevice,
		public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit ForceScale(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Device::ForceScale);

	/// Destructor.
	virtual ~ForceScale();

	/// Fully initialize the device.
	/// When the manager object creates the device, the internal state of the device usually isn't fully
	/// initialized yet.  This method performs any needed initialization.
	/// \return True on success.
	bool initialize() override;

	/// Set the initial input data.  Passes through all data unchanged.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	/// Notifies the consumer that the application input coming from the device has been updated.
	/// Passes through all data unchanged.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	/// Asks the producer to provide output state to the device.  Passes through all data, modifying certain data
	/// entries used by haptic devices to calculate force and torque.  Note that devices may never call this method,
	/// e.g., because the device doesn't actually have any output capability.
	/// \param device The name of the device that is requesting the output.  This should only be used to identify
	/// 	the device (e.g. if the producer is listening to several devices at once).
	/// \param [out] outputData The data being sent to the device.
	/// \return True if the producer has provided output data.  A producer that returns false should leave outputData
	///		unmodified.
	bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

	/// Set the force scale factor so that each direction has the same scale.
	/// \param forceScale The scalar scaling factor.
	void setForceScale(double forceScale);

	/// Set the torque scale factor so that each direction has the same scale.
	/// \param torqueScale The scalar scaling factor.
	void setTorqueScale(double torqueScale);

private:
	/// Finalize (de-initialize) the device.
	/// \return True on success.
	bool finalize() override;

	/// Filter the output data, scaling the forces and torques.
	/// \param dataToFilter The data that will be filtered.
	/// \param [in,out] result A pointer to a DataGroup object that must be assignable to by the dataToFilter object.
	///		Will contain the filtered data.
	void outputFilter(const SurgSim::DataStructures::DataGroup& dataToFilter,
		SurgSim::DataStructures::DataGroup* result);

	/// The mutex that protects the scaling factors.
	boost::mutex m_mutex;

	/// The scaling factor applied to each direction of the force.
	double m_forceScale;

	/// The scaling factor applied to each direction of the torque.
	double m_torqueScale;

	///@{
	/// The indices into the DataGroups.
	int m_forceIndex;
	int m_torqueIndex;
	int m_springJacobianIndex;
	int m_damperJacobianIndex;
	///@}

	/// True if the output DataGroup indices have been cached.
	bool m_cachedOutputIndices;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_FORCESCALE_H
