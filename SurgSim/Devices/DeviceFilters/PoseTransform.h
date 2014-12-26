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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_POSETRANSFORM_H
#define SURGSIM_DEVICES_DEVICEFILTERS_POSETRANSFORM_H

#include <boost/thread/mutex.hpp>
#include <memory>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/CommonDevice.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Device
{
/// A device filter that transforms the input pose.  It can scale the translation, and/or apply a constant transform.
/// Any other data in the DataGroup is passed through.  For an input/output device (e.g., a haptic device), the filter
/// should be added as one of the device's input consumers and set as the device's output producer.  For a purely input
/// device, the filter can just be added as an input consumer.  If it is used for both input and output, the input data
/// is filtered using the offset(s) and scaling, and the output data is un-filtered so the device does not need to know
/// about the filtering.
/// For haptic devices, so that changing the translation scaling does not alter the relationship between displayed
/// forces and collision penetrations, the output filter does not scale the nominal forces and torques, and it does
/// scale the Jacobians.  Thereby the displayed forces and torques are appropriate for the scene (not the device-space
/// motions).  In other words, a 1 m motion by the device's scene representation generates forces according to that 1 m
/// motion, instead of the original device motion before scaling.  This means the device displays forces and torques
/// that are "true" to the scene, with the consequence that increasing the translation scaling can negatively impact the
/// haptic stability.  As the scaling increases, the same motion would cause larger forces, until at great enough
/// scaling the system becomes unstable (either when colliding with another scene element, or just due to over-damping).
/// Consider chaining this device filter with a force scaling device filter to improve system stability.
/// \sa	SurgSim::Input::CommonDevice
/// \sa	SurgSim::Input::InputConsumerInterface
/// \sa	SurgSim::Input::OutputProducerInterface
class PoseTransform : public SurgSim::Input::CommonDevice,
	public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit PoseTransform(const std::string& name);

	/// Destructor.
	virtual ~PoseTransform();

	/// Fully initialize the device.
	/// When the manager object creates the device, the internal state of the device usually isn't fully
	/// initialized yet.  This method performs any needed initialization.
	/// \return True on success.
	bool initialize() override;

	/// Set the initial input data.  Used when transforming the pose coming from an input device.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	/// Notifies the consumer that the application input coming from the device has been updated.
	/// Used when transforming the pose coming from an input device.
	/// \param device The name of the device that is producing the input.  This should only be used to identify
	/// 	the device (e.g. if the consumer is listening to several devices at once).
	/// \param inputData The application input state coming from the device.
	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	/// Asks the producer to provide output state to the device.  Passes through all data, modifying the data entries
	/// used by haptic devices.  Note that devices may never call this method, e.g. because the device doesn't actually
	/// have any output capability.
	/// \param device The name of the device that is requesting the output.  This should only be used to identify
	/// 	the device (e.g. if the producer is listening to several devices at once).
	/// \param [out] outputData The data being sent to the device.
	/// \return True if the producer has provided output data.  A producer that returns false should leave outputData
	///		unmodified.
	bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

	/// Set the translation scale factor so that each direction has the same scale.
	/// \param translationScale The scalar scaling factor.
	/// \warning This setter is thread-safe, but after calling this function the output filter will use the new
	///		transform even if the following output data is based off input data that used the old transform.
	void setTranslationScale(double translationScale);

	/// Set the constant transform.  The transform is pre-applied to the input pose.
	/// \param transform The transform, which must be invertible.
	/// \warning This setter is thread-safe, but after calling this function the output filter will use the new
	///		transform even if the following output data is based off input data that used the old transform.
	void setTransform(const SurgSim::Math::RigidTransform3d& transform);

private:
	/// Finalize (de-initialize) the device.
	/// \return True on success.
	bool finalize() override;

	/// Filter the input data.
	/// \param dataToFilter The data that will be filtered.
	/// \param [in,out] result A pointer to a DataGroup object that must be assignable to by the dataToFilter object.
	///		Will contain the filtered data.
	void inputFilter(const SurgSim::DataStructures::DataGroup& dataToFilter,
		SurgSim::DataStructures::DataGroup* result);

	/// Filter the output data.
	/// If this device filter offsets/scales input data from a haptic device, then when output data (forces, torques,
	/// Jacobians) are created for output to that device, this function incorporates the transform/scaling to correct
	/// the displayed outputs.
	/// \param dataToFilter The data that will be filtered.
	/// \param [in,out] result A pointer to a DataGroup object that must be assignable to by the dataToFilter object.
	///		Will contain the filtered data.
	void outputFilter(const SurgSim::DataStructures::DataGroup& dataToFilter,
		SurgSim::DataStructures::DataGroup* result);

	/// The mutex that protects the transform and scaling factor.
	boost::mutex m_mutex;

	/// The constant pre-transform.
	SurgSim::Math::RigidTransform3d m_transform;

	/// The inverse of the pre-transform.
	SurgSim::Math::RigidTransform3d m_transformInverse;

	/// The scaling factor applied to each direction of the translation.
	double m_translationScale;

	///@{
	/// The indices into the DataGroups.
	int m_poseIndex;
	int m_linearVelocityIndex;
	int m_angularVelocityIndex;
	int m_forceIndex;
	int m_torqueIndex;
	int m_springJacobianIndex;
	int m_inputPoseIndex;
	int m_damperJacobianIndex;
	int m_inputLinearVelocityIndex;
	int m_inputAngularVelocityIndex;
	///@}

	/// True if the output DataGroup indices have been cached.
	bool m_cachedOutputIndices;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_POSETRANSFORM_H
