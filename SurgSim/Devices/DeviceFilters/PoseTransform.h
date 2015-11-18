// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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
#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_STATIC_REGISTRATION(PoseTransform);

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
class PoseTransform : public DeviceFilter
{
public:
	/// Constructor.
	/// \param name	Name of this device filter.
	explicit PoseTransform(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Devices::PoseTransform);

	/// \return Get the translation scale factor.
	double getTranslationScale() const;

	/// Set the translation scale factor so that each direction has the same scale.
	/// \param translationScale The scalar scaling factor.
	/// \warning This setter is thread-safe, but after calling this function the output filter will use the new
	///		transform even if the following output data is based off input data that used the old transform.
	void setTranslationScale(double translationScale);

	/// \return The current transform.
	const Math::RigidTransform3d& getTransform() const;

	/// Set the constant transform.  The transform is pre-applied to the input pose.
	/// \param transform The transform, which must be invertible.
	/// \warning This setter is thread-safe, but after calling this function the output filter will use the new
	///		transform even if the following output data is based off input data that used the old transform.
	void setTransform(const Math::RigidTransform3d& transform);

private:
	void filterInput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
		DataStructures::DataGroup* result) override;

	void filterOutput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
		DataStructures::DataGroup* result) override;

	/// The mutex that protects the transform and scaling factor.
	boost::mutex m_mutex;

	/// The constant pre-transform.
	Math::RigidTransform3d m_transform;

	/// The inverse of the pre-transform.
	Math::RigidTransform3d m_transformInverse;

	/// The scaling factor applied to each direction of the translation.
	double m_translationScale;
};

};  // namespace Devices
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_DEVICEFILTERS_POSETRANSFORM_H
