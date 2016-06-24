// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_DEVICES_DEVICEFILTERS_BOOLTOSCALAR_H
#define SURGSIM_DEVICES_DEVICEFILTERS_BOOLTOSCALAR_H

#include "SurgSim/Devices/DeviceFilters/DeviceFilter.h"
#include "SurgSim/Framework/Timer.h"

namespace SurgSim
{
namespace DataStructures
{
class DataGroupCopier;
}

namespace Devices
{

SURGSIM_STATIC_REGISTRATION(BoolToScalar);

/// Maps the on and off state of two boolean values to the increase and decrease of a scalar field,
/// this for example enables the driving on a scalar value through two buttons on a device, no change occurs
/// when both booleans are true at the same time
class BoolToScalar : public DeviceFilter
{
public:
	/// Constructor
	BoolToScalar(const std::string& name);

	/// Destructor
	~BoolToScalar();

	SURGSIM_CLASSNAME(SurgSim::Devices::BoolToScalar);

	void initializeInput(const std::string& device, const DataStructures::DataGroup& inputData) override;

	void filterInput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
					 DataStructures::DataGroup* result) override;

	/// Sets the value that gets used, the actual value to be added is scale * dt per call to filterInput
	/// \param val scale to be used
	void setScale(double val);

	/// \return the scale for addition
	double getScale() const;

	/// Set the range of values that should be produced by this
	/// \param val (min and max for the range)
	void setRange(const std::pair<double, double>& val);

	/// \return the range of values used
	std::pair<double, double> getRange() const;

	/// Enables or disables clamping on the output value (default true)
	/// \param val whether to enable or disable clamping
	void setClamping(bool val);

	/// \return whether the output value is being clamped
	bool isClamping();

	/// Set the field that is read to increase the scalar value, needs to be a bool
	/// \param val name of the field to read for an increase
	void setIncreaseField(const std::string& val);

	/// \return the name of the field used for increasing the value
	std::string getIncreaseField() const;

	/// Set the field this is read to decrease the scalar value, needs to be a bool
	/// \param val name of the field to be read for decrease
	void setDecreaseField(const std::string& val);

	/// \return the name of the field used for decreasing the value
	std::string getDecreaseField() const;

	/// Set the value of the mapped field, can also be used to set a starting value for the field
	/// \param val new Value to be used for the scalar field
	void setScalar(double val);

	/// \return the actual value of the mapped field
	double getScalar() const;

	/// Set the name of the target field that carries the scalar value, will be created in the datagroup if it doesn't
	/// exist
	/// \param val name of the target field
	void setTargetField(const std::string& val);

	/// \return the name of the target field for the scalar information
	std::string getTargetField() const;

private:
	bool m_isClamping;

	double m_value;

	double m_scale;

	std::pair<double, double> m_range;

	std::string m_increaseField;
	std::string m_decreaseField;
	std::string m_targetField;

	Framework::Timer m_timer;

	std::shared_ptr<DataStructures::DataGroupCopier> m_copier;

};

}; // namespace Devices
}; // namespace SurgSim

#endif // SURGSIM_DEVICES_BOOLTOSCALARFILTER_H
