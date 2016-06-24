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

#include "SurgSim/Devices/DeviceFilters/BoolToScalar.h"

#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/DataStructures/DataGroupCopier.h"
#include "SurgSim/Math/Scalar.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::BoolToScalar, BoolToScalar);

BoolToScalar::BoolToScalar(const std::string& name) :
	DeviceFilter(name),
	m_isClamping(true),
	m_value(0.0),
	m_scale(1.0),
	m_range(0.0, 1.0),
	m_increaseField(DataStructures::Names::BUTTON_2),
	m_decreaseField(DataStructures::Names::BUTTON_1),
	m_targetField(DataStructures::Names::TOOLDOF)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, double, Scale, getScale, setScale);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, double, Scalar, getScalar, setScalar);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, bool, Clamping, isClamping, setClamping);

	{
		typedef std::pair<double, double> ParamType;
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, ParamType, Range, getRange, setRange);
	}
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, std::string, IncreaseField,
									  getIncreaseField, setIncreaseField);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, std::string, DecreaseField,
									  getDecreaseField, setDecreaseField);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoolToScalar, std::string, TargetField, getTargetField, setTargetField);
}

BoolToScalar::~BoolToScalar()
{
}

void BoolToScalar::initializeInput(const std::string& device, const DataStructures::DataGroup& inputData)
{
	SURGSIM_ASSERT(inputData.booleans().getIndex(m_decreaseField) != -1)
			<< "Can't find decrease field " << m_decreaseField << " in booleans of datagroup.";

	SURGSIM_ASSERT(inputData.booleans().getIndex(m_increaseField) != -1)
			<< "Can't find increase field " << m_increaseField << " in booleans of datagroup.";

	if (getInputData().isEmpty())
	{
		if (!inputData.scalars().hasEntry(m_targetField))
		{
			DataStructures::DataGroupBuilder builder;
			builder.addEntriesFrom(inputData);
			builder.addScalar(m_targetField);
			getInputData() = builder.createData();
			m_copier = std::make_shared<DataStructures::DataGroupCopier>(inputData, &getInputData());
		}
	}

	if (m_copier == nullptr)
	{
		getInputData() = inputData;
	}
	else
	{
		m_copier->copy(inputData, &getInputData());
	}

	m_timer.start();
}

void BoolToScalar::filterInput(const std::string& device, const DataStructures::DataGroup& dataToFilter,
							   DataStructures::DataGroup* result)
{
	m_timer.markFrame();
	double dt = m_timer.getLastFramePeriod();

	if (m_copier == nullptr)
	{
		*result = dataToFilter;
	}
	else
	{
		m_copier->copy(dataToFilter, result);
	}

	bool increase = false;
	dataToFilter.booleans().get(m_increaseField, &increase);

	bool decrease = false;
	dataToFilter.booleans().get(m_decreaseField, &decrease);


	if (increase != decrease)
	{
		m_value += (increase) ? dt * m_scale : -1.0 * dt * m_scale;
		if (m_isClamping)
		{
			m_value = Math::clamp(m_value, m_range.first, m_range.second, 0.0);
		}
	}

	result->scalars().set(m_targetField, m_value);
}


double BoolToScalar::getScale() const
{
	return m_scale;
}

void BoolToScalar::setRange(const std::pair<double, double>& val)
{
	m_range = val;
	if (val.first > val.second)
	{
		std::swap(m_range.first, m_range.second);
	}
}

std::pair<double, double> BoolToScalar::getRange() const
{
	return m_range;
}

void BoolToScalar::setClamping(bool val)
{
	m_isClamping = val;
}

bool BoolToScalar::isClamping()
{
	return m_isClamping;
}

void BoolToScalar::setScale(double val)
{
	m_scale = val;
}

std::string BoolToScalar::getIncreaseField() const
{
	return m_increaseField;
}


void BoolToScalar::setIncreaseField(const std::string& val)
{
	m_increaseField = val;
}

std::string BoolToScalar::getDecreaseField() const
{
	return m_decreaseField;
}

void BoolToScalar::setDecreaseField(const std::string& val)
{
	m_decreaseField = val;
}

double BoolToScalar::getScalar() const
{
	return m_value;
}

void BoolToScalar::setScalar(double val)
{
	m_value = val;
	if (m_isClamping)
	{
		m_value = Math::clamp(m_value, m_range.first, m_range.second, 0.0);
	}
}


std::string BoolToScalar::getTargetField() const
{
	return m_targetField;
}

void BoolToScalar::setTargetField(const std::string& val)
{
	m_targetField = val;
}

}; // namespace Devices
}; // namepsace SurgSim