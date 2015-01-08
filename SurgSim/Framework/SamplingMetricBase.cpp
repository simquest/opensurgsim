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

#include "SamplingMetricBase.h"

#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Accessible.h"

namespace SurgSim
{
namespace Framework
{

SamplingMetricBase::SamplingMetricBase(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_logger(SurgSim::Framework::Logger::getLogger(name)),
	m_targetManagerType(SurgSim::Framework::MANAGER_TYPE_BEHAVIOR),
	m_elapsedTime(0.0),
	m_maxNumberOfMeasurements(54000)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Framework::SamplingMetricBase, size_t, MaxNumberOfMeasurements,
									  getMaxNumberOfMeasurements, setMaxNumberOfMeasurements);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurgSim::Framework::SamplingMetricBase, int, TargetManagerType,
									  getTargetManagerType, setTargetManagerType);
}

bool SamplingMetricBase::doWakeUp()
{
	return true;
}

bool SamplingMetricBase::doInitialize()
{
	m_elapsedTime = 0.0;
	m_measurementValues.clear();
	return true;
}

bool SamplingMetricBase::canMeasure(double dt)
{
	return true;
}

SamplingMetricBase::MeasurementsType SamplingMetricBase::getMeasurementValues()
{
	return m_measurementValues;
}

int SamplingMetricBase::getTargetManagerType() const
{
	return m_targetManagerType;
}

void SamplingMetricBase::setTargetManagerType(int targetManagerType)
{
	m_targetManagerType = targetManagerType;
}

void SamplingMetricBase::setMaxNumberOfMeasurements(size_t maxNumberOfMeasurements)
{
	m_maxNumberOfMeasurements = (maxNumberOfMeasurements > 0) ? maxNumberOfMeasurements : 1;
	if (m_measurementValues.size() > m_maxNumberOfMeasurements)
	{
		m_measurementValues.erase(std::begin(m_measurementValues),
								  std::begin(m_measurementValues) +
								  m_measurementValues.size() -
								  m_maxNumberOfMeasurements);
	}
}

size_t SamplingMetricBase::getMaxNumberOfMeasurements() const
{
	return m_maxNumberOfMeasurements;
}

size_t SamplingMetricBase::getCurrentNumberOfMeasurements() const
{
	return m_measurementValues.size();
}

double SamplingMetricBase::getElapsedTime() const
{
	return m_elapsedTime;
}

void SamplingMetricBase::update(double dt)
{
	m_elapsedTime += dt;
	if (canMeasure(dt))
	{
		MeasurementEntryType newEntry;
		newEntry.first = m_elapsedTime;
		newEntry.second = performMeasurement(dt);
		m_measurementValues.push_back(newEntry);
		m_elapsedTime = 0;
		if (m_measurementValues.size() > m_maxNumberOfMeasurements)
		{
			m_measurementValues.pop_front();
		}

	}
}
}
}


