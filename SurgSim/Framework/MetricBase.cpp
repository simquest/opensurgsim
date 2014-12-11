#include "MetricBase.h"

#include <SurgSim/Framework/Log.h>

MetricBase::MetricBase(const std::string& name) :
	SurgSim::Framework::Behavior(name),
	m_logger(SurgSim::Framework::Logger::getLogger(name)),
	m_elapsedTime(0.0),
	m_managerType(SurgSim::Framework::MANAGER_TYPE_NONE),
	m_maxNumberOfMeasurements(54000)
{
}

bool MetricBase::doWakeUp()
{
	m_elapsedTime = 0.0;
	m_measurementValues.clear();
	SURGSIM_LOG_INFO(m_logger) << " Waking up";
	return true;
}

bool MetricBase::doInitialize()
{
	return true;
}

bool MetricBase::doTest(double dt)
{
	return true;
}

double MetricBase::doMeasure(double dt)
{
	return 0.0;
}

MetricBase::MeasurementsType MetricBase::getMeasurementValues()
{
	return m_measurementValues;
}

int MetricBase::getTargetManagerType() const
{
	return m_managerType;
}

void MetricBase::setTargetManagerType(int type)
{
	m_managerType = type;
}

void MetricBase::setMaxNumberOfMeasurements(size_t maxNumberOfMeasurements)
{
	m_maxNumberOfMeasurements = (maxNumberOfMeasurements > 0) ? maxNumberOfMeasurements : 1;
	if (m_measurementValues.size() > m_maxNumberOfMeasurements)
	{
		m_measurementValues.erase(std::begin(m_measurementValues),
								  std::begin(m_measurementValues) + m_measurementValues.size() - m_maxNumberOfMeasurements);
	}
}

size_t MetricBase::getMaxNumberOfMeasurements() const
{
	return m_maxNumberOfMeasurements;
}

size_t MetricBase::getCurrentNumberOfMeasurements() const
{
	return m_measurementValues.size();
}

void MetricBase::update(double dt)
{
	m_elapsedTime += dt;
	if (doTest(dt))
	{
		MeasurementEntryType newEntry;
		newEntry.first = m_elapsedTime;
		newEntry.second = doMeasure(dt);
		m_measurementValues.push_back(newEntry);
		if (m_measurementValues.size() > m_maxNumberOfMeasurements)
		{
			m_measurementValues.pop_front();
		}

	}
}


