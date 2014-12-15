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

#ifndef SURGSIM_FRAMEWORK_MEASUREMENTBASE_H
#define SURGSIM_FRAMEWORK_MEASUREMENTBASE_H

#include <deque>

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/ObjectFactory.h>

namespace SurgSim
{
namespace Framework
{
class Logger;
};
};

namespace SurgSim
{
namespace Framework
{

/// SamplingMetricBase provides a base class to support metric development. New
/// metrics should derive from the base class and redefine canMeasure() and
/// takeMeasurement() ensure the system is ready to be measured and to perform
/// the measurement respectively.
///
/// We will manage the measurement stream as a deque with a maximum size to protect against
/// allowing memory to grow unbounded. The nominal setting provides for 30 minutes of
/// continuous sampling at 30 hertz. After that time, old measurements will be discarded
/// as new measurements are made so as to stay within the same memory footprint.
class SamplingMetricBase : public SurgSim::Framework::Behavior
{
public:
	/// Constructor for the class.
	/// \param name is the name given to the behavior.
	explicit SamplingMetricBase(const std::string& name);

	/// Type of the individual entries in the measurement data structure. The first field of the
	/// pair holds the elapsed time since the start of the measurement process and the second field
	/// of the pair holds the measurement value obtained at that time.
	typedef std::pair<double, double> MeasurementEntryType;

	/// Type of the cumulative entries data structure. The code current caps the number of entries at a
	/// user prescribed value to keep from overwriting all of memory when the process is allowed to run
	/// unchecked over long periods. The maximum number of entries is nominally capped at 30 minutes of samples
	/// taken 30 times per second, but it can be adjusted using he setMaxNumberOfMeasurements call. Note
	/// that we always save the last measurements taken. After the limit is reached we delete the oldest
	/// current entry every time we need to add a new measurement.
	typedef std::deque<MeasurementEntryType> MeasurementsType;

	/// Run an update cycle on this metric.
	/// \param dt is the elapsed time since the last call to update.
	virtual void update(double dt) override;

	/// Set the desired manager type for this metric. Given the potential tight coupling of the
	/// and the various other behaviors, this will provide us with the flexibility to choose
	/// the appropriate manager for the task.
	/// \param targetManagerType is the manager type to be used for managing this metric
	void setTargetManagerType(int);

	/// Get the desired manager type for this metric. Given the potential tight coupling of the
	/// and the various other behaviors, this will provide us with the flexibility to choose
	/// the appropriate manager for the task.
	/// \return the manager type to be used for managing this metric
	virtual int getTargetManagerType() const override;

	/// Set the maximum number of measurements to store.
	void setMaxNumberOfMeasurements(size_t numberOfMeasurements);

	/// \return Maximum number of measurements to be stored.
	size_t getMaxNumberOfMeasurements() const;

	/// \return Number of frames currently stored (not the maximum number of frames).
	size_t getCurrentNumberOfMeasurements() const;

	/// Get the amount of time since the last successful measurement reading based on the
	/// accumulation of successive dt values.
	/// \return the elapsed time
	double getElapsedTime() const;

	/// Return the deque of measurement values obtained for this measurement.
	virtual MeasurementsType getMeasurementValues();

protected:
	/// Wake up the behavior and perform any dependent initializations
	/// \return true or false depending if the wakeup succeeded or not.
	virtual bool doWakeUp() override;

	/// Initialize the behavior and perform any independent initializations
	/// \return true or false depending if the initialization succeeded or not.
	virtual bool doInitialize() override;

	/// \param dt is the elapsed time since the last call to update.
	/// \return if it is currently valid to calculate the next measurement value.
	virtual bool canMeasure(double dt);

	/// \param dt is the elapsed time since the last call to update.
	/// \return the next measurement value. This method should be overwritten to provide the
	/// various measurements for the simulation.
	virtual double performMeasurement(double dt) = 0;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

private:
	/// measurement list
	MeasurementsType m_measurementValues;
	int m_targetManagerType;
	double m_elapsedTime;
	size_t m_maxNumberOfMeasurements;
};
};
};

#endif // SURGSIM_FRAMEWORK_MEASUREMENTBASE_H
