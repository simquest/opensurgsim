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

#include <SurgSim/Framework/Behavior.h>
#include <deque>

namespace SurgSim
{
namespace Framework
{
class Logger;
};
};

/// We will manage the measurement stream as a deque with a maximum size to protect against
/// allowing memory to grow unbounded.

class MetricBase : public SurgSim::Framework::Behavior
{
public:
	explicit MetricBase(const std::string& name);
	virtual void update(double dt) override;
	void setTargetManagerType(int type);

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

	/// Set the maximum number of measurements to store.
	void setMaxNumberOfMeasurements(size_t numberOfMeasurements);

	/// \return Maximum number of measurements to be stored.
	size_t getMaxNumberOfMeasurements() const;

	/// \return Number of frames currently stored (not the maximum number of frames).
	size_t getCurrentNumberOfMeasurements() const;

	/// Return the deque of measurement values obtained for this measurement.
	virtual MeasurementsType getMeasurementValues();

protected:
	virtual bool doWakeUp() override;
	virtual bool doInitialize() override;

	/// \param dt is the elapsed time since the last call to update.
	/// \return if it is currently valid to calculate the next measurement value.
	virtual bool doTest(double dt);

	/// \param dt is the elapsed time since the last call to update.
	/// \return the next measurement value. This method should be overwritten to provide the
	/// various measurements for the simulation.
	virtual double doMeasure(double dt);

	virtual int getTargetManagerType() const override;

	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

private:
	/// measurement list
	MeasurementsType m_measurementValues;
	int m_managerType;
	double m_elapsedTime;
	size_t m_maxNumberOfMeasurements;
};

#endif // SURGSIM_FRAMEWORK_MEASUREMENTBASE_H
