#ifndef TESTDEVICE_H
#define TESTDEVICE_H

#include <SurgSim/Input/CommonDevice.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>

using SurgSim::Input::CommonDevice;
using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Input::InputConsumerInterface;
using SurgSim::Input::OutputProducerInterface;


class TestDevice : public CommonDevice
{
public:
	TestDevice(const std::string& uniqueName);

	virtual bool initialize();

	virtual bool finalize();

	virtual void pushInput();

	// Send some data down the stream
	void pushInput(std::string data);

	virtual bool pullOutput();

	const DataGroup& getOutputData() const;

	/// Builds the data layout for the application input (i.e. device output).
	static DataGroup buildInputData();
	DataGroup buildOutputData();

	std::string lastPulledData;
};


struct TestInputConsumer : public InputConsumerInterface
{
public:
	TestInputConsumer() :
	  m_numTimesReceivedInput(0)
	  {
	  }

	  virtual void handleInput(const std::string& device, const DataGroup& inputData);

	  int m_numTimesReceivedInput;
	  DataGroup m_lastReceivedInput;
};

struct TestOutputProducer : public OutputProducerInterface
{
public:
	TestOutputProducer() :
	  m_numTimesRequestedOutput(0),
		  m_refuseToProduce(false)
	  {
		  DataGroupBuilder builder;
		  builder.addInteger("value");
		  m_nextSentOutput = builder.createData();
		  m_nextSentOutput.integers().set("value", 123);
	  }

	  virtual bool requestOutput(const std::string& device, DataGroup* outputData);

	  int m_numTimesRequestedOutput;
	  bool m_refuseToProduce;
	  DataGroup m_nextSentOutput;
};

#endif // TESTDEVICE_H
