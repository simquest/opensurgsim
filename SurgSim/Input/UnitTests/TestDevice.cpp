
#include "TestDevice.h"

TestDevice::TestDevice(const std::string& uniqueName) :
	CommonDevice(uniqueName, buildInputData())
{

}

// required by the DeviceInterface API
bool TestDevice::initialize()
{
	return true;
}

// required by the DeviceInterface API
bool TestDevice::finalize()
{
	return true;
}

// expose the pushInput method to the world
void TestDevice::pushInput()
{
	CommonDevice::pushInput();
}

void TestDevice::pushInput(const std::string& data)
{
	getInputData().strings().set("helloWorld", data);
	pushInput();
}

// expose the pullOutput method to the world
bool TestDevice::pullOutput()
{
	bool result = CommonDevice::pullOutput();
	getOutputData().strings().get("data",&lastPulledData);
	return result;
}

// expose the getOutputData method to the world
const DataGroup& TestDevice::getOutputData() const
{
	return CommonDevice::getOutputData();
}

DataGroup TestDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addString("helloWorld");
	DataGroup data = builder.createData();
	data.strings().set("helloWorld", "data");
	return data;
}

// Consumer Class Callback Function
void TestInputConsumer::handleInput(const std::string& device, const DataGroup& inputData)
{
	++m_numTimesReceivedInput;
	m_lastReceivedInput = inputData;
}

// Producer class Hook
bool TestOutputProducer::requestOutput(const std::string& device, DataGroup* outputData)
{
	++m_numTimesRequestedOutput;

	if (m_refuseToProduce)
	{
		return false;
	}
	else
	{
		*outputData = m_nextSentOutput;
		return true;
	}
}

