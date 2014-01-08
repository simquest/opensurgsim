

SURGSIM_DOUBLE_SPECIALIZATION
template<typename Base>
static void YAML::convert<std::shared_ptr<SurgSim::Framework::Component>>::registerClass(const std::string& className)
{
	getFactory().registerClass<Base>(className);
}