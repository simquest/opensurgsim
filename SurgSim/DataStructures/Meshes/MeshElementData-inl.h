#include <typeinfo>

using SurgSim::DataStructures::MeshElementData;

template <unsigned int N>
MeshElementData<N>::MeshElementData()
{

}

template <unsigned int N>
MeshElementData<N>::~MeshElementData()
{

}

template <unsigned int N>
bool MeshElementData<N>::operator==(const MeshElementData<N>& data) const
{
	return (typeid(*this) == typeid(data)) && isEqual(data);
}

template <unsigned int N>
bool MeshElementData<N>::operator!=(const MeshElementData<N>& data) const
{
	return (typeid(*this) != typeid(data)) || ! isEqual(data);
}