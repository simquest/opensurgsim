#ifndef FEM3DTETRAHEDRON_H
#define FEM3DTETRAHEDRON_H

#include "SurgSim/DataStructures/TetrahedronMesh.h"
#include "SurgSim/DataStructures/EmptyData.h"

typedef SurgSim::DataStructures::TetrahedronMesh<SurgSim::DataStructures::EmptyData, SurgSim::DataStructures::EmptyData,
		SurgSim::DataStructures::EmptyData, SurgSim::DataStructures::EmptyData> TetrahedronBase;

class Fem3DTetrahedron : public TetrahedronBase
{
public:
	Fem3DTetrahedron()
	{
	}

	void setMassDensity(double value)
	{
		m_massDensity = value;
	}

	double getMassDensity()
	{
		return m_massDensity;
	}

	void setPoisonRatio(double value)
	{
		m_poissonRatio = value;
	}

	double getPoissonRatio()
	{
		return m_poissonRatio;
	}

	void setYoungModulus(double value)
	{
		m_youngModulus = value;
	}

	double getYoungModulus()
	{
		return m_youngModulus;
	}

private:
	double m_massDensity;
	double m_poissonRatio;
	double m_youngModulus;
};


#endif // FEM3DTETRAHEDRON_H
