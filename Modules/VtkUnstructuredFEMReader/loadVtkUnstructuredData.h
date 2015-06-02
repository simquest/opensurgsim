#ifndef LOAD_VTK_UNSTRUCTURED_DATA_H
#define LOAD_VTK_UNSTRUCTURED_DATA_H

#include <iostream>
#include <memory>
#include <array>

// VTK includes
#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>
#include <vtkFieldData.h>
#include <vtkUnsignedIntArray.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>

// OSS includes
#include "SurgSim/DataStructures/TetrahedronMesh.h"
#include "SurgSim/DataStructures/EmptyData.h"

#include "Fem3DTetrahedron.h"


void loadVtkUnstructuredData(const std::string &filename, std::shared_ptr<Fem3DTetrahedron> fem)
{
    // Read vtu dataset
    vtkNew<vtkXMLUnstructuredGridReader> reader;
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkUnstructuredGrid *mesh = reader->GetOutput();
    vtkFieldData *fields = mesh->GetFieldData();

    // Get topology data
    vtkPoints *vertices = mesh->GetPoints();
    vtkCellArray *cells = mesh->GetCells();

    // Get boundary conditions and material properties
    vtkUnsignedIntArray *boundaryConditions;
    if(fields)
    {
        boundaryConditions = vtkUnsignedIntArray::SafeDownCast(fields->GetArray("boundary_conditions"));
        fem->setMassDensity(fields->GetArray("mass_density")->GetComponent(0,0));
        fem->setPoisonRatio(fields->GetArray("poisson_ratio")->GetComponent(0,0));
        fem->setYoungModulus(fields->GetArray("young_modulus")->GetComponent(0,0));
    }

    // Populate the tetrahedron data structure
    // Copy vertices
    for(size_t i = 0; i < vertices->GetNumberOfPoints(); ++i)
    {
        double pointArray[3];
        vertices->GetPoint(i,pointArray);

        SurgSim::Math::Vector3d point(pointArray[0],pointArray[1],pointArray[2]);
        typename Fem3DTetrahedron::VertexType vertex(point);
        fem->addVertex(vertex);
    }

    // Copy cells (triangles and tetras) to the tetrahedron datastructure
    vtkNew<vtkIdList> element;
    for (vtkIdType cellId = 0; cells->GetNextCell(element.GetPointer()); ++cellId)
    {
        switch(element->GetNumberOfIds())
        {
            case 3:
            {
                std::array<size_t,3> triangleArray;
                triangleArray[0] = element->GetId(0);
                triangleArray[1] = element->GetId(1);
                triangleArray[2] = element->GetId(2);
                typename Fem3DTetrahedron::TriangleType triangle(triangleArray);
                fem->addTriangle(triangle);
                break;
            }
            case 4:
            {
                std::array<size_t,4> tetraArray;
                tetraArray[0] = element->GetId(0);
                tetraArray[1] = element->GetId(1);
                tetraArray[2] = element->GetId(2);
                tetraArray[3] = element->GetId(3);
                typename Fem3DTetrahedron::TetrahedronType tetra(tetraArray);
                fem->addTetrahedron(tetra);
                break;
            }
            default:
                std::cerr << "Unallowed element type encountered with " << element->GetNumberOfIds() << " vertices." << std::endl;

        }
    }

}

#endif //LOAD_VTK_UNSTRUCTURED_DATA_H
