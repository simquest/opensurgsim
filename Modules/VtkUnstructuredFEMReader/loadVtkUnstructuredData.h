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

template<typename TetrahedronMeshType>
void loadVtkUnstructuredData(const std::string &filename,
                             std::shared_ptr<TetrahedronMeshType> tetrahedonMesh)
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
    double mass = 0;
    double poissonRatio = 0;
    double youngModulus = 0;
    vtkUnsignedIntArray *boundaryConditions;
    if(fields)
    {
        boundaryConditions = vtkUnsignedIntArray::SafeDownCast(fields->GetArray("boundary_conditions"));
        mass = fields->GetArray("mass_density")->GetComponent(0,0);
        poissonRatio = fields->GetArray("poisson_ratio")->GetComponent(0,0);
        youngModulus = fields->GetArray("young_modulus")->GetComponent(0,0);
    }

    // Populate the tetrahedron data structure
    // Copy vertices
    for(size_t i = 0; i < vertices->GetNumberOfPoints(); ++i)
    {
        double pointArray[3];
        vertices->GetPoint(i,pointArray);

        SurgSim::Math::Vector3d point(pointArray[0],pointArray[1],pointArray[2]);
        typename TetrahedronMeshType::VertexType vertex(point);
        tetrahedonMesh->addVertex(vertex);
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
                typename TetrahedronMeshType::TriangleType triangle(triangleArray);
                tetrahedonMesh->addTriangle(triangle);
                break;
            }
            case 4:
            {
                std::array<size_t,4> tetraArray;
                tetraArray[0] = element->GetId(0);
                tetraArray[1] = element->GetId(1);
                tetraArray[2] = element->GetId(2);
                tetraArray[3] = element->GetId(3);
                typename TetrahedronMeshType::TetrahedronType tetra(tetraArray);
                tetrahedonMesh->addTetrahedron(tetra);
                break;
            }
            default:
                std::cerr << "Unallowed element type encountered with " << element->GetNumberOfIds() << " vertices." << std::endl;

        }
    }

}
