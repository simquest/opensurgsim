
#include "../loadVtkUnstructuredData.h"

using TetrahedronMeshType = SurgSim::DataStructures::TetrahedronMesh<
    SurgSim::DataStructures::EmptyData,
    SurgSim::DataStructures::EmptyData,
    SurgSim::DataStructures::EmptyData,
    SurgSim::DataStructures::EmptyData>;

int main(int ac, char **av)
{
    if(ac == 2)
    {
        std::shared_ptr<TetrahedronMeshType> tetrahedonMesh = std::make_shared<TetrahedronMeshType>();
        std::string filename(av[1]);
        loadVtkUnstructuredData(filename,tetrahedonMesh);

        std::cout << "Number vertices: " << tetrahedonMesh->getNumVertices() << std::endl;
        std::cout << "Number tetrahedrons: " << tetrahedonMesh->getNumTetrahedrons() << std::endl;
        std::cout << "Number surface triangles: " << tetrahedonMesh->getNumTriangles() << std::endl;

        for(size_t i = 0; i < tetrahedonMesh->getNumVertices(); ++i)
        {
            std::cout << tetrahedonMesh->getVertex(i).position << std::endl;
        }

        for(size_t i = 0; i < tetrahedonMesh->getNumTetrahedrons(); ++i)
        {
            std::cout << "[ ";
            for(size_t j = 0; j < 4; ++j)
            {
                std::cout << tetrahedonMesh->getTetrahedron(i).verticesId[j] << " ";
            }
            std::cout << "]" << std::endl;
        }

        for(size_t i = 0; i < tetrahedonMesh->getNumTriangles(); ++i)
        {
            std::cout << "[ ";
            for(size_t j = 0; j < 3; ++j)
            {
                std::cout << tetrahedonMesh->getTriangle(i).verticesId[j] << " ";
            }
            std::cout << "]" << std::endl;
        }

        return 0;
    }
    else
    {
        std::cout << "Usage: " << av[0] << " input_vtu_mesh" << std::endl;
        return 1;
    }
}
