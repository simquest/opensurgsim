
#include "../loadVtkUnstructuredData.h"

int main(int ac, char **av)
{
    if(ac == 2)
    {
        std::shared_ptr<Fem3DTetrahedron> fem = std::make_shared<Fem3DTetrahedron>();
        std::string filename(av[1]);
        loadVtkUnstructuredData(filename,fem);

        std::cout << "Number vertices: " << fem->getNumVertices() << std::endl;
        std::cout << "Number tetrahedrons: " << fem->getNumTetrahedrons() << std::endl;
        std::cout << "Number surface triangles: " << fem->getNumTriangles() << std::endl;
        std::cout << "Mass Density: " << fem->getMassDensity() << std::endl;
        std::cout << "Poisson's Ratio: " << fem->getPoissonRatio() << std::endl;
        std::cout << "Young's Ratio: " << fem->getYoungModulus() << std::endl;

        for(size_t i = 0; i < fem->getNumVertices(); ++i)
        {
            std::cout << fem->getVertex(i).position << std::endl;
        }

        for(size_t i = 0; i < fem->getNumTetrahedrons(); ++i)
        {
            std::cout << "[ ";
            for(size_t j = 0; j < 4; ++j)
            {
                std::cout << fem->getTetrahedron(i).verticesId[j] << " ";
            }
            std::cout << "]" << std::endl;
        }

        for(size_t i = 0; i < fem->getNumTriangles(); ++i)
        {
            std::cout << "[ ";
            for(size_t j = 0; j < 3; ++j)
            {
                std::cout << fem->getTriangle(i).verticesId[j] << " ";
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
