

#ifndef SURGSIM_GRAPHICS_OSGTANGENTSPACEGENERATOR_H
#define SURGSIM_GRAPHICS_OSGTANGENTSPACEGENERATOR_H

#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>

namespace SurgSim
{
namespace Graphics
{

/// Triangle index functor which calculates the tangent space basis vectors for the vertices
/// of a geometry from texture coordinates
class GenerateTangentSpaceTriangleIndexFunctor
{
public:
	/// Constructor
	GenerateTangentSpaceTriangleIndexFunctor();

	/// Sets whether the three tangent space basis vectors are made to be orthonormal;
	/// otherwise, each tangent is separately orthonormal to the normal, but not to each other
	/// \param orthonomal Whether or not to create a fully orthonormal basis
	void setBasisOrthonormality(bool orthonormal);

	/// \return Gets whether the three tangent space basis vectors are made to be orthonormal; otherwise,
	/// each tangent is separately orthonormal to the normal, but not to each other
	bool getBasisOrthonormality();

	//! Sets the arrays required to generate tangent space basis vectors
	/// \param vertexArray Array containing vertex positions
	/// \param normalArray Array containing vertex normals
	/// \param textureCoordArray Array containing texture coordinates
	/// \param tangentArray Array to store calculated tangents
	/// \param bitangentArray Array to store calculated bitangents
	void set(const osg::Vec3Array* vertexArray,
			 const osg::Vec3Array* normalArray,
			 const osg::Vec2Array* textureCoordArray,
			 osg::Vec4Array* tangentArray,
			 osg::Vec4Array* bitangentArray);

	/// Orthogonalize and normalize the calculated tangent space basis vectors
	void orthonormalize();

	/// Resets all calculated tangent space basis vectors to 0.
	void reset();

	/// Calculates the triangle tangent space basis vectors and adds it to each adjacent vertex's tangent
	//  space basis vectors.
	/// \param vertexIndex1 First triangle vertex index
	/// \param vertexIndex2 Second triangle vertex index
	/// \param vertexIndex3 Third triangle vertex index
	void operator()(unsigned int vertexIndex1, unsigned int vertexIndex2, unsigned int vertexIndex3);

private:
	/// Array containing vertex positions
	const osg::Vec3Array* m_vertexArray;

	/// Array containing normals
	const osg::Vec3Array* m_normalArray;

	/// Array containing texture coordinates
	const osg::Vec2Array* m_textureCoordArray;

	/// Array storing calculated tangents
	osg::Vec4Array* m_tangentArray;

	/// Array storing calculated bitangents
	osg::Vec4Array* m_bitangentArray;

	/// Whether or not to create a fully orthonormal basis; otherwise, each tangent is separately orthonormal
	/// to the normal, but not to eachother
	bool m_createOrthonormalBasis;
};

/// Node visitor which calculates the tangent space basis vectors from the texture coordinates of any
/// geometry it encounters
class TangentSpaceGenerator : public osg::NodeVisitor
{
public:
	/// Constructor
	/// \param textureCoordUnit Texture unit of texture coordinates to use for calculating the tangent space
	/// \param tangentAttribIndex Index of the vertex attribute array to store the calculated tangents
	/// \param bitangentAttribIndex Index of the vertex attribute array to store the calculated bitangents
	TangentSpaceGenerator(int textureCoordUnit, int tangentAttribIndex, int bitangentAttribIndex);

	/// Destructor
	virtual ~TangentSpaceGenerator();

	/// Sets whether the three tangent space basis vectors are made to be orthonormal;
	/// otherwise, each tangent is separately orthonormal to the normal, but not to each other
	/// \param orthonomal Whether or not to create a fully orthonormal basis
	void setBasisOrthonormality(bool orthonormal);

	/// Gets whether the three tangent space basis vectors are made to be orthonormal; otherwise,
	/// each tangent is separately orthonormal to the normal, but not to each other
	bool getBasisOrthonormality();

	//! Generates tangent space vectors for all geometry in the geode
	/// \param geode Geode to generate tangent space vectors
	virtual void apply(osg::Geode& geode);

	//! Generates tangent space basis vectors for the geometry
	/// \param geometry Geometry to generate normals
	/// \param textureCoordUnit Texture unit of texture coordinates to use for calculating the tangent space
	/// \param tangentAttribIndex Index of the vertex attribute array to store the calculated tangents
	/// \param bitangentAttribIndex Index of the vertex attribute array to store the calculated bi-tangents
	/// \param orthonormal  Whether or not to create a fully orthonormal basis; otherwise, each tangent is separately
	///        orthonormal to the normal, but not to each other
	static void generateTangentSpace(osg::Geometry* geometry,
									 int textureCoordUnit,
									 int tangentAttribIndex,
									 int bitangentAttribIndex,
									 bool orthonormal);

private:
	/// Texture unit of texture coordinates to use for calculating the tangent space
	int m_textureCoordUnit;

	/// Index of the vertex attribute array to store the calculated tangents
	int m_tangentAttribIndex;

	/// Index of the vertex attribute array to store the calculated bitangents
	int m_bitangentAttribIndex;

	/// Whether or not to create a fully orthonormal basis; otherwise, each tangent is separately orthonormal to
	/// the normal, but not to eachother
	bool m_createOrthonormalBasis;
};

}
}

#endif
