// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_PHYSICS_FEMREPRESENTATION_H
#define SURGSIM_PHYSICS_FEMREPRESENTATION_H

#include <future>
#include <memory>

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/Fem.h"

namespace SurgSim
{

namespace Physics
{

class FemElement;
class FemPlyReaderDelegate;

/// Finite Element Model (a.k.a FEM) is a deformable model (a set of nodes connected by FemElement).
/// \note A fem is a DeformableRepresentation (Physics::Representation and Math::OdeEquation), therefore it defines
/// a dynamic system \f$M.a=F(x,v)\f$
/// \note + The model handles damping through the Rayleigh damping (it is a combination of mass and stiffness)
/// \note + The model handles compliance warping (optional) from the paper:
/// \note  "Efficient Contact Modeling using Compliance Warping", G Saupin, C Duriez, S Cotin, L Grisoni;
/// Computer %Graphics International (CGI), Istanbul, Turkey, june 2008.
/// \note  To use compliance warping, it needs to be turned on by calling setComplianceWarping(true) and the method
/// updateNodesRotations() needs to be overloaded properly.
class FemRepresentation : public DeformableRepresentation
{
public:
	/// Constructor
	/// \param name The name of the FemRepresentation
	explicit FemRepresentation(const std::string& name);

	/// Destructor
	virtual ~FemRepresentation();

	/// Loads the FEM file into an Fem class data structure
	/// \param filename The file to load
	virtual void loadFem(const std::string& filename) = 0;

	/// Sets the FemElement type pulled from the object factory
	/// \param type A string of the full registered OSS class name in the factory
	virtual void setFemElementType(const std::string& type);

	/// Gets the FemElement type pulled from the object factory
	/// \return A string of the full registered OSS class name in the factory
	const std::string& getFemElementType() const;

	/// Adds a FemElement
	/// \param element The FemElement to add to the representation
	void addFemElement(const std::shared_ptr<FemElement> element);

	/// Gets the number of FemElement
	/// \return the number of FemElement
	size_t getNumFemElements() const;

	/// Retrieves a given FemElement from its id
	/// \param femElementId The FemElement id for which the FemElement is requested
	/// \return The FemElement for the given femElementId
	/// \note The FemElement is returned with read/write access
	/// \note Out of range femElementId will raise an exception
	std::shared_ptr<FemElement> getFemElement(size_t femElementId);

	/// Gets the total mass of the fem
	/// \return The total mass of the fem (in Kg)
	double getTotalMass() const;

	/// Gets the Rayleigh stiffness parameter
	/// \return The Rayleigh stiffness parameter
	double getRayleighDampingStiffness() const;

	/// Gets the Rayleigh mass parameter
	/// \return The Rayleigh mass parameter
	double getRayleighDampingMass() const;

	/// Sets the Rayleigh stiffness parameter
	/// \param stiffnessCoef The Rayleigh stiffness parameter
	void setRayleighDampingStiffness(double stiffnessCoef);

	/// Sets the Rayleigh mass parameter
	/// \param massCoef The Rayleigh mass parameter
	void setRayleighDampingMass(double massCoef);

	/// Determines whether the associated coordinate is valid
	/// \param coordinate Coordinate to check
	/// \return True if coordinate is valid
	virtual bool isValidCoordinate(const SurgSim::DataStructures::IndexedLocalCoordinate& coordinate) const;

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	void beforeUpdate(double dt) override;

	void update(double dt) override;

	/// Set the compliance warping flag
	/// \param useComplianceWarping True to use compliance warping, False otherwise
	/// \exception SurgSim::Framework::AssertionFailure If the call is done after initialization
	/// \note Compliance warping is currently disabled in this version.
	void setComplianceWarping(bool useComplianceWarping);

	/// Get the compliance warping flag (default = false)
	/// \return True if compliance warping is used, False otherwise
	bool getComplianceWarping() const;

	/// If using compliance warping, set the calculation to synchronous or asynchronous. Defaults to synchronous.
	/// Synchronous calculation is slower, but after free motion the compliance matrix will be up-to-date,
	/// which means the MLCP will be as accurate as possible. Asynchronous calculation is faster, but the compliance
	/// matrix will be stale by at least one update.
	/// \param complianceWarpingSynchronous True for synchronous computation of the compliance warping matrix.
	/// \exception SurgSim::Framework::AssertionFailure If the call is done after initialization.
	void setComplianceWarpingSynchronous(bool complianceWarpingSynchronous);

	/// Get whether or not any compliance warping will be done synchronously. Default is synchronous.
	/// \return True if compliance warping is calculated synchronously.
	bool isComplianceWarpingSynchronous() const;

	/// Enable mass lumping for the mass matrix, currently we use the row sum i.e.
	/// \f$M_{ii}^{(lumped)} = \sum_{j} M_{ji}\f$
	/// \param useMassLumping whether to enable or disable lumped masses
	void setMassLumping(bool useMassLumping);

	/// \return True if lumped masses are enabled
	bool getMassLumping() const;

	/// Calculate the product C.b where C is the compliance matrix with boundary conditions
	/// \param state \f$(x, v)\f$ the current position and velocity to evaluate the various terms with
	/// \param b The input matrix b
	/// \return Returns the matrix \f$C.b\f$
	Math::Matrix applyCompliance(const Math::OdeState& state, const Math::Matrix& b) override;

	const SurgSim::Math::Matrix& getComplianceMatrix() const override;

	void updateFMDK(const SurgSim::Math::OdeState& state, int options) override;

protected:
	/// Adds the Rayleigh damping forces
	/// \param[in,out] f The force vector to cumulate the Rayleigh damping force into
	/// \param state The state vector containing positions and velocities
	/// \param useGlobalMassMatrix, useGlobalStiffnessMatrix True indicates that the global mass and stiffness matrices
	///        should be used (F = -c.M.v - d.K.v)
	/// \param scale A scaling factor to apply on the damping force
	/// \note Damping matrix D = c.M + d.K (Rayleigh damping definition)
	/// \note F = - D.v = -c.M.v - d.K.v
	/// \note If {useGlobalMassMatrix | useGlobalStiffnessMatrix} is True, {M | K} will be used
	/// \note If {useGlobalMassMatrix | useGlobalStiffnessMatrix} is False
	/// \note    the {mass|stiffness} component will be computed FemElement by FemElement
	void addRayleighDampingForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state,
								 bool useGlobalMassMatrix = false, bool useGlobalStiffnessMatrix = false,
								 double scale = 1.0);

	/// Adds the FemElements forces to f (given a state)
	/// \param[in,out] f The force vector to cumulate the FemElements forces into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the FemElements forces with
	void addFemElementsForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state, double scale = 1.0);

	/// Adds the gravity force to f (given a state)
	/// \param[in,out] f The force vector to cumulate the gravity force into
	/// \param state The state vector containing positions and velocities
	/// \param scale A scaling factor to scale the gravity force with
	/// \note This method does not do anything if gravity is disabled
	void addGravityForce(SurgSim::Math::Vector* f, const SurgSim::Math::OdeState& state, double scale = 1.0);

	bool doInitialize() override;

	/// Updates the compliance matrix using nodes transformation (useful for compliance warping)
	/// \param state The state to compute the nodes transformation from
	/// \note This computes the diagonal block matrix m_complianceWarpingTransformation and
	///       transforms the initial compliance matrix with it.
	void updateComplianceMatrix(const SurgSim::Math::OdeState& state);

	/// Retrieves a specific node transformation (useful for compliance warping)
	/// \param state The state to extract the node transformation from
	/// \param nodeId The node to update the rotation for
	/// \return The node transformation. i.e. a numDofPerNode x numDofPerNode matrix
	virtual SurgSim::Math::Matrix getNodeTransformation(const SurgSim::Math::OdeState& state, size_t nodeId);

	/// Gets the flag keeping track of the initial compliance matrix calculation (compliance warping case)
	/// \return True if the initial compliance matrix has been computed, False otherwise
	bool isInitialComplianceMatrixComputed() const;

	/// Sets the flag keeping track of the initial compliance matrix calculation (compliance warping case)
	/// \param flag True if the initial compliance matrix is computed, False otherwise
	void setIsInitialComplianceMatrixComputed(bool flag);

	void computeF(const SurgSim::Math::OdeState& state) override;

	void computeM(const SurgSim::Math::OdeState& state) override;

	void computeD(const SurgSim::Math::OdeState& state) override;

	void computeK(const SurgSim::Math::OdeState& state) override;

	void computeFMDK(const SurgSim::Math::OdeState& state) override;

	/// Useful information per node
	std::vector<double> m_massPerNode; ///< Useful in setting up the gravity force F=mg

	/// FemElements
	std::vector<std::shared_ptr<FemElement>> m_femElements;

	/// The FemElement factory parameter invoked in doInitialize() when using a MeshAsset
	/// either with LoadFem(const std::stirng& filename) or setFem(std::shared_ptr<Asset> mesh)
	/// This ensures that when using a mesh Asset, a single FemElement type is used. Therefore we do
	/// not need to define this type in the ply file, but rather is part of the Representation properties (YAML).
	std::string m_femElementType;

	bool m_useComplianceWarping; ///< Are we using Compliance Warping or not ?

	/// Is the compliance warping computation be synchronous (slower but up-to-date results)?
	bool m_isComplianceWarpingSynchronous;

	bool m_isInitialComplianceMatrixComputed; ///< For compliance warping: Is the initial compliance matrix computed ?

	SurgSim::Math::Matrix m_complianceWarpingMatrix; ///< The compliance warping matrix if compliance warping in use

	/// The system-size transformation matrix. It contains nodes transformation on the diagonal blocks.
	Eigen::SparseMatrix<double> m_complianceWarpingTransformation;

	///@{
	/// For the asynchronous compliance matrix multiplication.
	Eigen::SparseMatrix<double> m_complianceWarpingTransformationForCalculation;
	std::future<SurgSim::Math::Matrix> m_task;
	///@}

private:
	/// Rayleigh damping parameters (massCoefficient and stiffnessCoefficient)
	/// D = massCoefficient.M + stiffnessCoefficient.K
	/// Matrices: D = damping, M = mass, K = stiffness
	struct
	{
		double massCoefficient;
		double stiffnessCoefficient;
	} m_rayleighDamping;

	bool m_useMassLumping;
};

} // namespace Physics
} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMREPRESENTATION_H
