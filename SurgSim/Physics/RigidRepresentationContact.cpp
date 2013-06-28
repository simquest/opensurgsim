//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.

#include <memory>

#include <SurgSim/Physics/RigidRepresentationContact.h>
#include <SurgSim/Physics/ContactConstraintData.h>
#include <SurgSim/Physics/ConstraintImplementation.h>

#include <SurgSim/Physics/Localization.h>
#include <SurgSim/Physics/RigidRepresentationLocalization.h>

namespace SurgSim
{

namespace Physics
{

void RigidRepresentationContact::doBuild(double dt,
			const ConstraintData& data,
			MlcpPhysicsProblem& mlcp,
			unsigned int indexRepresentation,
			unsigned int indexConstraint,
			ConstraintSideSign sign)
{
	Eigen::MatrixXd& H    = mlcp.H;
	Eigen::MatrixXd& CHt  = mlcp.CHt;
	Eigen::MatrixXd& HCHt = mlcp.A;
	Eigen::VectorXd& b    = mlcp.b;
	Eigen::VectorXd& mu   = mlcp.mu;
	std::vector<SurgSim::Math::MlcpConstraintType>& constraintListTypes = mlcp.constraintTypes;

	std::shared_ptr<Representation> representation = getLocalization()->getRepresentation();
	std::shared_ptr<RigidRepresentation> rigid = std::static_pointer_cast<RigidRepresentation>(representation);

	if (! rigid->isActive())
	{
		return;
	}

	const double scale = (sign == CONSTRAINT_POSITIVE_SIDE ? 1.0 : -1.0);
	const Eigen::Matrix<double, 6,6, Eigen::DontAlign | Eigen::RowMajor>& C = rigid->getComplianceMatrix();
	const unsigned int numDof = rigid->getNumDof();
	const ContactConstraintData& contactData = static_cast<const ContactConstraintData&>(data);
	const SurgSim::Math::Vector3d& n = contactData.getNormal();
	const double d = contactData.getDistance();

	// FRICTIONLESS CONTACT in a LCP
	//   (n, d) defines the plane of contact
	//   P(t) the point of contact (usually after free motion)
	// The constraint equation is: n.P(t+dt) + d >= 0
	// n.[ P(t) + dt.V(t+dt) ] + d >= 0   (using the numerical integration scheme Backward Euler)
	// n.dt.[dG(t+dt) + w(t+dt)^GP] + n.P(t) + d >= 0
	// n.dt.[dGx(t+dt) + (wy(t+dt).GPz-wz(t+dt).GPy)] + n.P(t) + d >= 0        ]
	//      [dGy(t+dt) + (wz(t+dt).GPx-wx(t+dt).GPz)]
	//      [dGz(t+dt) + (wx(t+dt).GPy-wy(t+dt).GPx)]
	// H.v(t+dt) + b >= 0
	// H = dt.[nx  ny  nz  nz.GPy-ny.GPz  nx.GPz-nz.GPx  ny.GPx-nx.GPy]
	// b = n.P(t) + d             -> P(t) evaluated after free motion

	SurgSim::Math::Vector3d globalPosition = getLocalization()->calculatePosition();
	SurgSim::Math::Vector3d GP = globalPosition - rigid->getCurrentState().getPose().translation();

	// Fill up b with the constraint equation...
	double violation = n.dot(globalPosition) + d;
	b[indexConstraint] += violation * scale;

	// Fill up H with just the non null values
	H.block<1,3>(indexConstraint, indexRepresentation + 0) += dt * scale * n;
	H.block<1,3>(indexConstraint, indexRepresentation + 3) += dt * scale * GP.cross(n);

	// Fill up CH^t with just the non null values
	for (unsigned int CHt_line = 0; CHt_line < rigid->getNumDof(); CHt_line++)
	{
		CHt(indexRepresentation + CHt_line, indexConstraint) +=
			C.block<1,6>(CHt_line, 0) * H.block<1,6>(indexConstraint, indexRepresentation).transpose();
	}

	// Fill up HCHt (add 1 line and 1 column to it)
	// NOTE: HCHt is symmetric => we compute the last line and reflect it on the last column
	for (unsigned int col = 0; col < indexConstraint; col++)
	{
		HCHt(indexConstraint, col) +=
			H.block<1, 6>(indexConstraint, indexRepresentation) * CHt.block<6, 1>(indexRepresentation, col);
		HCHt(col, indexConstraint) = HCHt(indexConstraint, col);
	}
	HCHt(indexConstraint, indexConstraint) +=
		H.block<1, 6>(indexConstraint, indexRepresentation) * CHt.block<6, 1>(indexRepresentation, indexConstraint);
}

}; // namespace Physics

}; // namespace SurgSim
