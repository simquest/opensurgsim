#include "Constraint.hpp"

#include <SqAssert.h>

#include "FEM/FEMTetrahedra.h"

#include "SutureInteractions.h"
#include "NeedleInteractions.h"
#include "FEM3DInteractions.h"
#include "FEM2DInteractions.h"
#include "ToolPieceEntity.h"
#include "ToolPieceEntity_PositionBased.h"
#include "ToolPieceEntity_velBased.h"

std::deque<SutureConstraintLocalization*> Constraint::unusedSutureConstraintLocalizations;
std::deque<FEM3DConstraintLocalization*> Constraint::unusedFEM3DConstraintLocalizations;
std::deque<FEM2DConstraintLocalization*> Constraint::unusedFEM2DConstraintLocalizations;
std::deque<FEM1DConstraintLocalization*> Constraint::unusedFEM1DConstraintLocalizations;
std::deque<NeedleConstraintLocalization*> Constraint::unusedNeedleConstraintLocalizations;
std::deque<FixedConstraintLocalization*> Constraint::unusedFixedConstraintLocalizations;
std::deque<MeshConstraintLocalization*> Constraint::unusedMeshConstraintLocalizations;
std::deque<ToolConstraintLocalization*> Constraint::unusedToolConstraintLocalizations;
std::deque<RigidBodyConstraintLocalization*> Constraint::unusedRigidBodyConstraintLocalizations;

std::stack<Frictionless_Contact*> Constraint::unusedFrictionlessContactData;
std::stack<Frictional_Contact*> Constraint::unusedFrictionalContactData;
std::stack<Frictionless_Suturing*> Constraint::unusedFrictionlessSuturingData;
std::stack<Frictional_Suturing*> Constraint::unusedFrictionalSuturingData;
std::stack<Bilateral_Direction*> Constraint::unusedBilateralDirectionData;


bool Constraint::isEquilvalentTo(const Constraint& c, const double& maxLocationTolerance, const double& maxPlaneAngleTolerance, int localizationToCheck) const
{
	const double maxLocationTolerance2 = maxLocationTolerance*maxLocationTolerance;
	// Check the type
	if (c.type != type)
	{
		return false;
	}
	// Check the entities involved
	if (entityData[0] != c.entityData[0] || entityData[1] != c.entityData[1])
	{
		return false;
	}
	// Check the location of the constraint
	if (maxLocationTolerance > 0)
	{
		double pos0[3];
		double pos1[3];
		if (localizationToCheck == 0 || localizationToCheck == -1)
		{
			entity[0]->getCurrentPosition(entity_modelConstraintLocalization[0], pos0);
			c.entity[0]->getCurrentPosition(c.entity_modelConstraintLocalization[0], pos1);
			pos0[0] -= pos1[0];
			pos0[1] -= pos1[1];
			pos0[2] -= pos1[2];
			if (pos0[0]*pos0[0] + pos0[1]*pos0[1] + pos0[2]*pos0[2] > maxLocationTolerance2)
			{
				return false;
			}
		}
		if (localizationToCheck == 1 || localizationToCheck == -1)
		{
			entity[1]->getCurrentPosition(entity_modelConstraintLocalization[1], pos0);
			c.entity[1]->getCurrentPosition(c.entity_modelConstraintLocalization[1], pos1);
			pos0[0] -= pos1[0];
			pos0[1] -= pos1[1];
			pos0[2] -= pos1[2];
			if (pos0[0]*pos0[0] + pos0[1]*pos0[1] + pos0[2]*pos0[2] > maxLocationTolerance2)
			{
				return false;
			}
		}
	}

	// Check the geometry
	switch (type)
	{
	case CONSTRAINT_FRICTIONLESS_CONTACT:
		if (maxPlaneAngleTolerance > 0)
		{
			const Frictionless_Contact* c1=static_cast<Frictionless_Contact*>(constraintDataStructure);
			const Frictionless_Contact* c2=static_cast<Frictionless_Contact*>(c.constraintDataStructure);
			const SqVec3d n1(c1->n[0], c1->n[1], c1->n[2]);
			const SqVec3d n2(c2->n[0], c2->n[1], c2->n[2]);
			const double angle = atan2(n1.cross(n2).norm(), n1.dot(n2));
			if (abs(angle) > maxPlaneAngleTolerance)
			{
				return false;
			}
		}
		break;
	case CONSTRAINT_FRICTIONAL_CONTACT:
	{
		const Frictional_Contact* c1=static_cast<Frictional_Contact*>(constraintDataStructure);
		const Frictional_Contact* c2=static_cast<Frictional_Contact*>(c.constraintDataStructure);
		const SqVec3d n1(c1->n[0], c1->n[1], c1->n[2]);
		const SqVec3d n2(c2->n[0], c2->n[1], c2->n[2]);
		if (maxPlaneAngleTolerance > 0)
		{
			const double angle = atan2(n1.cross(n2).norm(), n1.dot(n2));
			if (abs(angle) > maxPlaneAngleTolerance)
			{
				return false;
			}
		}
		if (c1->frictionCoef != c2->frictionCoef)
		{
			return false;
		}
	}
	break;
	case CONSTRAINT_FRICTIONLESS_SLIDING:
		if (maxPlaneAngleTolerance > 0)
		{
			const Frictionless_Suturing* c1=static_cast<Frictionless_Suturing*>(constraintDataStructure);
			const Frictionless_Suturing* c2=static_cast<Frictionless_Suturing*>(c.constraintDataStructure);
			SqVec3d n1(c1->n1[0], c1->n1[1], c1->n1[2]);
			SqVec3d n2(c2->n1[0], c2->n1[1], c2->n1[2]);
			double angle = atan2(n1.cross(n2).norm(), n1.dot(n2));
			if (abs(angle) > maxPlaneAngleTolerance)
			{
				return false;
			}
			n1.set(c1->n2[0], c1->n2[1], c1->n2[2]);
			n2.set(c2->n2[0], c2->n2[1], c2->n2[2]);
			angle = atan2(n1.cross(n2).norm(), n1.dot(n2));
			if (abs(angle) > maxPlaneAngleTolerance)
			{
				return false;
			}
		}
		break;
	case CONSTRAINT_FRICTIONAL_SLIDING:
	{
		const Frictional_Suturing* c1=static_cast<Frictional_Suturing*>(constraintDataStructure);
		const Frictional_Suturing* c2=static_cast<Frictional_Suturing*>(c.constraintDataStructure);
		SqVec3d n1(c1->n1[0], c1->n1[1], c1->n1[2]);
		SqVec3d n2(c2->n1[0], c2->n1[1], c2->n1[2]);
		if (maxPlaneAngleTolerance > 0)
		{
			double angle = atan2(n1.cross(n2).norm(), n1.dot(n2));
			if (abs(angle) > maxPlaneAngleTolerance)
			{
				return false;
			}
			n1.set(c1->n2[0], c1->n2[1], c1->n2[2]);
			n2.set(c2->n2[0], c2->n2[1], c2->n2[2]);
			angle = atan2(n1.cross(n2).norm(), n1.dot(n2));
			if (abs(angle) > maxPlaneAngleTolerance)
			{
				return false;
			}
		}
		if (c1->frictionCoef != c2->frictionCoef)
		{
			return false;
		}
	}
	break;
	default:
		SQ_ASSERT_WARNING(false, "Not implemented");
		return false;
	}
	return true;
}

bool Constraint::operator ==(const Constraint& c) const
{
	// It does not matter when the constraints are happening, are they equal geometrically !?
	//if( fabs(c.localTimeOfImpact-localTimeOfImpact)>1-6 ) return false;

	if (c.type!=type)
	{
		return false;
	}
	switch (type)
	{
	case CONSTRAINT_BILATERAL_3D:
	case CONSTRAINT_BILATERAL_DIRECTION:
		break;
	case CONSTRAINT_FRICTIONLESS_CONTACT:
	{
		Frictionless_Contact* c1=static_cast<Frictionless_Contact*>(constraintDataStructure);
		Frictionless_Contact* c2=static_cast<Frictionless_Contact*>(c.constraintDataStructure);
		// This test is too strict...it will barely happens, we need a softer one !
		//if(c1->n[0]!=c2->n[0] || c1->n[1]!=c2->n[1] || c1->n[2]!=c2->n[2] || c1->n[3]!=c2->n[3])
		//	return false;
		double n1_dot_n2 = c1->n[0]*c2->n[0] + c1->n[1]*c2->n[1] + c1->n[2]*c2->n[2];
		if (fabs(n1_dot_n2-1.0)>1e-6)
		{
			return false;
		}
		if (fabs(c1->n[3]-c2->n[3])>1e-5)
		{
			return false;
		}
	}
	break;
	case CONSTRAINT_FRICTIONAL_CONTACT:
	{
		Frictional_Contact* c1=static_cast<Frictional_Contact*>(constraintDataStructure);
		Frictional_Contact* c2=static_cast<Frictional_Contact*>(c.constraintDataStructure);
		// This test is too strict...it will barely happens, we need a softer one !
		//if(c1->n[0]!=c2->n[0] || c1->n[1]!=c2->n[1] || c1->n[2]!=c2->n[2] || c1->n[3]!=c2->n[3])
		//	return false;
		double n1_dot_n2 = c1->n[0]*c2->n[0] + c1->n[1]*c2->n[1] + c1->n[2]*c2->n[2];
		if (fabs(n1_dot_n2-1.0)>1e-6)
		{
			return false;
		}
		if (fabs(c1->n[3]-c2->n[3])>1e-5)
		{
			return false;
		}
		// If the normal is equal => (t1,t2) define the same plane...so no need to check the vectors
		// Moreover, given how we build (t1,t2), a slight change in n can cause a totally different base (t1,t2)
		// they still define the same plane, and that is all it matters...so DO NOT CHECK THEM HERE !
		//if(c1->t1[0]!=c2->t1[0] || c1->t1[1]!=c2->t1[1] || c1->t1[2]!=c2->t1[2] || c1->t1[3]!=c2->t1[3])
		//	return false;
		//if(c1->t2[0]!=c2->t2[0] || c1->t2[1]!=c2->t2[1] || c1->t2[2]!=c2->t2[2] || c1->t2[3]!=c2->t2[3])
		//	return false;
		if (c1->frictionCoef!=c2->frictionCoef)
		{
			return false;
		}
	}
	break;
	case CONSTRAINT_FRICTIONLESS_SLIDING:
	{
		Frictionless_Suturing* c1=static_cast<Frictionless_Suturing*>(constraintDataStructure);
		Frictionless_Suturing* c2=static_cast<Frictionless_Suturing*>(c.constraintDataStructure);
		// This test is too strict...it will barely happens, we need a softer one !
		//if(c1->n1[0]!=c2->n1[0] || c1->n1[1]!=c2->n1[1] || c1->n1[2]!=c2->n1[2] || c1->n1[3]!=c2->n1[3])
		//	return false;
		//if(c1->n2[0]!=c2->n2[0] || c1->n2[1]!=c2->n2[1] || c1->n2[2]!=c2->n2[2] || c1->n2[3]!=c2->n2[3])
		//	return false;

		double n1_dot_n2 = c1->n1[0]*c2->n1[0] + c1->n1[1]*c2->n1[1] + c1->n1[2]*c2->n1[2];
		if (fabs(n1_dot_n2-1.0)>1e-6)
		{
			return false;
		}
		if (fabs(c1->n1[3]-c2->n1[3])>1e-5)
		{
			return false;
		}

		n1_dot_n2 = c1->n2[0]*c2->n2[0] + c1->n2[1]*c2->n2[1] + c1->n2[2]*c2->n2[2];
		if (fabs(n1_dot_n2-1.0)>1e-6)
		{
			return false;
		}
		if (fabs(c1->n2[3]-c2->n2[3])>1e-5)
		{
			return false;
		}
	}
	break;
	case CONSTRAINT_FRICTIONAL_SLIDING:
	{
		Frictional_Suturing* c1=static_cast<Frictional_Suturing*>(constraintDataStructure);
		Frictional_Suturing* c2=static_cast<Frictional_Suturing*>(c.constraintDataStructure);
		// This test is too strict...it will barely happens, we need a softer one !
		//if(c1->n1[0]!=c2->n1[0] || c1->n1[1]!=c2->n1[1] || c1->n1[2]!=c2->n1[2] || c1->n1[3]!=c2->n1[3])
		//	return false;
		//if(c1->n2[0]!=c2->n2[0] || c1->n2[1]!=c2->n2[1] || c1->n2[2]!=c2->n2[2] || c1->n2[3]!=c2->n2[3])
		//	return false;
		//if(c1->t[0]!=c2->t[0] || c1->t[1]!=c2->t[1] || c1->t[2]!=c2->t[2] || c1->t[3]!=c2->t[3])
		//	return false;
		double n1_dot_n2 = c1->n1[0]*c2->n1[0] + c1->n1[1]*c2->n1[1] + c1->n1[2]*c2->n1[2];
		if (fabs(n1_dot_n2-1.0)>1e-6)
		{
			return false;
		}
		if (fabs(c1->n1[3]-c2->n1[3])>1e-5)
		{
			return false;
		}

		n1_dot_n2 = c1->n2[0]*c2->n2[0] + c1->n2[1]*c2->n2[1] + c1->n2[2]*c2->n2[2];
		if (fabs(n1_dot_n2-1.0)>1e-6)
		{
			return false;
		}
		if (fabs(c1->n2[3]-c2->n2[3])>1e-5)
		{
			return false;
		}

		if (c1->frictionCoef!=c2->frictionCoef)
		{
			return false;
		}
	}
	break;
	case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
	case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
		break;
	case CONSTRAINT_INVALID:
	default:
		return false;
	}

	if (c.entityData[0]==entityData[0] && c.entityData[1]==entityData[1])
	{
		bool res=false;
		if (entity[0] && c.entity[0])
		{
			res|=entity[0]->areLocalizationsEqual(entity_modelConstraintLocalization[0],c.entity_modelConstraintLocalization[0]);
		}
		if (entity[1] && c.entity[1])
		{
			res|=entity[1]->areLocalizationsEqual(entity_modelConstraintLocalization[1],c.entity_modelConstraintLocalization[1]);
		}
		return res;
	}

	if (c.entityData[0]==entityData[1] && c.entityData[1]==entityData[0])
	{
		bool res=false;
		if (entity[0] && c.entity[1])
		{
			res|=entity[0]->areLocalizationsEqual(entity_modelConstraintLocalization[0],c.entity_modelConstraintLocalization[1]);
		}
		if (entity[1] && c.entity[0])
		{
			res|=entity[1]->areLocalizationsEqual(entity_modelConstraintLocalization[1],c.entity_modelConstraintLocalization[0]);
		}
		return res;
	}

	return false;
}

bool Constraint::getConstraintForceRecipe(int entityNum, LambdaWeightedForceVector& recipe) const
{
	// XXX TODO: there is now a lot of duplication of logic between this method and getConstraintForcePosition().
	//   We should try to consolidate that (e.g. implement getConstraintForcePosition using this) to avoid accidental bifurcation in the future. --bert

	bool debugThisFnc = false;

	recipe.clear();

	if (type == CONSTRAINT_INVALID)
	{
		return false;
	}
	else if (m_firstLambdaGlobalIndex < 0)
	{
		if (! m_cachedLambda)
		{
			SQ_WARNING(std::string("No available lambda to compute force for constraint ") + getTypeByName(type) + " (missing index+lambda)");
		}
		else
		{
			SQ_WARNING(std::string("No available lambda to compute force for constraint ") + getTypeByName(type) + " (missing index only)");
		}
		return false;
	}
	else if (! m_cachedLambda)
	{
		// This means a lambda index was cached, but a pointer to the lambda itself was not.
		// This happens when there's a lambda DOF but the solver failed, so the lambda values are garbage.
		// The fast haptics may find a solution after the violations change, so we keep going with the index.
	}

	double entityDirection = ((entityNum == 0) ? +1 : -1);
	int    firstIndex      = m_firstLambdaGlobalIndex;

	{
		SqVec3d location;
		getConstraintPosition(entityNum, location.getPtr());
		recipe.setLocation(location);
	}

	switch (type)
	{
	case CONSTRAINT_BILATERAL_TWIST:
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(1,0,0), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		// XXX FIXME: this only updates force[0].  For now, we zero out the other indices, but this is a hack; we ought to apply a direction.
		SQ_WARNING_ONCE("force calculations are only partially implemented for CONSTRAINT_BILATERAL_TWIST");
		break;

	case CONSTRAINT_BILATERAL_3D:
		// The lambdas are the forces in each of the world coordinate directions.
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(1,0,0), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+1, entityDirection * SqVec3d(0,1,0), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+2, entityDirection * SqVec3d(0,0,1), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		break;

	case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
		// The lambdas are the forces in each of the world coordinate directions.
		// (IS THAT REALLY CORRECT?  The code in getConstraintForcePosition() certainly assumes so as of this writing...)
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(1,0,0), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+1, entityDirection * SqVec3d(0,1,0), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+2, entityDirection * SqVec3d(0,0,1), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		break;

	case CONSTRAINT_FRICTIONLESS_CONTACT:
	{
		Frictionless_Contact* constraintData = static_cast<Frictionless_Contact*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONLESS_CONTACT constraint to expected type");
		}

		// The lambda is the force in the direction of the normal.
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(constraintData->n), LambdaWeightedForceVector::DISCARD_ALL_IF_NEGATIVE_LAMBDA);
	}
	break;

	case CONSTRAINT_FRICTIONAL_CONTACT:
	{
		Frictional_Contact* constraintData = static_cast<Frictional_Contact*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONAL_CONTACT constraint to expected type");
		}

		// The lambdas are the forces in the direction of the normal and each of the tangents, respectively.
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(constraintData->n),  LambdaWeightedForceVector::DISCARD_ALL_IF_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+1, entityDirection * SqVec3d(constraintData->t1), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+2, entityDirection * SqVec3d(constraintData->t2), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
	}
	break;

	case CONSTRAINT_FRICTIONLESS_SLIDING:
	{
		Frictionless_Suturing* constraintData = static_cast<Frictionless_Suturing*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONLESS_SLIDING constraint to expected type");
		}

		// The lambdas are the forces in the direction of each of the plane normals that define the constraint.
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(constraintData->n1), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+1, entityDirection * SqVec3d(constraintData->n2), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
	}
	break;

	case CONSTRAINT_FRICTIONAL_SLIDING:
	{
		Frictional_Suturing* constraintData = static_cast<Frictional_Suturing*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONAL_SLIDING constraint to expected type");
		}

		// The lambdas are the forces in the direction of each of the planes that define the constraint, and the tangent direction, respectively.
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(constraintData->n1), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+1, entityDirection * SqVec3d(constraintData->n2), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+2, entityDirection * SqVec3d(constraintData->t),  LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
	}
	break;

	case CONSTRAINT_BILATERAL_DIRECTION:
	{
		Bilateral_Direction* constraintData = static_cast<Bilateral_Direction*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a BILATERAL_DIRECTION constraint to expected type");
		}

		// The lambdas are the forces in the direction of each of the planes that define the constraint.
		recipe.addLambda(firstIndex+0, entityDirection * SqVec3d(constraintData->n1), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
		recipe.addLambda(firstIndex+1, entityDirection * SqVec3d(constraintData->n2), LambdaWeightedForceVector::ALLOW_NEGATIVE_LAMBDA);
	}
	break;

	case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
		SQ_WARNING_ONCE("force calculations not implemented for CONSTRAINT_BILATERAL_AXIAL_ROTATION");
		// XXX TODO
		break;

	default:
		SQ_FAILURE("constraint has an unexpected type index");
		return false;
	}

	SQ_ASSERT(m_outputForceScale >= 0, "Scale of forces output to haptics must be non-negative!");
	recipe.setOutputForceScale(m_outputForceScale);

	if (debugThisFnc)
	{
		cout << "Adding recipe for " << getTypeByName(type) << " constraint, " << recipe.getNumLambdas() << " ingredients." << endl;
	}
	return true;
}

bool Constraint::getConstraintForcePosition(int iEntity, double force[3], double point[3], double& toi) const
{
	// XXX TODO: there is now a lot of duplication of logic between this method and getConstraintForceRecipe().
	//   We should try to consolidate that (e.g. implement this using getConstraintForceRecipe) to avoid accidental bifurcation in the future. --bert

	bool debugThisFnc = false;
	// For each side of the constraint
	if (type == CONSTRAINT_INVALID || ! m_cachedLambda)
	{
		for (int jj=0; jj<3; ++jj)
		{
			point[jj] = 0;
			force[jj] = 0;
		}
		toi = -1;
		if (debugThisFnc)
		{
			cout << "No available lambda to compute force for constraint " << getTypeByName(type) << endl;
		}
		return false;
	}
	else
	{
		getConstraintPosition(iEntity, point);
		toi = globalTimeOfImpact;
		switch (type)
		{
		case CONSTRAINT_BILATERAL_TWIST:
			for (int jj=0; jj<1; ++jj)
			{
				force[jj] = (0==iEntity?1:-1)*m_cachedLambda[jj];
			}
			// XXX FIXME: this only updates force[0].  For now, we zero out the other indices, but this is a hack; we ought to apply a direction.
			force[1] = force[2] = 0;  //XXX!!!XXX
			SQ_WARNING_ONCE("force calculations are only partially implemented for CONSTRAINT_BILATERAL_TWIST");
			break;

		case CONSTRAINT_BILATERAL_3D:
			for (int jj=0; jj<3; ++jj)
			{
				force[jj] = (0==iEntity?1:-1)*m_cachedLambda[jj];
			}
			break;
		case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
		{
			for (int jj=0; jj<3; ++jj)
			{
				force[jj] = (0==iEntity?1:-1)*m_cachedLambda[jj];
			}
			break;
		}
		case CONSTRAINT_FRICTIONLESS_CONTACT:
		{
			Frictionless_Contact* constraintData = static_cast<Frictionless_Contact*>(constraintDataStructure);
			if (! constraintData)
			{
				SQ_FAILURE("could not cast a FRICTIONLESS_CONTACT constraint to expected type");
			}
			else
			{
				for (int jj=0; jj<3; ++jj)
				{
					force[jj] = (0==iEntity?1:-1)*m_cachedLambda[0]*constraintData->n[jj];
				}
			}
			break;
		}
		case CONSTRAINT_FRICTIONAL_CONTACT:
		{
			Frictional_Contact* constraintData = static_cast<Frictional_Contact*>(constraintDataStructure);
			if (! constraintData)
			{
				SQ_FAILURE("could not cast a FRICTIONAL_CONTACT constraint to expected type");
			}
			else
			{
				for (int jj=0; jj<3; ++jj)
				{
					force[jj]  = (0==iEntity?1:-1)*m_cachedLambda[0]*constraintData->n[jj];
					force[jj] += (0==iEntity?1:-1)*m_cachedLambda[1]*constraintData->t1[jj];
					force[jj] += (0==iEntity?1:-1)*m_cachedLambda[2]*constraintData->t2[jj];
				}
			}
			break;
		}
		case CONSTRAINT_FRICTIONLESS_SLIDING:
		{
			Frictionless_Suturing* constraintData = static_cast<Frictionless_Suturing*>(constraintDataStructure);
			if (! constraintData)
			{
				SQ_FAILURE("could not cast a FRICTIONLESS_SLIDING constraint to expected type");
			}
			else
			{
				for (int jj=0; jj<3; ++jj)
				{
					force[jj]  = (0==iEntity?1:-1)*m_cachedLambda[0]*constraintData->n1[jj];
					force[jj] += (0==iEntity?1:-1)*m_cachedLambda[1]*constraintData->n2[jj];
				}
			}
			break;
		}
		case CONSTRAINT_FRICTIONAL_SLIDING:
		{
			Frictional_Suturing* constraintData = static_cast<Frictional_Suturing*>(constraintDataStructure);
			if (! constraintData)
			{
				SQ_FAILURE("could not cast a FRICTIONAL_SLIDING constraint to expected type");
			}
			else
			{
				for (int jj=0; jj<3; ++jj)
				{
					force[jj]  = (0==iEntity?1:-1)*m_cachedLambda[0]*constraintData->n1[jj];
					force[jj] += (0==iEntity?1:-1)*m_cachedLambda[1]*constraintData->n2[jj];
					force[jj] += (0==iEntity?1:-1)*m_cachedLambda[2]*constraintData->t[jj];
				}
			}
			break;
		}
		case CONSTRAINT_BILATERAL_DIRECTION:
		{
			Bilateral_Direction* constraintData = static_cast<Bilateral_Direction*>(constraintDataStructure);
			if (! constraintData)
			{
				SQ_FAILURE("could not cast a BILATERAL_DIRECTION constraint to expected type");
			}
			else
			{
				for (int jj=0; jj<3; ++jj)
				{
					force[jj] = (0==iEntity?1:-1)*m_cachedLambda[0]*constraintData->n1[jj];
					force[jj] += (0==iEntity?1:-1)*m_cachedLambda[1]*constraintData->n2[jj];
				}
			}
			break;
		}
		case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
			SQ_WARNING_ONCE("force calculations not implemented for CONSTRAINT_BILATERAL_AXIAL_ROTATION");
			// XXX TODO
			break;
		default:
			SQ_FAILURE("constraint has an unexpected type index");
			return false;
		}
		if (debugThisFnc)
			cout << "Adding " << getTypeByName(type) << " constraint, force = [" <<
			     force[0] << " " << force[1] << " " << force[2] << "] @ [ " <<
			     point[0] << " " << point[1] << " " << point[2] << "]" << endl;
		return true;
	}
}

void Constraint::getConstraintPosition(int entityNum, double point[3]) const
{
	SQ_ASSERT(entityNum==0||entityNum==1, "Bad entity number");

	SQ_ASSERT(entity[entityNum]->entityType() != INVALID_ENTITY && entity[entityNum]->entityType() != EMPTY_ENTITY,
	          "Cannot get constraint position for invalid or empty entity "+entity[entityNum]->getName());

	// If it is not a fixed entity, we must be able to get the position directly from the entity
	if (entity[entityNum]->entityType() != FIXED_ENTITY)
	{
		entity[entityNum]->getCurrentPosition(entity_modelConstraintLocalization[entityNum], point);
	}
	// If it is a fixed entity, get the position from the fixed constraint localization
	else
	{
		FixedConstraintLocalization* fixedLocalization = dynamic_cast<FixedConstraintLocalization*>(entity_modelConstraintLocalization[entityNum]);
		for (int i = 0; i < 3; i++)
		{
			point[i] = fixedLocalization->Position[i];
		}
	}
}

bool Constraint::getConstraintFriction(int iEntity, double friction[3]) const
{
	bool debugThisFnc = false;
	// For each side of the constraint
	if (type == CONSTRAINT_INVALID || ! m_cachedLambda)
	{
		for (int jj=0; jj<3; ++jj)
		{
			friction[jj] = 0;
		}
		if (debugThisFnc)
		{
			cout << "No available lambda to compute force for constraint " << getTypeByName(type) << endl;
		}
		return false;
	}
	else
	{
		switch (type)
		{
		case CONSTRAINT_FRICTIONAL_CONTACT:
		{
			Frictional_Contact* constraintData = static_cast<Frictional_Contact*>(constraintDataStructure);
			if (constraintData)
			{
				for (int jj=0; jj<3; ++jj)
				{
					friction[jj] = (0==iEntity?1:-1)*m_cachedLambda[1]*constraintData->t1[jj];
					friction[jj] += (0==iEntity?1:-1)*m_cachedLambda[2]*constraintData->t2[jj];
				}
			}
			else
			{
				SQ_FAILURE("could not cast a FRICTIONAL_CONTACT constraint to expected type");
			}
			break;
		}
		case CONSTRAINT_FRICTIONAL_SLIDING:
		{
			Frictional_Suturing* constraintData = static_cast<Frictional_Suturing*>(constraintDataStructure);
			if (constraintData)
			{
				for (int jj=0; jj<3; ++jj)
				{
					friction[jj] = (0==iEntity?1:-1)*m_cachedLambda[2]*constraintData->t[jj];
				}
			}
			else
			{
				SQ_FAILURE("could not cast a FRICTIONAL_SLIDING constraint to expected type");
			}
			break;
		}
		case CONSTRAINT_BILATERAL_TWIST:
		case CONSTRAINT_BILATERAL_3D:
		case CONSTRAINT_FRICTIONLESS_CONTACT:
		case CONSTRAINT_FRICTIONLESS_SLIDING:
		case CONSTRAINT_BILATERAL_DIRECTION:
		case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
		case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
			for (int jj=0; jj<3; ++jj)
			{
				friction[jj] = 0.0;
			}
			break;
		default:
			SQ_FAILURE("constraint has an unexpected type index");
			return false;
		}
		return true;
	}
}

const double Constraint::getOutputForceScale() const
{
	return m_outputForceScale;
}

bool Constraint::getViolationAdjustmentRecipe(int entityNum, ElementAdjustmentFromPoseProjection& recipe) const
{
	// XXX TODO: there is now a lot of duplication of logic between this method, various (mostly tool/rigid body) buildMLCP_* methods, and even getConstraintForceRecipe().
	//   We should try to consolidate that (e.g. implement getConstraintForcePosition using this) to avoid accidental bifurcation in the future. --bert

	bool debugThisFnc = false;

	recipe.clear();

	if (type == CONSTRAINT_INVALID)
	{
		return false;
	}

	double entityDirection = ((entityNum == 0) ? +1 : -1);
	int    firstIndex      = m_firstLambdaGlobalIndex;

	SqVec3d worldLocation;
	getConstraintPosition(entityNum, worldLocation.getPtr());
	recipe.setWorldLocation(worldLocation);

	float pose[7];
	{
		RigidBodyInteractions*               rigidBodyPositionModel = dynamic_cast<RigidBodyInteractions*>(entity[entityNum]);
		RigidBodyInteractions_VelocityBased* rigidBodyVelocityModel = dynamic_cast<RigidBodyInteractions_VelocityBased*>(entity[entityNum]);
		if (rigidBodyPositionModel)
		{
			rigidBodyPositionModel->getCurrentModelFramePose(pose);
		}
		else if (rigidBodyVelocityModel)
		{
			rigidBodyVelocityModel->getCurrentModelFramePose(pose);
		}
		else
		{
			printf("entity type is %d %s\n", entity[entityNum]->entityType(), entity[entityNum]->entityTypeName().c_str());
			SQ_WARNING("tried to call getViolationAdjustmentRecipe for a non-rigid body; not supported yet");
			return false;
		}
	}

	SqTransformd rigidBodyPose(SqQuaterniond(pose[3],pose[4],pose[5],pose[6]), SqVec3d(pose[0],pose[1],pose[2]));
	SqVec3d localPoint = rigidBodyPose.inverse() * worldLocation;  // THIS IS A BIG HACK...
	recipe.setLocalLocation(localPoint);

	switch (type)
	{
	case CONSTRAINT_BILATERAL_TWIST:
		SQ_WARNING_ONCE("violation adjustments not implemented for CONSTRAINT_BILATERAL_TWIST");
		// XXX TODO
		break;

	case CONSTRAINT_BILATERAL_3D:
		// The violations should be offset by the change of position in the world X,Y,Z directions respectively.
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(1,0,0));
		recipe.addAdjustment(firstIndex+1, entityDirection * SqVec3d(0,1,0));
		recipe.addAdjustment(firstIndex+2, entityDirection * SqVec3d(0,0,1));
		break;

	case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
		// The violations should be offset by the change of position in the world X,Y,Z directions respectively.
		// XXX TODO: I think this is wrong... --bert
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(1,0,0));
		recipe.addAdjustment(firstIndex+1, entityDirection * SqVec3d(0,1,0));
		recipe.addAdjustment(firstIndex+2, entityDirection * SqVec3d(0,0,1));
		break;

	case CONSTRAINT_FRICTIONLESS_CONTACT:
	{
		Frictionless_Contact* constraintData = static_cast<Frictionless_Contact*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONLESS_CONTACT constraint to expected type");
		}

		// The violation should be offset by the change of position in the direction of the normal.
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(constraintData->n));
	}
	break;

	case CONSTRAINT_FRICTIONAL_CONTACT:
	{
		Frictional_Contact* constraintData = static_cast<Frictional_Contact*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONAL_CONTACT constraint to expected type");
		}

		// The violations should be offset by the change of position in the direction of the normal and each of the tangents, respectively.
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(constraintData->n));
		recipe.addAdjustment(firstIndex+1, entityDirection * SqVec3d(constraintData->t1));
		recipe.addAdjustment(firstIndex+2, entityDirection * SqVec3d(constraintData->t2));
	}
	break;

	case CONSTRAINT_FRICTIONLESS_SLIDING:
	{
		Frictionless_Suturing* constraintData = static_cast<Frictionless_Suturing*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONLESS_SLIDING constraint to expected type");
		}

		// The violations should be offset by the change of position in the normal direction of each of the planes that define the constraint.
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(constraintData->n1));
		recipe.addAdjustment(firstIndex+1, entityDirection * SqVec3d(constraintData->n2));
	}
	break;

	case CONSTRAINT_FRICTIONAL_SLIDING:
	{
		Frictional_Suturing* constraintData = static_cast<Frictional_Suturing*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a FRICTIONAL_SLIDING constraint to expected type");
		}

		// The violations should be offset by the change of position in the normal direction of each of the planes that define the constraint, and the tangent direction.
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(constraintData->n1));
		recipe.addAdjustment(firstIndex+1, entityDirection * SqVec3d(constraintData->n2));
		recipe.addAdjustment(firstIndex+2, entityDirection * SqVec3d(constraintData->t));
	}
	break;

	case CONSTRAINT_BILATERAL_DIRECTION:
	{
		Bilateral_Direction* constraintData = static_cast<Bilateral_Direction*>(constraintDataStructure);
		if (! constraintData)
		{
			SQ_FAILURE("could not cast a BILATERAL_DIRECTION constraint to expected type");
		}

		// The violations should be offset by the change of position in the normal direction of each of the planes that define the constraint.
		recipe.addAdjustment(firstIndex+0, entityDirection * SqVec3d(constraintData->n1));
		recipe.addAdjustment(firstIndex+1, entityDirection * SqVec3d(constraintData->n2));
	}
	break;

	case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
		SQ_WARNING_ONCE("violation adjustments not implemented for CONSTRAINT_BILATERAL_AXIAL_ROTATION");
		// XXX TODO
		break;

	default:
		SQ_FAILURE("constraint has an unexpected type index");
		return false;
	}

	if (debugThisFnc)
	{
		//cout << "Adding recipe for " << getTypeByName(type) << " constraint, " << recipe.getNumXxx() << " ingredients." << endl;
	}
	return true;
}

std::string Constraint::getTypeByName(int type)
{
	switch (type)
	{
	case CONSTRAINT_INVALID:
		return "Invalid";
	case CONSTRAINT_BILATERAL_3D:
		return "Bilateral 3d";
	case CONSTRAINT_BILATERAL_DIRECTION:
		return "Directional";
	case CONSTRAINT_FRICTIONLESS_CONTACT:
		return "Frictionless Contact";
	case CONSTRAINT_FRICTIONAL_CONTACT:
		return "Frictional Contact";
	case CONSTRAINT_FRICTIONLESS_SLIDING:
		return "Frictionless Sliding";
	case CONSTRAINT_FRICTIONAL_SLIDING:
		return "Frictional Sliding";
	case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
		return "Angular Velocity";
	case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
		return "Axial";
	default:
		return "Unknown";
	}
}

std::ostream& operator<<(std::ostream& os, const ConstraintLocalization& loc)
{
	switch (loc.getType())
	{
	case ConstraintLocalization::SUTURE_LOCALIZATION:
	{
		const SutureConstraintLocalization* sutureLoc = static_cast<const SutureConstraintLocalization*>(&loc);
		os << "Cylinder id = " << sutureLoc->cylID << ", abscissa = " << sutureLoc->localAbs;
	}
	break;
	case ConstraintLocalization::FEM1D_LOCALIZATION:
	{
		const FEM1DConstraintLocalization* femLoc = static_cast<const FEM1DConstraintLocalization*>(&loc);
		os << "Beam id = " << femLoc->beamID << ", abscissa = " <<  femLoc->abs;
	}
	break;
	case ConstraintLocalization::FEM2D_LOCALIZATION:
	{
		const FEM2DConstraintLocalization* femLoc = static_cast<const FEM2DConstraintLocalization*>(&loc);
		os << "Triangle id = " << femLoc->triID << ", barycentric coords = [ " << SqVec3d(femLoc->triBarycentricCoord) << " ]";
	}
	break;
	case ConstraintLocalization::FEM3D_LOCALIZATION:
	{
		const FEM3DConstraintLocalization* femLoc = static_cast<const FEM3DConstraintLocalization*>(&loc);
		os << "Triangle id = " << femLoc->triID << ", barycentric coords = [ " <<
		   femLoc->triBarycentricCoord[0] << " " << femLoc->triBarycentricCoord[1] << " " << femLoc->triBarycentricCoord[2] << " ]" <<
		   ", Tetrahedron id = " << femLoc->tetID << ", barycentric coords = " <<
		   femLoc->tetBarycentricCoord[0] << " " << femLoc->tetBarycentricCoord[1] << " " << femLoc->tetBarycentricCoord[2] << " " << femLoc->tetBarycentricCoord[3] << " ]";
	}
	break;
	case ConstraintLocalization::NEEDLE_LOCALIZATION:
	{
		const NeedleConstraintLocalization* needleLoc = static_cast<const NeedleConstraintLocalization*>(&loc);
		os << "Points id = [ " << needleLoc->ptID1 << " , " << needleLoc->ptID2 << " ]" <<
		   ", abscissa = " << needleLoc->abs;
	}
	break;
	case ConstraintLocalization::FIXED_LOCALIZATION:
	{
		const FixedConstraintLocalization* fixedLoc = static_cast<const FixedConstraintLocalization*>(&loc);
		os << "Position = [ " << fixedLoc->Position[0] << " " << fixedLoc->Position[1] << " " << fixedLoc->Position[2] << " ], twist = " << fixedLoc->Twist;
	}
	break;
	case ConstraintLocalization::MESH_LOCALIZATION:
	{
		const MeshConstraintLocalization* meshLoc = static_cast<const MeshConstraintLocalization*>(&loc);
		os << "Triangle id = " << meshLoc->triID << ", barycentric coords = [ " <<
		   meshLoc->triBarycentricCoord[0] << " " << meshLoc->triBarycentricCoord[1] << " " << meshLoc->triBarycentricCoord[2] << " ]";
	}
	break;
	case ConstraintLocalization::TOOL_MESH_LOCALIZATION:
	{
		const MeshConstraintLocalization& meshLoc = static_cast<const ToolConstraintLocalization*>(&loc)->meshConstraintLocalization;
		os << "Triangle id = " << meshLoc.triID << ", barycentric coords = [ " <<
		   meshLoc.triBarycentricCoord[0] << " " << meshLoc.triBarycentricCoord[1] << " " << meshLoc.triBarycentricCoord[2] << " ]";
	}
	break;
	case ConstraintLocalization::TOOL_LOCAL_SPACE_LOCALIZATION:
	{
		const FixedConstraintLocalization& fixedLoc = static_cast<const ToolConstraintLocalization*>(&loc)->fixedConstraintLocalization;
		os << "Local Position = [ " << fixedLoc.Position[0] << " " << fixedLoc.Position[1] << " " << fixedLoc.Position[2] << " ], twist = " << fixedLoc.Twist;
	}
	break;
	case ConstraintLocalization::RIGID_BODY_MESH_LOCALIZATION:
	{
		const MeshConstraintLocalization& meshLoc = static_cast<const RigidBodyConstraintLocalization*>(&loc)->meshConstraintLocalization;
		os << "Triangle id = " << meshLoc.triID << ", barycentric coords = [ " <<
		   meshLoc.triBarycentricCoord[0] << " " << meshLoc.triBarycentricCoord[1] << " " << meshLoc.triBarycentricCoord[2] << " ]";
	}
	break;
	case ConstraintLocalization::RIGID_BODY_LOCAL_SPACE_LOCALIZATION:
	{
		const FixedConstraintLocalization& fixedLoc = static_cast<const RigidBodyConstraintLocalization*>(&loc)->fixedConstraintLocalization;
		os << "Local Position = [ " << fixedLoc.Position[0] << " " << fixedLoc.Position[1] << " " << fixedLoc.Position[2] << " ], twist = " << fixedLoc.Twist;
	}
	break;
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const Constraint& c)
{
	os << "Type: " << c.getTypeByName(c.getType()) << ", violation = " << c.violation <<
	   ", entities [ " << (c.entity[0]?c.entity[0]->getName():"NULL") << ", " << (c.entity[1]?c.entity[1]->getName():"NULL") << " ]" <<
	   ", TOI: [ " << c.globalTimeOfImpact << " , " << c.localTimeOfImpact << " ]" << endl;
	if (c.entity_modelConstraintLocalization[0])
	{
		os << "\t, localizations 0 = " << *c.entity_modelConstraintLocalization[0] << endl;
	}
	if (c.entity_modelConstraintLocalization[1])
	{
		os << "\t, localizations 1 = " << *c.entity_modelConstraintLocalization[1] << endl;
	}
	switch (c.getType())
	{
	case CONSTRAINT_FRICTIONLESS_CONTACT:
	{
		const Frictionless_Contact* contact = dynamic_cast<const Frictionless_Contact*>(c.constraintDataStructure);
		SQ_ASSERT(contact, "Bad contact construction");
		os << "\t, normal = " << SqVec3d(contact->n) << endl;
	}
	break;
	case CONSTRAINT_FRICTIONAL_CONTACT:
	{
		const Frictional_Contact* contact = dynamic_cast<const Frictional_Contact*>(c.constraintDataStructure);
		SQ_ASSERT(contact, "Bad contact construction");
		os << "\t, normal = " << SqVec3d(contact->n) << "\t t1 = " << SqVec3d(contact->t1) << "\t t2 = " << SqVec3d(contact->t2) << endl;
	}
	break;
	default:
		break;
	}
	return os;
}


// Method to manage the diverse Constraint from outside (retrieve/release constraints)
SutureConstraintLocalization* Constraint::retrieveSutureConstraintLocalization(void)
{
	if (! unusedSutureConstraintLocalizations.empty())
	{
		SutureConstraintLocalization* res = unusedSutureConstraintLocalizations.front();
		unusedSutureConstraintLocalizations.pop_front();
		return res;
	}
	return new SutureConstraintLocalization;
}
FEM3DConstraintLocalization* Constraint::retrieveFEM3DConstraintLocalization(void)
{
	if (! unusedFEM3DConstraintLocalizations.empty())
	{
		FEM3DConstraintLocalization* res = unusedFEM3DConstraintLocalizations.front();
		unusedFEM3DConstraintLocalizations.pop_front();
		return res;
	}
	return new FEM3DConstraintLocalization;
}
FEM2DConstraintLocalization* Constraint::retrieveFEM2DConstraintLocalization(void)
{
	if (! unusedFEM2DConstraintLocalizations.empty())
	{
		FEM2DConstraintLocalization* res = unusedFEM2DConstraintLocalizations.front();
		unusedFEM2DConstraintLocalizations.pop_front();
		return res;
	}
	return new FEM2DConstraintLocalization;
}
FEM1DConstraintLocalization* Constraint::retrieveFEM1DConstraintLocalization(void)
{
	if (! unusedFEM1DConstraintLocalizations.empty())
	{
		FEM1DConstraintLocalization* res = unusedFEM1DConstraintLocalizations.front();
		unusedFEM1DConstraintLocalizations.pop_front();
		return res;
	}
	return new FEM1DConstraintLocalization;
}
NeedleConstraintLocalization* Constraint::retrieveNeedleConstraintLocalization(void)
{
	if (! unusedNeedleConstraintLocalizations.empty())
	{
		NeedleConstraintLocalization* res = unusedNeedleConstraintLocalizations.front();
		unusedNeedleConstraintLocalizations.pop_front();
		return res;
	}
	return new NeedleConstraintLocalization;
}
FixedConstraintLocalization* Constraint::retrieveFixedConstraintLocalization(void)
{
	if (! unusedFixedConstraintLocalizations.empty())
	{
		FixedConstraintLocalization* res = unusedFixedConstraintLocalizations.front();
		unusedFixedConstraintLocalizations.pop_front();
		return res;
	}
	return new FixedConstraintLocalization;
}
MeshConstraintLocalization* Constraint::retrieveMeshConstraintLocalization(void)
{
	if (! unusedMeshConstraintLocalizations.empty())
	{
		MeshConstraintLocalization* res = unusedMeshConstraintLocalizations.front();
		unusedMeshConstraintLocalizations.pop_front();
		return res;
	}
	return new MeshConstraintLocalization;
}
ToolConstraintLocalization* Constraint::retrieveToolConstraintLocalization(void)
{
	if (! unusedToolConstraintLocalizations.empty())
	{
		ToolConstraintLocalization* res = unusedToolConstraintLocalizations.front();
		unusedToolConstraintLocalizations.pop_front();
		return res;
	}
	return new ToolConstraintLocalization;
}
RigidBodyConstraintLocalization* Constraint::retrieveRigidBodyConstraintLocalization(void)
{
	if (! unusedRigidBodyConstraintLocalizations.empty())
	{
		RigidBodyConstraintLocalization* res = unusedRigidBodyConstraintLocalizations.front();
		unusedRigidBodyConstraintLocalizations.pop_front();
		return res;
	}
	return new RigidBodyConstraintLocalization;
}

void Constraint::releaseSutureConstraintLocalization(SutureConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->cylID = -1;
		unusedSutureConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseFEM3DConstraintLocalization(FEM3DConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->tetID = -1;
		loc->triID = -1;
		unusedFEM3DConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseFEM2DConstraintLocalization(FEM2DConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->triID = -1;
		unusedFEM2DConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseFEM1DConstraintLocalization(FEM1DConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->beamID = -1;
		unusedFEM1DConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseNeedleConstraintLocalization(NeedleConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->ptID1 = -1;
		loc->ptID2 = -1;
		unusedNeedleConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseFixedConstraintLocalization(FixedConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->ElementID = -1;
		unusedFixedConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseMeshConstraintLocalization(MeshConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->triID = -1;
		unusedMeshConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseToolConstraintLocalization(ToolConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->meshConstraintLocalization.triID = -1;
		loc->fixedConstraintLocalization.ElementID = -1;
		unusedToolConstraintLocalizations.push_back(loc);
	}
}
void Constraint::releaseRigidBodyConstraintLocalization(RigidBodyConstraintLocalization* loc)
{
	if (loc)
	{
		SQ_ASSERT(!isConstraintLocalizationUnused(loc), "Releasing a constraint that is already unused!");
		loc->meshConstraintLocalization.triID = -1;
		loc->fixedConstraintLocalization.ElementID = -1;
		unusedRigidBodyConstraintLocalizations.push_back(loc);
	}
}

void Constraint::releaseConstraintLocalization(ConstraintLocalization* loc)
{
	if (! loc)
	{
		return;
	}

	switch (loc->getType())
	{
	case ConstraintLocalization::FEM1D_LOCALIZATION:
		releaseFEM1DConstraintLocalization(static_cast<FEM1DConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::FEM2D_LOCALIZATION:
		releaseFEM2DConstraintLocalization(static_cast<FEM2DConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::FEM3D_LOCALIZATION:
		releaseFEM3DConstraintLocalization(static_cast<FEM3DConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::SUTURE_LOCALIZATION:
		releaseSutureConstraintLocalization(static_cast<SutureConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::NEEDLE_LOCALIZATION:
		releaseNeedleConstraintLocalization(static_cast<NeedleConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::MESH_LOCALIZATION:
		releaseMeshConstraintLocalization(static_cast<MeshConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::FIXED_LOCALIZATION:
		releaseFixedConstraintLocalization(static_cast<FixedConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::TOOL_MESH_LOCALIZATION:
	case ConstraintLocalization::TOOL_LOCAL_SPACE_LOCALIZATION:
		releaseToolConstraintLocalization(static_cast<ToolConstraintLocalization*>(loc));
		break;
	case ConstraintLocalization::RIGID_BODY_MESH_LOCALIZATION:
	case ConstraintLocalization::RIGID_BODY_LOCAL_SPACE_LOCALIZATION:
		releaseRigidBodyConstraintLocalization(static_cast<RigidBodyConstraintLocalization*>(loc));
		break;
	default:
		SQ_FAILURE("ConstraintLocalization type unknown !");
		break;
	}
}

bool Constraint::isConstraintLocalizationUnused(ConstraintLocalization* loc)
{
	if (! loc)
	{
		return false;
	}

	switch (loc->getType())
	{
	case ConstraintLocalization::FEM1D_LOCALIZATION:
		return std::find(unusedFEM1DConstraintLocalizations.begin(), unusedFEM1DConstraintLocalizations.end(), loc) != unusedFEM1DConstraintLocalizations.end();
		break;
	case ConstraintLocalization::FEM2D_LOCALIZATION:
		return std::find(unusedFEM2DConstraintLocalizations.begin(), unusedFEM2DConstraintLocalizations.end(), loc) != unusedFEM2DConstraintLocalizations.end();
		break;
	case ConstraintLocalization::FEM3D_LOCALIZATION:
		return std::find(unusedFEM3DConstraintLocalizations.begin(), unusedFEM3DConstraintLocalizations.end(), loc) != unusedFEM3DConstraintLocalizations.end();
		break;
	case ConstraintLocalization::SUTURE_LOCALIZATION:
		return std::find(unusedSutureConstraintLocalizations.begin(), unusedSutureConstraintLocalizations.end(), loc) != unusedSutureConstraintLocalizations.end();
		break;
	case ConstraintLocalization::NEEDLE_LOCALIZATION:
		return std::find(unusedNeedleConstraintLocalizations.begin(), unusedNeedleConstraintLocalizations.end(), loc) != unusedNeedleConstraintLocalizations.end();
		break;
	case ConstraintLocalization::MESH_LOCALIZATION:
		return std::find(unusedMeshConstraintLocalizations.begin(), unusedMeshConstraintLocalizations.end(), loc) != unusedMeshConstraintLocalizations.end();
		break;
	case ConstraintLocalization::FIXED_LOCALIZATION:
		return std::find(unusedFixedConstraintLocalizations.begin(), unusedFixedConstraintLocalizations.end(), loc) != unusedFixedConstraintLocalizations.end();
		break;
	case ConstraintLocalization::TOOL_MESH_LOCALIZATION:
	case ConstraintLocalization::TOOL_LOCAL_SPACE_LOCALIZATION:
		return std::find(unusedToolConstraintLocalizations.begin(), unusedToolConstraintLocalizations.end(), loc) != unusedToolConstraintLocalizations.end();
		break;
	case ConstraintLocalization::RIGID_BODY_MESH_LOCALIZATION:
	case ConstraintLocalization::RIGID_BODY_LOCAL_SPACE_LOCALIZATION:
		return std::find(unusedRigidBodyConstraintLocalizations.begin(), unusedRigidBodyConstraintLocalizations.end(), loc) != unusedRigidBodyConstraintLocalizations.end();
		break;
	default:
		SQ_FAILURE("ConstraintLocalization type unknown !");
		break;
	}
}