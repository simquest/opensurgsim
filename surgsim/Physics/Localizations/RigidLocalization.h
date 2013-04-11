#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__

#include <stack>
#include <deque>
#include <SqAssert.h>
#include <InterfaceLib/LambdaWeightedForceVector.h>
#include <InterfaceLib/ElementAdjustmentFromPoseProjection.h>

enum EntityType { 
	INVALID_ENTITY=0x0, 
	EMPTY_ENTITY=0x1,
	SUTURE_ENTITY=0x2, 
	NEEDLE_ENTITY=0x4, 
	FIXED_ENTITY=0x8, 
	TOOL_ENTITY=0x10, 
	RIGID_BODY_ENTITY=0x20, 
	FIXED_RIGID_BODY_ENTITY=0x40, 
	FEM3D_ENTITY=0x80, 
	FEM2D_ENTITY=0x100, 
	FEM1D_ENTITY=0x200, 
	ERODIBLE_ENTITY=0x400,
	DEFORMABLE_ENTITIES=(FEM1D_ENTITY|FEM2D_ENTITY|FEM3D_ENTITY|SUTURE_ENTITY),
	NON_DEFORMABLE_ENTITIES=(NEEDLE_ENTITY|FIXED_ENTITY|TOOL_ENTITY|FIXED_RIGID_BODY_ENTITY|RIGID_BODY_ENTITY|ERODIBLE_ENTITY),
	ALL_ENTITIES=0xFFFF
	};

enum ConstraintType
{
	CONSTRAINT_INVALID = -1,

	CONSTRAINT_BILATERAL_TWIST = 0,
	CONSTRAINT_BILATERAL_3D,
	CONSTRAINT_BILATERAL_ANGULAR_VELOCITY,
	CONSTRAINT_FRICTIONLESS_CONTACT,
	CONSTRAINT_FRICTIONAL_CONTACT,
	CONSTRAINT_FRICTIONLESS_SLIDING,
	CONSTRAINT_FRICTIONAL_SLIDING,

	CONSTRAINT_BILATERAL_DIRECTION,
	CONSTRAINT_BILATERAL_AXIAL_ROTATION,

	NUM_CONSTRAINT_TYPES
};


//#######################################
// Constraint localization on the model:
//    param abs                 for suture entity
//    tet ID+barycentric coord  for FEM entity
//    3D global position        for needle entity (rigid body)
//    3D global position        for fixed entity (anchor in 3D space basically)

class ConstraintLocalization
{
public:
	enum ConstraintLocalizationType { INVALID_LOCALIZATION, EMPTY_LOCALIZATION, SUTURE_LOCALIZATION, FEM1D_LOCALIZATION, FEM2D_LOCALIZATION, 
		FEM3D_LOCALIZATION, NEEDLE_LOCALIZATION, FIXED_LOCALIZATION, MESH_LOCALIZATION, TOOL_MESH_LOCALIZATION, TOOL_LOCAL_SPACE_LOCALIZATION, RIGID_BODY_MESH_LOCALIZATION, RIGID_BODY_LOCAL_SPACE_LOCALIZATION};
	ConstraintLocalization() {};
	virtual ~ConstraintLocalization() {};
	virtual ConstraintLocalizationType getType() const
	{ return INVALID_LOCALIZATION; }
protected:
	ConstraintLocalization(const ConstraintLocalization&) {}
	ConstraintLocalization& operator= (const ConstraintLocalization&) {}
};
std::ostream& operator<<(std::ostream&, const ConstraintLocalization&);

struct SutureConstraintLocalization : public ConstraintLocalization
{
	virtual ConstraintLocalizationType getType() const
	{ return SUTURE_LOCALIZATION; }
	int cylID;
	double localAbs;
};

struct FEMConstraintLocalization : public ConstraintLocalization
{
	FEMConstraintLocalization() {}
	FEMConstraintLocalization(const FEMConstraintLocalization& other)
	{
		*this = other;
	}
};

struct FEM1DConstraintLocalization : public FEMConstraintLocalization
{
	int beamID;
	double abs;

	FEM1DConstraintLocalization() {}
	FEM1DConstraintLocalization(const FEM1DConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const FEM1DConstraintLocalization& other) const
	{
		return ( beamID == other.beamID && abs == other.abs);
	}
	FEM1DConstraintLocalization& operator=(const FEM1DConstraintLocalization& other)
	{
		beamID = other.beamID;
		abs = other.abs;
		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{ return FEM1D_LOCALIZATION; }
};

struct FEM2DConstraintLocalization : public FEMConstraintLocalization
{
	int triID;
	double triBarycentricCoord[3];

	FEM2DConstraintLocalization() {}
	FEM2DConstraintLocalization(const FEM2DConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const FEM2DConstraintLocalization& other) const
	{
		return ( triID == other.triID && 
			triBarycentricCoord[0] == other.triBarycentricCoord[0] && triBarycentricCoord[1] == other.triBarycentricCoord[1] && triBarycentricCoord[2] == other.triBarycentricCoord[2]);
	}
	FEM2DConstraintLocalization& operator=(const FEM2DConstraintLocalization& other)
	{
		triID = other.triID;
		for (int i=0; i<3; ++i)
			triBarycentricCoord[i] = other.triBarycentricCoord[i];
		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{ return FEM2D_LOCALIZATION; }
};

struct FEM3DConstraintLocalization : public FEMConstraintLocalization
{
	int triID;
	double triBarycentricCoord[3];
	int tetID;
	double tetBarycentricCoord[4];
	FEM3DConstraintLocalization() {}
	FEM3DConstraintLocalization(const FEM3DConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const FEM3DConstraintLocalization& other) const
	{
		return ( triID == other.triID && 
			tetID == other.tetID && 
			triBarycentricCoord[0] == other.triBarycentricCoord[0] && triBarycentricCoord[1] == other.triBarycentricCoord[1] && triBarycentricCoord[2] == other.triBarycentricCoord[2] &&
			tetBarycentricCoord[0] == other.tetBarycentricCoord[0] && tetBarycentricCoord[1] == other.tetBarycentricCoord[1] && tetBarycentricCoord[2] == other.tetBarycentricCoord[2] && tetBarycentricCoord[3] == other.tetBarycentricCoord[3] );
	}
	FEM3DConstraintLocalization& operator=(const FEM3DConstraintLocalization& other)
	{
		triID = other.triID;
		tetID = other.tetID;
		for (int i=0; i<3; ++i)
			triBarycentricCoord[i] = other.triBarycentricCoord[i];
		for (int i=0; i<4; ++i)
			tetBarycentricCoord[i] = other.tetBarycentricCoord[i];
		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{ return FEM3D_LOCALIZATION; }
};

struct NeedleConstraintLocalization : public ConstraintLocalization
{
	//double globalPosition[3];
	int ptID1,ptID2;	// Centerline segment given by 2 node ID (usually consecutive)
	double abs;		// local abs on this segment [0..1]
	virtual ConstraintLocalizationType getType() const
	{ return NEEDLE_LOCALIZATION; }
};

struct FixedConstraintLocalization : public ConstraintLocalization
{
	double Position[3];
	double Twist;
	int ElementID; //! Useful for suturable to locate in which OctreeNode the constraint is located
	FixedConstraintLocalization() {}
	FixedConstraintLocalization(const FixedConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const FixedConstraintLocalization& other) const
	{
		return ( Twist == other.Twist && 
			Position[0] == other.Position[0] && Position[1] == other.Position[1] && Position[2] == other.Position[2] &&
			ElementID == other.ElementID);
	}
	FixedConstraintLocalization& operator=(const FixedConstraintLocalization& other)
	{
		ElementID = other.ElementID;
		Twist = other.Twist;
		for (int i=0; i<3; ++i)
			Position[i] = other.Position[i];
		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{ return FIXED_LOCALIZATION; }
};

struct MeshConstraintLocalization : public ConstraintLocalization
{
	int triID;
	double triBarycentricCoord[3];
	MeshConstraintLocalization() {}
	MeshConstraintLocalization(const MeshConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const MeshConstraintLocalization& other) const
	{
		return ( triID == other.triID && 
			triBarycentricCoord[0] == other.triBarycentricCoord[0] && triBarycentricCoord[1] == other.triBarycentricCoord[1] && triBarycentricCoord[2] == other.triBarycentricCoord[2]);
	}
	MeshConstraintLocalization& operator=(const MeshConstraintLocalization& other)
	{
		triID = other.triID;
		for (int i=0; i<3; ++i)
			triBarycentricCoord[i] = other.triBarycentricCoord[i];
		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{ return MESH_LOCALIZATION; }
};

struct ToolConstraintLocalization: public ConstraintLocalization
{
	MeshConstraintLocalization meshConstraintLocalization;
	FixedConstraintLocalization fixedConstraintLocalization; // NOTE: This is used to store a LOCAL position (in the TOOL frame) !!!!

	bool isUsingMeshLocalization;

	ToolConstraintLocalization() {}
	ToolConstraintLocalization(const ToolConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const ToolConstraintLocalization& other) const
	{
		if(isUsingMeshLocalization && other.isUsingMeshLocalization)
			meshConstraintLocalization == other.meshConstraintLocalization;
		else if(!isUsingMeshLocalization && !other.isUsingMeshLocalization)
			return fixedConstraintLocalization == other.fixedConstraintLocalization;

		// At this point, we have different type of constraint, so they are different !
		return false;
	}
	ToolConstraintLocalization& operator=(const ToolConstraintLocalization& other)
	{
		// We need to copy everything (we might be using a FixedLoc with isUsingMeshLoc=false...
		// but the meshLoc could still be filled up with useful info that we need to keep for 2 points grasp)
		meshConstraintLocalization	= other.meshConstraintLocalization;
		fixedConstraintLocalization	= other.fixedConstraintLocalization;
		isUsingMeshLocalization		= other.isUsingMeshLocalization;

		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{
		if(isUsingMeshLocalization)
			return TOOL_MESH_LOCALIZATION;
		else
			return TOOL_LOCAL_SPACE_LOCALIZATION;
	}
};

struct RigidBodyConstraintLocalization: public ConstraintLocalization
{
	MeshConstraintLocalization meshConstraintLocalization;
	FixedConstraintLocalization fixedConstraintLocalization; // NOTE: This is used to store a LOCAL position (in the TOOL frame) !!!!

	bool isUsingMeshLocalization;

	RigidBodyConstraintLocalization() {}
	RigidBodyConstraintLocalization(const RigidBodyConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const RigidBodyConstraintLocalization& other) const
	{
		if(isUsingMeshLocalization && other.isUsingMeshLocalization)
			return (meshConstraintLocalization == other.meshConstraintLocalization);
		else if(!isUsingMeshLocalization && !other.isUsingMeshLocalization)
			return (fixedConstraintLocalization == other.fixedConstraintLocalization);

		// At this point, we have different type of constraint, so they are different !
		return false;
	}
	RigidBodyConstraintLocalization& operator=(const RigidBodyConstraintLocalization& other)
	{
		// We need to copy everything (we might be using a FixedLoc with isUsingMeshLoc=false...
		// but the meshLoc could still be filled up with useful info that we need to keep for 2 points grasp)
		meshConstraintLocalization	= other.meshConstraintLocalization;
		fixedConstraintLocalization	= other.fixedConstraintLocalization;
		isUsingMeshLocalization		= other.isUsingMeshLocalization;

		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{
		if(isUsingMeshLocalization)
			return RIGID_BODY_MESH_LOCALIZATION;
		else
			return RIGID_BODY_LOCAL_SPACE_LOCALIZATION;
	}
};

//#######################################
// Contact data structures

class ConstraintData
{
public:
	ConstraintData() {};
	virtual ~ConstraintData() {};
private:
	// prohibit copying
	ConstraintData(const ConstraintData&);
	ConstraintData& operator= (const ConstraintData&);
};

struct Frictionless_Contact : public ConstraintData
{
	double n[4]; // Contact plane: n[0].Px + n[1].Py + n[2].Pz + n[3] = 0
};

struct Frictional_Contact : public ConstraintData
{
	double n[4];  // Contact plane: n[0].Px  + n[1].Py  + n[2].Pz  + n[3]  = 0
	double t1[4]; // Contact plane: t1[0].Px + t1[1].Py + t1[2].Pz + t1[3] = 0
	double t2[4]; // Contact plane: t2[0].Px + t2[1].Py + t2[2].Pz + t2[3] = 0
	double frictionCoef;
};


//#######################################
// Suturing data structures
struct Frictionless_Suturing : public ConstraintData
{
	double n1[4],n2[4]; // Intersection of 2 planes = direction of sliding
};

struct Frictional_Suturing : public ConstraintData
{
	double n1[4],n2[4]; // Intersection of 2 planes = direction of sliding
	double t[4];        // Direction of sliding along which friction occurs
	double frictionCoef;
};

//#######################################
// Directional constraint data structures
struct Bilateral_Direction : public ConstraintData
{
	ConstraintLocalization* secondConstraintLocalizations[2];

	double t[3];  // current direction for debug visualization.
	double n1[3];
	double n2[3];
};

class EntityInteracting;

struct EntityData
{
	int type;   // the value is really an EntityType enumeration
	int index;  // which entity of this type
	int grpIndex;  // which group in the entity of this type (finer grained control)

	// convenience constructor
	EntityData() : type(INVALID_ENTITY), index(-1), grpIndex(-1) {};
	bool operator ==(const EntityData &e) const { return (type==e.type && index==e.index && grpIndex==e.grpIndex); };
	bool operator !=(const EntityData &e) const { return (type!=e.type || index!=e.index || grpIndex!=e.grpIndex); };
};

class Constraint
{
public:
	Constraint() : type(CONSTRAINT_INVALID), oldType(type), m_cachedLambda(NULL), m_firstLambdaGlobalIndex(-1), m_outputForceScale(-1.0)
	{
		entity[0] = 0;
		entity[1] = 0;
		entity_modelConstraintLocalization[0] = 0;
		entity_modelConstraintLocalization[1] = 0;
		constraintDataStructure = 0;
		localTimeOfImpact = 0.0;
		globalTimeOfImpact = 0.0;
		violation = 0.0;
	}

	~Constraint()
	{
		releaseData();
		zeroOutData();
	}

public:
	// == copy construction and assignment ==

	// Calling this method make the Constraint parameter INVALID (SOME POINTERS ARE NULLED).
	// You SHOULD NOT use the old data structure anymore.
	Constraint(const Constraint& rhs)
	{
		// moveData no longer releases our own old data first, so there's no reason to set things to null before we call it.

		// the const_cast is necessary because the STL (vector, etc) requires a copy constructor to have a const argument
		moveData(const_cast<Constraint &>(rhs));
	}

	// Calling this method make the Constraint parameter INVALID (SOME POINTERS ARE NULLED).
	// You SHOULD NOT use the old data structure anymore.
	Constraint& operator =(const Constraint& rhs)
	{
		// Before we grab the pointers from the other object, we release our own (if any).
		releaseData();

		// the const_cast is necessary because the STL (vector, etc) requires a copy constructor to have a const argument
		moveData(const_cast<Constraint &>(rhs)); // const_cast necessary because of the way stl is dealing with the vector info !
		return *this;
	}

	bool operator ==(const Constraint &c) const;
	bool operator < (const Constraint &c) const
	{ return violation < c.violation; }

	bool isEquilvalentTo(const Constraint &c, const double& maxLocationTolerance, const double& maxPlaneAngle, int localizationToCheck=-1) const;

	static std::string getTypeByName(int type);
private:

	static std::deque<SutureConstraintLocalization*> unusedSutureConstraintLocalizations;
	static std::deque<FEM3DConstraintLocalization*> unusedFEM3DConstraintLocalizations;
	static std::deque<FEM2DConstraintLocalization*> unusedFEM2DConstraintLocalizations;
	static std::deque<FEM1DConstraintLocalization*> unusedFEM1DConstraintLocalizations;
	static std::deque<NeedleConstraintLocalization*> unusedNeedleConstraintLocalizations;
	static std::deque<FixedConstraintLocalization*> unusedFixedConstraintLocalizations;
	static std::deque<MeshConstraintLocalization*> unusedMeshConstraintLocalizations;
	static std::deque<ToolConstraintLocalization*> unusedToolConstraintLocalizations;
	static std::deque<RigidBodyConstraintLocalization*> unusedRigidBodyConstraintLocalizations;

	static std::stack<Frictionless_Contact*> unusedFrictionlessContactData;
	static std::stack<Frictional_Contact*> unusedFrictionalContactData;
	static std::stack<Frictionless_Suturing*> unusedFrictionlessSuturingData;
	static std::stack<Frictional_Suturing*> unusedFrictionalSuturingData;
	static std::stack<Bilateral_Direction*> unusedBilateralDirectionData;

	void moveData(Constraint& rhs);
	void releaseData();
	void zeroOutData();

public:
	// Method to manage the diverse Constraint from outside (retrieve/release constraints)
	static SutureConstraintLocalization* retrieveSutureConstraintLocalization(void);
	static FEM3DConstraintLocalization* retrieveFEM3DConstraintLocalization(void);
	static FEM2DConstraintLocalization* retrieveFEM2DConstraintLocalization(void);
	static FEM1DConstraintLocalization* retrieveFEM1DConstraintLocalization(void);
	static NeedleConstraintLocalization* retrieveNeedleConstraintLocalization(void);
	static FixedConstraintLocalization* retrieveFixedConstraintLocalization(void);
	static MeshConstraintLocalization* retrieveMeshConstraintLocalization(void);
	static ToolConstraintLocalization* retrieveToolConstraintLocalization(void);
	static RigidBodyConstraintLocalization* retrieveRigidBodyConstraintLocalization(void);

	static void releaseSutureConstraintLocalization(SutureConstraintLocalization*);
	static void releaseFEM3DConstraintLocalization(FEM3DConstraintLocalization*);
	static void releaseFEM2DConstraintLocalization(FEM2DConstraintLocalization*);
	static void releaseFEM1DConstraintLocalization(FEM1DConstraintLocalization*);
	static void releaseNeedleConstraintLocalization(NeedleConstraintLocalization*);
	static void releaseFixedConstraintLocalization(FixedConstraintLocalization*);
	static void releaseMeshConstraintLocalization(MeshConstraintLocalization*);
	static void releaseToolConstraintLocalization(ToolConstraintLocalization*);
	static void releaseRigidBodyConstraintLocalization(RigidBodyConstraintLocalization*);

	static void releaseConstraintLocalization(ConstraintLocalization*);

	static bool isConstraintLocalizationUnused(ConstraintLocalization*);
public:
	void setSutureSide(int slot, int objIndex, int cylID, double localAbs);
	void setFem3DTetSide(int slot, int objIndex, int tetID, double barycentricW0, double barycentricW1, double barycentricW2, double barycentricW3);
	void setFem3DTriSide(int slot, int objIndex, int triID, double barycentricW0, double barycentricW1, double barycentricW2);
	void setFem2DSide(int slot, int objIndex, int triID, double barycentricW0, double barycentricW1, double barycentricW2);
	void setNeedleSide(int slot, int objIndex, int ptID1, int ptID2, double localPositionOnSegment);
	void setFixedSide(int slot, int objIndex, double globalPosX, double globalPosY, double globalPosZ);
	void setFixedSide(int slot, int objIndex, double globalPosX, double globalPosY, double globalPosZ, double globalTwist);
	void setToolSide(int slot, int objIndex, int triID, double barycentricW0, double barycentricW1, double barycentricW2);
	void setToolSide(int slot, int objIndex, double localPosX, double localPosY, double localPosZ);
	void setRigidBodySide(int slot, int objIndex, int triID, double barycentricW0, double barycentricW1, double barycentricW2);
	void setRigidBodySide(int slot, int objIndex, double localPosX, double localPosY, double localPosZ, double localTwist=0.0);
	void setFrictionlessContact(double nx, double ny, double nz, double nOff);
	void setFrictionlessContact(double nx, double ny, double nz, double p0x, double p0y, double p0z);
	void setTwistConstraint(void);
	void setBilateralConstraint(void);
	void setBilateralAngularVelocityConstraint();
	void setFrictionalContact(double nx, double ny, double nz,
		double t1x, double t1y, double t1z,
		double t2x, double t2y, double t2z,
		double p0x, double p0y, double p0z,
		double frictionCoef);
	void setFrictionalContact(const double *n, const double *t1, const double *t2, const double frictionCoef);
	void setFrictionlessSliding(double n1x, double n1y, double n1z,
		double n2x, double n2y, double n2z,
		double p0x, double p0y, double p0z);
	void setFrictionlessSliding(const double *n1, const double *n2);
	void setFrictionalSliding(double n1x, double n1y, double n1z,
		double n2x, double n2y, double n2z,
		double tx, double ty, double tz,
		double p0x, double p0y, double p0z,
		double frictionCoef);
	void setFrictionalSliding(const double *n1, const double *n2, const double *t, const double frictionCoef);
	void setAxialRotation();
	void setDirectionalConstraintSutureSide(int slot, int objIndex, int cylID, double localAbs);
	void setDirectionalConstraintFemTetSide(int slot, int objIndex, int tetID, double barycentricW0, double barycentricW1, double barycentricW2, double barycentricW3);
	void setDirectionalConstraintFemTriSide(int slot, int objIndex, int triID, double barycentricW0, double barycentricW1, double barycentricW2);
	void setDirectionalConstraintNeedleSide(int slot, int objIndex, int ptID1, int ptID2, double localPositionOnSegment);
	void setDirectionalConstraintFixedSide(int slot, int objIndex, double globalPosX, double globalPosY, double globalPosZ);
	void setDirectionalConstraintFixedToolSide(int slot, int objIndex, int triID, double barycentricW0, double barycentricW1, double barycentricW2);
	void setDirectionalConstraintFixedToolSide(int slot, int objIndex, double localPositionX, double localPositionY, double localPositionZ);
	void setTimeOfImpact(double localTOI, double globalTOI);
	void setOutputForceScale(double scale);
	void invalidate();
	void deActivate();
	void activate();
	bool isActive() const;
	bool getConstraintForcePosition(int entityNum, double force[3], double point[3], double& toi) const;
	bool getConstraintForceRecipe(int entityNum, LambdaWeightedForceVector& recipe) const;
	bool getViolationAdjustmentRecipe(int entityNum, ElementAdjustmentFromPoseProjection& recipe) const;
	void getConstraintPosition(int entityNum, double point[3]) const;
	void getFirstConstraintPosition(double point[3]) const 
	{ getConstraintPosition(0, point); }
	void getSecondConstraintPosition(double point[3]) const
	{ getConstraintPosition(1, point); }
	bool getConstraintFriction(int entityNum, double friction[3]) const;
	const double getOutputForceScale() const;

	int getDof() const
	{
		switch (type)
		{

		case CONSTRAINT_INVALID:
			return 0;
		case CONSTRAINT_BILATERAL_TWIST:
			return 1;
		case CONSTRAINT_BILATERAL_3D:
			return 3;
		case CONSTRAINT_BILATERAL_ANGULAR_VELOCITY:
			return 3;
		case CONSTRAINT_FRICTIONLESS_CONTACT:
			return 1;
		case CONSTRAINT_FRICTIONAL_CONTACT:
			return 3;
		case CONSTRAINT_FRICTIONLESS_SLIDING:
			return 2;
		case CONSTRAINT_FRICTIONAL_SLIDING:
			return 3;
		case CONSTRAINT_BILATERAL_DIRECTION:
			return 2;
		case CONSTRAINT_BILATERAL_AXIAL_ROTATION:
			return 1;
		default:
			SQ_FAILURE("unknown number of DOFs for this constraint type!?!");
			return -1;
		}
	}

	ConstraintType getType() const    { return type; }
	ConstraintType getOldType() const { return oldType; }

	void changeType(ConstraintType newType)
	{
		SQ_ASSERT(type != CONSTRAINT_INVALID, "cannot change the type on a deactivated/invalid constraint");
		SQ_ASSERT(oldType == CONSTRAINT_INVALID, "type and oldType cannot both be valid at the same time");
		SQ_ASSERT(newType != CONSTRAINT_INVALID, "cannot change the type to invalid");
		SQ_ASSERT(newType >= NUM_CONSTRAINT_TYPES, "cannot change the type to invalid");
		// TODO: check whether changing the type makes sense?
		//       mostly it's sane only for certain downgrades (e.g. frictional -> frictionless) where you don't need to recompute everything.
		type = newType;
	}

protected:
	ConstraintType type;
	ConstraintType oldType;
public:  // ...at least for now; we should probably change it
	double globalTimeOfImpact;  // relative to the entire time step (range: 0 to 1)
	double localTimeOfImpact;   // relative to the collision pass interval (range: 0 to 1)

	EntityData              entityData[2];             // information on the 1st and 2nd entities
	EntityInteracting*      entity[2];                 // 1st and 2nd entities interacting through this constraint

	ConstraintLocalization* entity_modelConstraintLocalization[2];  // 1st and 2nd entities constraint localization
	ConstraintData*         constraintDataStructure;                // Constraint data structure

	// Pointer to the last computed value of lambda in the MasterSolver - this is for debug and it is up to the user to have it properly synced
	double* m_cachedLambda;
	// Index in MasterSolver's global lambda array of the first lambda for this constraint.  If there is more than one lambda, the rest follow consecutively.
	int     m_firstLambdaGlobalIndex;

	double violation;

	// Scale to apply to forces output by this constraint in haptics
	double m_outputForceScale;
	
};

std::ostream& operator<<(std::ostream&, const Constraint&);

#endif