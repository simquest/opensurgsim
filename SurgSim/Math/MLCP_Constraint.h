#ifndef MLCPCONSTRAINT_H
#define MLCPCONSTRAINT_H

#define NB_MAX_MLCP_ATOMIC_ENTRY 300
typedef enum
{
    MLCP_INVALID_CONSTRAINT = -1,                    // Invalid value -- not used for any constraint type
    MLCP_BILATERAL_1D_CONSTRAINT = 0,                // Fixing 1 DOF
    MLCP_BILATERAL_2D_CONSTRAINT,                    // Fixing 2 DOF
    MLCP_BILATERAL_3D_CONSTRAINT,                    // Fixing 3 DOF (could be a fixed point in 3D space for example)
    //MLCP_BILATERAL_4D_CONSTRAINT,                  // Fixing 4 DOF (could be a fixed point with twist included for the MechanicalSpline for example)
    MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT,      // Frictionless contact (could be in 2D or 3D => only 1 atomic unilateral constraint)
    MLCP_UNILATERAL_3D_FRICTIONAL_CONSTRAINT,        // Frictional contact in 3D, using Coulomb friction => 1 entry in the friction coefficient array !
    MLCP_BILATERAL_FRICTIONLESS_SLIDING_CONSTRAINT,  // Frictionless suturing in 3D => 2 directional constraint = 2 bilateral constraints !
    MLCP_BILATERAL_FRICTIONAL_SLIDING_CONSTRAINT,    // Frictional suturing in 3D => 2 directional constraint + 1 frictional tangent constraint linked by the Coulomb's law
    MLCP_NUM_CONSTRAINT_TYPES
} MLCP_Constraint;

#endif // MLCPCONSTRAINT_H
