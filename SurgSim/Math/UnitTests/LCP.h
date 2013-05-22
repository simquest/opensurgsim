#ifndef __LCP__
#define __LCP__

#define IN
#define OUT
#define INOUT

//#include <iostream>
#include <vector>
using namespace std;

//! class LCP.
/*!
This class defines a LCP problem defined by:
A x + b = c
0 <= x orthogonal c => 0

This class is templatized on the matrix type and vector type for more flexibility usage.

Ax+b (=c) measures de violation of the constraint (inequality)
Either we have Ax+b>0 AND x=0 (On the good side of the inequality => no violation therefore no force (x=0))
Or     we have Ax+b=0 AND x>0 (On the wrong side of the inequality => violation therefore a force (x>0) bring back the constraint to verified Ax+b=0)
*/
template <class Matrix, class Vector> class LCP
{
public:
	//! Empty constructor for easy usage.
	LCP() {};
	//! Destructor empty....implicit call to the member's destructor.
	virtual ~LCP() {};

	//! Resolution of a given LCP (empty)
	/*!
	This method will be implemented by inherited classes.
	Each of them will implement a different algorithm.
	USING 1 UNIQUE FRICTION COEFFICIENT
	*/
	virtual bool solve(IN int n, IN int m, IN Matrix& A, IN int nbColumnInA,  IN Vector& b, INOUT Vector& initialGuess_and_solution, OUT Vector& tmp)
	{
		cerr << "LCP::solve is empty" << endl;
		return false;
	};
	//! Resolution of a given LCP (empty)
	/*!
	This method will be implemented by inherited classes.
	Each of them will implement a different algorithm.
	USING DIFFERENT FRICTION COEFFICIENT FOR EACH CONTACT
	*/
	virtual bool solve(IN int n, IN int m, IN Matrix& A, IN int nbColumnInA,  IN Vector& frictionCoefs, IN Vector& b, INOUT Vector& initialGuess_and_solution, OUT Vector& tmp)
	{
		cerr << "LCP::solve is empty" << endl;
		return false;
	};

	//! Evaluate the LCP inequality
	/*!
	Evaluate Ax+x and stores it in c
	*/
	bool isLCPsolved(IN int n, IN int m, IN Matrix& A, IN Vector& b, IN Vector& x, OUT Vector& c);
};


#include "LCP.hpp"
//#include "LCP_Lemke.h"
//#include "LCP_GaussSeidel.h"
//#include "LCP_StatusMethod_with_DirectResolution.h"
//#include "LCP_2DContactFriction_GaussSeidel.h"
//#include "LCP_3DContactFriction_GaussSeidel.h" // Based on Christian Duriez's PhD 04 (using a non linear phi function and its derivative).
//#include "LCP_3DContactFriction_GaussSeidel_algo2.h" // Based on Christian Duriez's TVCG 05 paper.
//#include "LCP_3DContactFriction_GaussSeidel_Christian.h" // Based on Christian Duriez's TVCG 05 paper + recent Christian's work
//#include "LCP_3DStatusMethodFriction_with_DirectResolution.h"
//#include "LCP_3DStatusMethodNOFriction_with_DirectResolution.h"
//#include "LCP_3DSutureFriction_GaussSeidel.h"




//! class LCP_concave
/*!
This class defines a LCP problem defined by:
A x + b = c
0 <= x orthogonal c => 0

This class is templatized on the matrix type and vector type for more flexibility usage.

Ax+b (=c) measures de violation of the constraint (inequality)
Either we have Ax+b>0 AND x=0 (On the good side of the inequality => no violation therefore no force (x=0))
Or     we have Ax+b=0 AND x>0 (On the wrong side of the inequality => violation therefore a force (x>0) bring back the constraint to verified Ax+b=0)

THE LINEAR CONSTRAINT ARE LIMITED IN SPACE (triangles) !!
For that, we need 2 more info:
  1) the triangle for each contact and
  2) the matrix to compute the displacement from the contact forces (formely CLt, while the LCP matrix is LCLt)
*/
template <class Matrix, class Vector> class LCP_concave
{
public:
	typedef struct
	{
		// Contact model info
		int nbContacts;

		double p[500][3];                         // Position of the mechanical constrained node
		double modelInternalParameter[500][3];         // coef barycentrics for 1D,2D,3D linear model, or spline parametric abscissae for spline model (1D,2D or 3D)

		double normal[500][4];                    // triangle normal (plane equation)
		double t1[500][3],t2[500][3],t3[500][3];  // Triangle vertices corresponding to this contact (if any !)
		double limitEdge1[500][4];                // Triangle edge 1 plane equation (point t1 t2)
		double limitEdge2[500][4];                // Triangle edge 2 plane equation (point t1 t3)
		double limitEdge3[500][4];                // Triangle edge 3 plane equation (point t2 t3)

		bool concaveLocality[500];                // If FALSE=>triangle limits won't be check in the loop, otherwise a special care will be taken !
		bool active[500];                         // Internal usage, this will vary during the Gauss Seidel loop as necessary (concave case (de)activation)
	} ContactInfo;


	//! Empty constructor for easy usage.
	LCP_concave() {};
	//! Destructor empty....implicit call to the member's destructor.
	virtual ~LCP_concave() {};

	//! Resolution of a given LCP (empty)
	/*!
	This method will be implemented by inherited classes.
	Each of them will implement a different algorithm.
	*/
	virtual bool solve(IN int n, IN Matrix& LCLt, IN int nbColumnInA, IN Vector& b, INOUT Vector& initialGuess_and_solution, OUT Vector& tmp,
	                   IN ContactInfo* info,
	                   IN Matrix& CLt,
	                   IN double nbDOF,
	                   IN double* DOF,
	                   IN double* DOFoffset)
	{
		cerr << "LCP::solve is empty" << endl;
		return false;
	};

	//! Evaluate the LCP inequality
	/*!
	Evaluate Ax+x and stores it in c
	*/
	bool isLCPsolved(IN int n, IN int m, IN Matrix& A, IN Vector& b, IN Vector& x, OUT Vector& c);
};

//#include "LCP_3DContactFriction_GaussSeidel_algo2_concaveProof.h" // Based on Christian Duriez's TVCG 05 paper + checking triangle limits in the Gauss Seidel loop



#include "MLCP_Constraint.h"
//! class MLCP.
/*!
This class defines a Mixed LCP problem defined by:
A x + b = c
0 <= x orthogonal c => 0 for a subset

This class is templatized on the matrix type and vector type for more flexibility usage.

Ax+b (=c) measures de violation of the constraint (inequality)
Either we have Ax+b>0 AND x=0 (On the good side of the inequality => no violation therefore no force (x=0))
Or     we have Ax+b=0 AND x>0 (On the wrong side of the inequality => violation therefore a force (x>0) bring back the constraint to verified Ax+b=0)

We have a mixed of bilateral constraints and unilateral constraints
*/

template <class Matrix, class Vector> class MLCP
{
public:
	//! Empty constructor for easy usage.
	MLCP() {};
	//! Destructor empty....implicit call to the member's destructor.
	virtual ~MLCP() {};

	//! Resolution of a given LCP (empty)
	/*!
		@param n an integer argument, the number of atomic constraint in the MLCP
		@param A a Matrix, the MLCP matrix
		@param nbColumnInA an integer argument, the actual number of column (alocated in memory) of the matrix A
		@param b a Vector, the MLCP right hand side (the initial constraints violation)
		@param initialGuess_and_solution a Vector, contains the initial guess and the final solution on exit !
		@param frictionCoefs a Vector, contains the list of friction coefficient for each constraint (or 0 if not needed)
		@param constraintsList a vector, contains the list of constraint type, to determine how much atomic constraint each takes and how to handle it !

	This method will be implemented by inherited classes.
	Each of them will implement a different algorithm.
	USING DIFFERENT FRICTION COEFFICIENT FOR EACH CONTACT
	*/
	virtual bool solve(IN int n, IN Matrix& A, IN int nbColumnInA, IN Vector& b, INOUT Vector& initialGuess_and_solution,
	                   IN Vector& frictionCoefs, IN vector<MLCP_Constraint>& constraintsType, OUT int* MLCP_nbIterations=0)
	{
		cerr << "MLCP::solve is empty" << endl;
		return false;
	};

	//! Evaluate the MLCP inequality
	/*!
	Evaluate Ax+x and stores it in c
	*/
	bool isMLCPsolved(IN int n, IN Matrix& A, IN Vector& b, IN Vector& x, OUT Vector& c);
};


#include "MLCP_GaussSeidel_Christian.h"
//#include "SuccessiveEquilibriumSolver.h"

#endif
