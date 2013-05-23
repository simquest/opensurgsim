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

/** @file
* Tests for the Gauss-Seidel implementation of the MLCP solver.
*/

#include <math.h>
#include <SurgSim/Math/MLCP_GaussSeidel_Christian.h>
#include "MlcpTestProblems.h"
#include "gtest/gtest.h"
#include <boost/chrono.hpp>

TEST(MlcpGaussSeidelSolverTests, CanConstruct)
{
	//ASSERT_NO_THROW({
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd, Eigen::VectorXd> mlcpSolver(1.0, 1.0, 100);
}

TEST(MlcpGaussSeidelSolverTests, CompareResult1)
{
	const MlcpTestData* problem = getTestProblem1();

	// NB: need to make the solver calls const-correct.
	Eigen::MatrixXd HCHt = problem->HCHt;
	Eigen::VectorXd E = problem->E;
	Eigen::VectorXd mu = problem->mu;
	std::vector<MLCP_Constraint> constraintTypes = problem->constraintTypes;

	const int size = problem->getSize();
	Eigen::VectorXd lambda(size);

	//################################
	// Gauss-Seidel solver
	const double solverPrecision = 1e-4;
	const double _contactTolerance = 2e-5;
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd,Eigen::VectorXd> mlcpSolver(solverPrecision, _contactTolerance, 30);

	printf("  ### Gauss Seidel solver:\n");
	int nbIteration;
	bool converged;
	bool Signorini;
	lambda.setZero();

	bool res = mlcpSolver.solve(size, HCHt, size, E, lambda, mu, constraintTypes, 1.0,
	                            &nbIteration, &converged, &Signorini);

	printf("\tsolver did %d iterations convergence=%d Signorini=%d\n",nbIteration,converged,Signorini);

	ASSERT_EQ(size, lambda.rows());
	ASSERT_EQ(size, problem->expectedLambda.rows());
	for (int i = 0;  i < size;  ++i)
	{
		EXPECT_NEAR(problem->expectedLambda[i], lambda[i], 1e-9);
	}

	double convergenceCriteria=0.0;
	bool validSignorini=false;
	int nbContactToSkip=0;
//	int currentAtomicIndex = calculateConvergenceCriteria(lambda, nbContactToSkip, convergenceCriteria, validSignorini,1.0);
//	printf("\tStatus method final [convergence criteria=%g, Signorini=%d]\n",convergenceCriteria,validSignorini?1:0);
	printf("############\n");
}

TEST(MlcpGaussSeidelSolverTests, MeasureExecutionTime1)
{
	const MlcpTestData* problem = getTestProblem1();

	// NB: need to make the solver calls const-correct.
	Eigen::MatrixXd HCHt  = problem->HCHt;
	Eigen::VectorXd E = problem->E;
	Eigen::VectorXd mu = problem->mu;
	std::vector<MLCP_Constraint> constraintTypes = problem->constraintTypes;

	const int size = problem->getSize();
	Eigen::VectorXd lambda(size);

	const int repetitions = 100000;

	typedef boost::chrono::high_resolution_clock clock;
	clock::time_point time0 = clock::now();

	for (int i = repetitions;  i > 0;  --i)
	{
		HCHt = problem->HCHt;
		E = problem->E;
		mu = problem->mu;
		constraintTypes = problem->constraintTypes;
	}

	const double solverPrecision = 1e-4;
	const double _contactTolerance = 2e-5;
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd,Eigen::VectorXd> mlcpSolver(solverPrecision, _contactTolerance, 30);

	clock::time_point time1 = clock::now();

	for (int i = repetitions;  i > 0;  --i)
	{
		HCHt = problem->HCHt;
		E = problem->E;
		mu = problem->mu;
		constraintTypes = problem->constraintTypes;

		lambda.setZero();

		int nbIteration;
		bool converged;
		bool Signorini;

		bool res = mlcpSolver.solve(size, HCHt, size, E, lambda, mu, constraintTypes, 1.0,
		                            &nbIteration, &converged, &Signorini);
	}

	clock::time_point time2 = clock::now();

	boost::chrono::nanoseconds elapsedSolveTime =
	    boost::chrono::duration_cast<boost::chrono::nanoseconds>(time2 - time1);
	//std::cout << "Elapsed:               " << (elapsedSolveTime.count() / 1000000.) << " ms" << std::endl;
	boost::chrono::nanoseconds elapsedBaseline =
	    boost::chrono::duration_cast<boost::chrono::nanoseconds>(time1 - time0);
	//std::cout << "Baseline:              " << (elapsedBaseline.count() / 1000000.) << " ms" << std::endl;
	double solveTimeUsec = (elapsedSolveTime - elapsedBaseline).count() / 1000. / repetitions;
	std::cout << "Average solution time: " << solveTimeUsec << " microseconds" << std::endl;

	// When refactoring the MLCP code, it can be very useful to compare the execution time before and after making
	// changes.  But you can't usefully compare execution times on different machines, or with different build
	// settings, so you'll need to manually uncomment the following line and adjust the time accordingly.
//	EXPECT_LE(solveTimeUsec, 14.0);
}
