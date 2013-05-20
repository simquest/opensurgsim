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
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include <boost/chrono.hpp>

#include <SurgSim/Math/Valid.h>
#include <SurgSim/Math/MLCP_GaussSeidel_Christian.h>
#include "MlcpTestData.h"
#include "ReadText.h"

using SurgSim::Math::isValid;


static std::shared_ptr<MlcpTestData> loadTestProblem(const std::string& fileName)
{
	std::shared_ptr<MlcpTestData> problem = std::make_shared<MlcpTestData>();
	if (! readMlcpTestDataAsText("MlcpTestData/" + fileName, problem.get()))
	{
		problem.reset();
	}
	return problem;
}



TEST(MlcpGaussSeidelSolverTests, CanConstruct)
{
	//ASSERT_NO_THROW({
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd, Eigen::VectorXd> mlcpSolver(1.0, 1.0, 100);
}

TEST(MlcpGaussSeidelSolverTests, CompareResult1)
{
	const std::shared_ptr<MlcpTestData> problem = loadTestProblem("mlcpOriginalTest.txt");
	ASSERT_TRUE(problem) << "Failed to load " << "mlcpOriginalTest.txt";

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
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd, Eigen::VectorXd> mlcpSolver(solverPrecision, _contactTolerance, 30);

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
	EXPECT_TRUE(isValid(lambda));
	for (int i = 0;  i < size;  ++i)
	{
		EXPECT_NEAR(problem->expectedLambda[i], lambda[i], 1e-9);
	}

	double convergenceCriteria=0.0;
	bool validSignorini=false;
	int nbContactToSkip=0;
//	int currentAtomicIndex = calculateConvergenceCriteria(lambda, nbContactToSkip, convergenceCriteria,
//		validSignorini, 1.0);
//	printf("\tStatus method final [convergence criteria=%g, Signorini=%d]\n",convergenceCriteria,validSignorini?1:0);
	printf("############\n");
}

static void solveRepeatedly(const MlcpTestData& problem,
							/*XXX const */ MLCP_GaussSeidel_Christian<Eigen::MatrixXd,Eigen::VectorXd>* mlcpSolver,
							const int repetitions)
{
	// NB: need to make the solver calls const-correct.
	Eigen::MatrixXd HCHt;
	Eigen::VectorXd E;
	Eigen::VectorXd mu;
	std::vector<MLCP_Constraint> constraintTypes;

	const int size = problem.getSize();
	Eigen::VectorXd lambda(size);

	for (int i = repetitions;  i > 0;  --i)
	{
		HCHt = problem.HCHt;
		E = problem.E;
		mu = problem.mu;
		constraintTypes = problem.constraintTypes;

		if (mlcpSolver)
		{
			lambda.setZero();

			int nbIteration;
			bool converged;
			bool Signorini;

			bool res = mlcpSolver->solve(size, HCHt, size, E, lambda, mu, constraintTypes, 1.0,
				&nbIteration, &converged, &Signorini);
		}
	}
}

TEST(MlcpGaussSeidelSolverTests, MeasureExecutionTime1)
{
	const std::shared_ptr<MlcpTestData> problem = loadTestProblem("mlcpOriginalTest.txt");
	ASSERT_TRUE(problem);

	const double solverPrecision = 1e-4;
	const double _contactTolerance = 2e-5;
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd,Eigen::VectorXd> mlcpSolver(solverPrecision, _contactTolerance, 30);

	typedef boost::chrono::high_resolution_clock clock;
	clock::time_point calibrationStart = clock::now();

	const int calibrationRepetitions = 1000;
	solveRepeatedly(*problem, &mlcpSolver, calibrationRepetitions);

	boost::chrono::duration<double> calibrationTime = clock::now() - calibrationStart;
	double desiredTestTimeSec = 1.0;
	const int repetitions = std::max(10, std::min(1000000,
		static_cast<int>(desiredTestTimeSec * calibrationRepetitions / calibrationTime.count())));

	clock::time_point time0 = clock::now();

	// Do not actually solve the problem; just copy the input data.
	solveRepeatedly(*problem, nullptr, repetitions);

	clock::time_point time1 = clock::now();

	// Actually solve the problem.
	solveRepeatedly(*problem, &mlcpSolver, repetitions);

	clock::time_point time2 = clock::now();

	boost::chrono::duration<double> elapsedSolveTime = time2 - time1;
	//std::cout << "Elapsed:               " << (elapsedSolveTime.count() * 1000.) << " ms" << std::endl;
	boost::chrono::duration<double> elapsedBaseline = time1 - time0;
	//std::cout << "Baseline:              " << (elapsedBaseline.count() * 1000.) << " ms" << std::endl;
	double solveTimeUsec = (elapsedSolveTime - elapsedBaseline).count() * 1e6 / repetitions;
	double copyTimeUsec = elapsedBaseline.count() * 1e6 / repetitions;
	std::cout << "Average solution time: " << solveTimeUsec << " microseconds (over " <<
		repetitions << " loops)" << std::endl;

	// When refactoring the MLCP code, it can be very useful to compare the execution time before and after making
	// changes.  But you can't usefully compare execution times on different machines, or with different build
	// settings, so you'll need to manually uncomment the following line and adjust the time accordingly.
//	EXPECT_LE(solveTimeUsec, 14.0);
}
