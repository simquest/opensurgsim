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


static void compareResult(const std::string& fileName,
                          double gsSolverPrecision = 1e-8, double gsContactTolerance = 1e-8, int gsMaxIterations = 20)
{
	const std::shared_ptr<MlcpTestData> problem = loadTestProblem(fileName);
	ASSERT_TRUE(problem) << "Failed to load " << fileName;

	// NB: need to make the solver calls const-correct.
	Eigen::MatrixXd HCHt = problem->HCHt;
	Eigen::VectorXd E = problem->E;
	Eigen::VectorXd mu = problem->mu;
	std::vector<MLCP_Constraint> constraintTypes = problem->constraintTypes;

	const int size = problem->getSize();
	Eigen::VectorXd lambda(size);

	//################################
	// Gauss-Seidel solver
	MLCP_GaussSeidel_Christian<Eigen::MatrixXd, Eigen::VectorXd> mlcpSolver(gsSolverPrecision, gsContactTolerance,
	        gsMaxIterations);

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
	ASSERT_TRUE(isValid(lambda)) << lambda;

	// TODO(advornik): Because this is a mixed LCP problem *with friction*, we can't just easily check that
	//   all x are positive (the frictional entries may not be), or that all Ax+b are positive(ditto).
	//   We need to either (a) make the test aware of the meaning of the constraint types, or (b) get rid
	//   of the constraint types and flag the frictional DOFs more directly.
	//
	//   For now, we check if the MLCP is really a pure LCP (only unilaterals without friction), and we
	//   perform some simple checks that apply to that case and that case only.
	bool isSimpleLcp = true;
	for (auto it = constraintTypes.cbegin();  it != constraintTypes.cend();  ++it)
	{
		if ((*it) != MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		{
			isSimpleLcp = false;
		}
	}

	if (isSimpleLcp && (size > 0))
	{
		EXPECT_GE(lambda.minCoeff(), 0.0) << "x contains negative coefficients:" << std::endl << lambda;

		Eigen::VectorXd c = problem->HCHt * lambda + problem->E;
		EXPECT_GE(c.minCoeff(), -gsSolverPrecision) << "Ax+b contains negative coefficients:" << std::endl << c;
		EXPECT_NEAR(0.0, c.dot(lambda), 1e-9) << "Ax+b is not orthogonal to x!" << std::endl <<
			"x:" << std::endl << lambda << std::endl << "Ax+b:" << std::endl << c;
	}

	EXPECT_TRUE(lambda.isApprox(problem->expectedLambda)) << "lambda:" << std::endl << lambda << std::endl <<
		"expected:" << std::endl << problem->expectedLambda;

//	double convergenceCriteria=0.0;
//	bool validSignorini=false;
//	int nbContactToSkip=0;
//	int currentAtomicIndex = calculateConvergenceCriteria(lambda, nbContactToSkip, convergenceCriteria,
//		validSignorini, 1.0);
//	printf("\tStatus method final [convergence criteria=%g, Signorini=%d]\n",convergenceCriteria,validSignorini?1:0);
	printf("############\n");
}

TEST(MlcpGaussSeidelSolverTests, CompareResultOriginal)
{
	const double gsSolverPrecision = 1e-4;
	const double gsContactTolerance = 2e-5;
	int gsMaxIterations = 30;
	compareResult("mlcpOriginalTest.txt", gsSolverPrecision, gsContactTolerance, gsMaxIterations);
}

TEST(MlcpGaussSeidelSolverTests, CompareResultsSequence)
{
	for (int i = 0;  i <= 9;  ++i)
	{
		std::string index = std::to_string(i);
		while (index.length() < 3)
		{
			index = "0" + index;
		}

		SCOPED_TRACE("while running test " + index);
		printf("-- TEST %s --\n", index.c_str());
		compareResult("mlcpTest" + index + ".txt");
	}
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

static double measureExecutionTimeUsec(const std::string& fileName,
                                       double gsSolverPrecision = 1e-8, double gsContactTolerance = 1e-8,
                                       int gsMaxIterations = 20)
{
	const std::shared_ptr<MlcpTestData> problem = loadTestProblem(fileName);
	EXPECT_TRUE(problem) << "Failed to load " << fileName;

	MLCP_GaussSeidel_Christian<Eigen::MatrixXd,Eigen::VectorXd> mlcpSolver(gsSolverPrecision, gsContactTolerance,
	        gsMaxIterations);

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
	return solveTimeUsec;
}

TEST(MlcpGaussSeidelSolverTests, MeasureExecutionTimeOriginal)
{
	const double gsSolverPrecision = 1e-4;
	const double gsContactTolerance = 2e-5;
	int gsMaxIterations = 30;
	double solveTimeUsec = measureExecutionTimeUsec("mlcpOriginalTest.txt", gsSolverPrecision, gsContactTolerance,
	                                                gsMaxIterations);

	// When refactoring the MLCP code, it can be very useful to compare the execution time before and after making
	// changes.  But you can't usefully compare execution times on different machines, or with different build
	// settings, so you'll need to manually uncomment the following line and adjust the time accordingly.
//	EXPECT_LE(solveTimeUsec, 14.0);
}
