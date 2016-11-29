// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

/// \file
/// Tests for the Gauss-Seidel implementation of the MLCP solver.

#include <math.h>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include <boost/chrono.hpp>

#include "SurgSim/Math/Valid.h"
#include "SurgSim/Math/MlcpGaussSeidelSolver.h"
#include "SurgSim/Math/MlcpSolution.h"
#include "SurgSim/Testing/MlcpIO/MlcpTestData.h"
#include "SurgSim/Testing/MlcpIO/ReadText.h"

using SurgSim::Math::isValid;
using SurgSim::Math::MlcpGaussSeidelSolver;

TEST(MlcpGaussSeidelSolverTests, CanConstruct)
{
	//ASSERT_NO_THROW({
	MlcpGaussSeidelSolver mlcpSolver(1.0, 1.0, 100);
}


static void solveAndCompareResult(const std::string& fileName,
								  double gsSolverPrecision = 1e-9, double gsContactTolerance = 1e-9,
								  int gsMaxIterations = 100)
{
	SCOPED_TRACE("while running test " + fileName);

	const std::shared_ptr<MlcpTestData> data = loadTestData(fileName);
	ASSERT_TRUE(data != nullptr) << "Failed to load " << fileName;

	// NB: need to make the solver calls const-correct.
	Eigen::MatrixXd A = data->problem.A;
	Eigen::VectorXd b = data->problem.b;
	Eigen::VectorXd mu = data->problem.mu;
	std::vector<SurgSim::Math::MlcpConstraintType> constraintTypes = data->problem.constraintTypes;

	const size_t size = data->getSize();
	SurgSim::Math::MlcpSolution solution;
	solution.x.resize(size);

	//################################
	// Gauss-Seidel solver
	MlcpGaussSeidelSolver mlcpSolver(gsSolverPrecision, gsContactTolerance,
									 gsMaxIterations);

	solution.x.setZero();

	// XXX set ratio to 1
	mlcpSolver.solve(data->problem, &solution);

	ASSERT_EQ(static_cast<SurgSim::Math::Vector::Index>(size), solution.x.rows());
	ASSERT_EQ(static_cast<Eigen::VectorXd::Index>(size), data->expectedLambda.rows());
	if (size > 0)
	{
		ASSERT_TRUE(isValid(solution.x)) << solution.x;
	}

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
		if ((*it) != SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		{
			isSimpleLcp = false;
		}
	}

	if (isSimpleLcp && (size > 0))
	{
		EXPECT_GE(solution.x.minCoeff(), 0.0) << "x contains negative coefficients:" << std::endl << solution.x;

		Eigen::VectorXd c = A * solution.x + b;
		EXPECT_GE(c.minCoeff(), -gsSolverPrecision) << "Ax+b contains negative coefficients:" << std::endl << c;

		// Orthogonality test should be taking into account the scaling factor
		double maxAbsForce = abs(solution.x.maxCoeff());
		if (abs(solution.x.minCoeff()) > maxAbsForce)
		{
			maxAbsForce = abs(solution.x.minCoeff());
		}
		const double epsilon = 1e-9 * maxAbsForce;
		EXPECT_NEAR(0.0, c.dot(solution.x), epsilon) << "Ax+b is not orthogonal to x!" << std::endl <<
			"x:" << std::endl << solution.x << std::endl << "Ax+b:" << std::endl << c;
	}

	EXPECT_TRUE(solution.x.isApprox(data->expectedLambda)) << "lambda:" << std::endl << solution.x << std::endl <<
		"expected:" << std::endl << data->expectedLambda;

//	double convergenceCriteria=0.0;
//	bool validSignorini=false;
//	int nbContactToSkip=0;
//	int currentAtomicIndex = calculateConvergenceCriteria(lambda, nbContactToSkip, convergenceCriteria,
//		validSignorini, 1.0);
}

TEST(MlcpGaussSeidelSolverTests, SolveOriginal)
{
	const double gsSolverPrecision = 1e-4;
	const double gsContactTolerance = 2e-4;
	int gsMaxIterations = 30;
	solveAndCompareResult("mlcpOriginalTest.txt", gsSolverPrecision, gsContactTolerance, gsMaxIterations);
}

TEST(MlcpGaussSeidelSolverTests, SolveSequence)
{
	for (int i = 0;  i <= 9;  ++i)
	{
		solveAndCompareResult(getTestFileName("mlcpTest", i, ".txt"));
	}
}

static void solveRepeatedly(const MlcpTestData& data,
							/*XXX const */ MlcpGaussSeidelSolver* mlcpSolver,
							const int repetitions)
{
	// NB: need to make the solver calls const-correct.
	SurgSim::Math::MlcpProblem problem;
	SurgSim::Math::MlcpSolution solution;
	std::vector<SurgSim::Math::MlcpConstraintType> constraintTypes;

	const size_t size = data.getSize();
	solution.x.resize(size);

	for (int i = repetitions;  i > 0;  --i)
	{
		problem = data.problem;

		if (mlcpSolver)
		{
			solution.x.setZero();
			mlcpSolver->solve(problem, &solution);
		}
	}
}

static double measureExecutionTimeUsec(const std::string& fileName,
									   double gsSolverPrecision = 1e-8, double gsContactTolerance = 1e-8,
									   int gsMaxIterations = 20)
{
	SCOPED_TRACE("while running test " + fileName);

	const std::shared_ptr<MlcpTestData> data = loadTestData(fileName);
	EXPECT_TRUE(data != nullptr) << "Failed to load " << fileName;

	MlcpGaussSeidelSolver mlcpSolver(gsSolverPrecision, gsContactTolerance, gsMaxIterations);

	typedef boost::chrono::high_resolution_clock clock;
	clock::time_point calibrationStart = clock::now();

	const int calibrationRepetitions = 1000;
	solveRepeatedly(*data, &mlcpSolver, calibrationRepetitions);

	boost::chrono::duration<double> calibrationTime = clock::now() - calibrationStart;
	double desiredTestTimeSec = 1.0;
	double desiredRepetitions = desiredTestTimeSec * calibrationRepetitions / calibrationTime.count();
	const int repetitions = std::max(10, std::min(1000000, static_cast<int>(desiredRepetitions)));

	clock::time_point time0 = clock::now();

	// Do not actually solve the problem; just copy the input data.
	solveRepeatedly(*data, nullptr, repetitions);

	clock::time_point time1 = clock::now();

	// Actually solve the problem.
	solveRepeatedly(*data, &mlcpSolver, repetitions);

	clock::time_point time2 = clock::now();

	boost::chrono::duration<double> elapsedSolveTime = time2 - time1;
	//std::cout << "Elapsed:               " << (elapsedSolveTime.count() * 1000.) << " ms" << std::endl;
	boost::chrono::duration<double> elapsedBaseline = time1 - time0;
	//std::cout << "Baseline:              " << (elapsedBaseline.count() * 1000.) << " ms" << std::endl;
	double solveTimeUsec = (elapsedSolveTime - elapsedBaseline).count() * 1e6 / repetitions;
	double copyTimeUsec = elapsedBaseline.count() * 1e6 / repetitions;
	std::cout << "Average solution time: " << solveTimeUsec << " microseconds (~" << (solveTimeUsec+copyTimeUsec) <<
		"-" << copyTimeUsec << ") over " << repetitions << " loops" << std::endl;
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
	EXPECT_LE(solveTimeUsec, 1e6);
}
