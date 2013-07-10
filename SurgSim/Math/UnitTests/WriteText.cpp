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

#include "SurgSim/Math/UnitTests/WriteText.h"

#include <string>
#include <vector>
#include <stdio.h>

#include <Eigen/Core>

#include <SurgSim/Math/MlcpConstraintType.h>
#include <SurgSim/Math/MlcpConstraintTypeName.h>

#include "SurgSim/Math/UnitTests/MlcpTestData.h"
#include "SurgSim/Math/UnitTests/TextLabels.h"


// output helpers

static void writeCommentBlock(FILE* out, const std::string& text)
{
	const std::string prefix = "# ";
	const std::string suffix = "";

	size_t start = 0;
	while (start < text.length())
	{
		size_t next = text.find_first_of('\n', start);
		if (next == std::string::npos)
		{
			break;
		}
		fprintf(out, "%s%s%s\n", prefix.c_str(), text.substr(start, next - start).c_str(), suffix.c_str());
		start = next+1;
	}
	if (start >= text.length())
	{
		fprintf(out, "%s%s\n", prefix.c_str(), suffix.c_str());
	}
	else
	{
		fprintf(out, "%s%s%s\n", prefix.c_str(), text.substr(start).c_str(), suffix.c_str());
	}
}

static void writeEigenRowVector(FILE* out, const char* label, const Eigen::RowVectorXd& vector)
{
	fprintf(out, "%s (", label);
	for (int i = 0;  i < vector.cols();  i++)
	{
		fprintf(out, " %.20g", vector[i]);
	}
	fprintf(out, ")\n");
}

static void writeEigenVector(FILE* out, const char* label, const Eigen::VectorXd& vector)
{
	writeEigenRowVector(out, label, vector.transpose());
}

static void writeEigenMatrix(FILE* out, const char* label, const Eigen::MatrixXd& matrix)
{
	fprintf(out, "%s (\n", label);
	for (int i = 0;  i < matrix.rows();  ++i)
	{
		writeEigenRowVector(out, " ", matrix.row(i));
	}
	fprintf(out, ")\n");
}




bool writeMlcpTestDataAsText(const std::string& fileName, const MlcpTestData& testData)
{
	FILE* out = fopen(fileName.c_str(), "wt");
	if (! out)
	{
		fprintf(stderr, "File '%s' could not be created to export the current MLCP for GTest\n", fileName.c_str());
		return false;
	}

	int numConstraints = testData.problem.constraintTypes.size();
	int numAtomicConstraints = testData.problem.b.rows();
	if (testData.problem.b.rows() != numAtomicConstraints ||
		testData.problem.A.rows() != numAtomicConstraints || testData.problem.A.cols() != numAtomicConstraints ||
		testData.expectedLambda.rows() != numAtomicConstraints)
	{
		fprintf(stderr, "Inconsistent dimensions in file '%s'!\n", fileName.c_str());
		fclose(out);
		return false;
	}

	writeCommentBlock(out, testData.description);

	fprintf(out, "%s", TEXT_LABEL_FLAGS_LIST);
	for (auto it = testData.flags.begin();  it != testData.flags.end();  ++it)
	{
		fprintf(out, " %s", it->c_str());
	}
	fprintf(out, "\n");


	fprintf(out, "%s %d\n", TEXT_LABEL_NUM_DEGREES_OF_FREEDOM, testData.numDegreesOfFreedom);
	fprintf(out, "%s %d\n", TEXT_LABEL_NUM_CONSTRAINTS, numConstraints);
	fprintf(out, "%s %d\n", TEXT_LABEL_NUM_ATOMIC_CONSTRAINTS, numAtomicConstraints);

	fprintf(out, "%s", TEXT_LABEL_CONSTRAINT_TYPES_LIST);
	for (auto it = testData.problem.constraintTypes.begin();  it != testData.problem.constraintTypes.end();  ++it)
	{
		fprintf(out, " %s", getMlcpConstraintTypeName(*it).c_str());
	}
	fprintf(out, "\n");

	writeEigenVector(out, TEXT_LABEL_E_VIOLATIONS_VECTOR, testData.problem.b);
	writeEigenMatrix(out, TEXT_LABEL_HCHt_MLCP_MATRIX, testData.problem.A);
	writeEigenVector(out, TEXT_LABEL_MU_FRICTION_VECTOR, testData.problem.mu);
	writeEigenVector(out, TEXT_LABEL_LAMBDA_VECTOR, testData.expectedLambda);


	fprintf(out, "%s\n", TEXT_LABEL_END_OF_FILE);

	if (ferror(out))
	{
		fprintf(stderr, "Unexpected error writing file '%s'\n", fileName.c_str());
		return false;

	}
	if (fclose(out))
	{
		fprintf(stderr, "Error closing file '%s' for writing.\n", fileName.c_str());
		return false;
	}

	return true;
}
