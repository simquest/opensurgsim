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

#include "SurgSim/Testing/MlcpIO/ReadText.h"

#include <string>
#include <vector>
#include <stdio.h>

#include <Eigen/Core>

#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/MlcpConstraintTypeName.h"

#include "SurgSim/Testing/MlcpIO/MlcpTestData.h"
#include "SurgSim/Testing/MlcpIO/TextLabels.h"


// input helpers

static std::string getRawLine(FILE* in)
{
	char buffer[1024];
	if (! fgets(buffer, sizeof(buffer), in))
	{
		return "";
	}
	std::string line(buffer);
	if ((line.length() > 0) && (line[line.length()-1] == '\n'))
	{
		line.resize(line.length()-1);  // remove the last character
	}
	if ((line.length() > 0) && (line[line.length()-1] == '\r'))
	{
		line.resize(line.length()-1);  // remove the last character
	}
	return line;
}

static bool getLine(const std::string& fileName, FILE* in, std::string* line, bool skipEmptyLines = true)
{
	if (ferror(in) || feof(in))
	{
		fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
		return false;
	}
	*line = getRawLine(in);
	while (skipEmptyLines && (line->length() == 0))
	{
		if (ferror(in) || feof(in))
		{
			fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
			return false;
		}
		*line = getRawLine(in);
	}
	if (ferror(in))
	{
		fprintf(stderr, "Unexpected error in file '%s'\n", fileName.c_str());
		return false;
	}
	return true;
}

static bool readInt(const std::string& fileName, FILE* in, const char* label, int* value)
{
	if (ferror(in) || feof(in))
	{
		fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
		return false;
	}
	std::string format = std::string(" ") + label + " %d";
	if (fscanf(in, format.c_str(), value) != 1)
	{
		fprintf(stderr, "Bad integer input in '%s'\n  near text '%s'\n", fileName.c_str(), getRawLine(in).c_str());
		return false;
	}
	if (ferror(in))
	{
		fprintf(stderr, "Unexpected error in file '%s'\n", fileName.c_str());
		return false;
	}
	return true;
}

static bool readEigenRowVector(const std::string& fileName, FILE* in, const char* label, Eigen::RowVectorXd* vector)
{
	// Read the label.
	{
		if (ferror(in) || feof(in))
		{
			fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
			return false;
		}

		if (std::string(label) == "")
		{
			if (fscanf(in, " (") != 0)
			{
				fprintf(stderr, "Unable to read label for Eigen input in '%s'\n  near text '%s'\n",
						fileName.c_str(), getRawLine(in).c_str());
				return false;
			}
		}
		else
		{
			char readLabel[100];
			if (fscanf(in, " %[^(](", readLabel) != 1)
			{
				fprintf(stderr, "Unable to read label for Eigen input in '%s'\n  near text '%s'\n",
						fileName.c_str(), getRawLine(in).c_str());
				return false;
			}
			if (std::string(label) + " " != std::string(readLabel))
			{
				fprintf(stderr, "Bad label for Eigen input in '%s'\n  near text '%s'\n",
						fileName.c_str(), getRawLine(in).c_str());
				return false;
			}
		}
	}

	// Read elements until ')' is found.
	std::vector<double> values;
	while (true)
	{
		// Scan for a non-whitespace character and check it.
		while (true)
		{
			if (ferror(in) || feof(in))
			{
				fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
				return false;
			}

			int nextCharacter = getc(in);
			if (nextCharacter == EOF)  // either EOF or read error
			{
				fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
				return false;
			}
			else if (nextCharacter == ')')
			{
				// We're done reading input.
				if (ferror(in))
				{
					fprintf(stderr, "Unexpected error in file '%s'\n", fileName.c_str());
					return false;
				}

				vector->resize(values.size());
				for (size_t i = 0; i < values.size(); i++)
				{
					(*vector)[i] = values[i];
				}
				return true;
			}
			else if (! isspace(nextCharacter))
			{
				// The next character isn't ')'.  Pretend we never read it, and go on reading data.
				ungetc(nextCharacter, in);
				break;
			}
		}

		if (ferror(in) || feof(in))
		{
			fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
			return false;
		}

		// Ready to read the next element.
		double nextValue = 0;
		if (fscanf(in, " %lg", &nextValue) != 1)
		{
			fprintf(stderr, "Bad data for Eigen input in '%s'\n  near text '%s'\n",
					fileName.c_str(), getRawLine(in).c_str());
			return false;
		}

		values.push_back(nextValue);
	}
}

static bool readEigenVector(const std::string& fileName, FILE* in, const char* label, Eigen::VectorXd* vector)
{
	Eigen::RowVectorXd rowVector;
	if (! readEigenRowVector(fileName, in, label, &rowVector))
	{
		return false;
	}
	*vector = rowVector.transpose();
	return true;
}

static bool readEigenMatrix(const std::string& fileName, FILE* in, const char* label, Eigen::MatrixXd* matrix)
{
	// Read the label.
	{
		if (ferror(in) || feof(in))
		{
			fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
			return false;
		}

		char readLabel[100];
		if (fscanf(in, " %[^(](", readLabel) != 1)
		{
			fprintf(stderr, "Unable to read label for Eigen input in '%s'\n  near text '%s'\n",
					fileName.c_str(), getRawLine(in).c_str());
			return false;
		}
		if (std::string(label) + " " != std::string(readLabel))
		{
			fprintf(stderr, "Bad label for Eigen input in '%s'\n  near text '%s'\n",
					fileName.c_str(), getRawLine(in).c_str());
			return false;
		}
	}

	// Read rows until ')' is found.
	bool firstRow = true;
	matrix->resize(0, 0);
	while (true)
	{
		// Scan for a non-whitespace character and check it.
		while (true)
		{
			if (ferror(in) || feof(in))
			{
				fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
				return false;
			}

			int nextCharacter = getc(in);
			if (nextCharacter == EOF)  // either EOF or read error
			{
				fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
				return false;
			}
			else if (nextCharacter == ')')
			{
				// We're done reading input.
				if (ferror(in))
				{
					fprintf(stderr, "Unexpected error in file '%s'\n", fileName.c_str());
					return false;
				}
				return true;
			}
			else if (! isspace(nextCharacter))
			{
				// The next character isn't ')'.  Pretend we never read it, and go on reading data.
				ungetc(nextCharacter, in);
				break;
			}
		}

		if (ferror(in) || feof(in))
		{
			fprintf(stderr, "Unexpected error or EOF in file '%s'\n", fileName.c_str());
			return false;
		}

		// Ready to read the next row.
		Eigen::RowVectorXd rowVector;
		if (! readEigenRowVector(fileName, in, "", &rowVector))
		{
			return false;
		}
		if (firstRow)
		{
			*matrix = rowVector;
			firstRow = false;
		}
		else
		{
			ptrdiff_t numCols = matrix->cols();
			ptrdiff_t newNumRows = matrix->rows() + 1;
			if (rowVector.cols() != numCols)
			{
				fprintf(stderr, "Inconsistent number of columns for Eigen matrix (%lld vs %lld)\n  in file '%s'\n",
						static_cast<long long int>(numCols), static_cast<long long int>(rowVector.cols()), // NOLINT
						fileName.c_str());
				return false;
			}
			matrix->conservativeResize(newNumRows, numCols);
			matrix->row(newNumRows-1) = rowVector;
		}
	}
}

static bool extractWordList(const std::string& fileName, const std::string& line, const char* label,
							std::vector<std::string>* words)
{
	std::string labelString(label);
	if (line.substr(0, labelString.length()) != labelString)
	{
		fprintf(stderr,
				"Unexpected input line: '%s'\n"
				"  expected:            '%s'...\n"
				"  in file '%s'\n",
				line.c_str(), label, fileName.c_str());
		return false;
	}

	words->clear();
	size_t start = line.find_first_not_of(" \t", labelString.length());
	while (start != std::string::npos)
	{
		size_t end = line.find_first_of(" \t", start+1);
		if (end == std::string::npos)
		{
			words->push_back(line.substr(start));
			break;
		}
		words->push_back(line.substr(start, end - start));
		start = line.find_first_not_of(" \t", end+1);
	}
	return true;
}

static bool readWordList(const std::string& fileName, FILE* in, const char* label, std::vector<std::string>* words)
{
	std::string line;
	if (! getLine(fileName, in, &line))
	{
		return false;
	}

	return extractWordList(fileName, line, label, words);
}

static bool checkInputLine(const std::string& fileName, const std::string& expected, const std::string& line)
{
	if (line != expected)
	{
		fprintf(stderr,
				"Unexpected input line: '%s'\n"
				"  expected:            '%s'\n"
				"  in file '%s'\n",
				line.c_str(), expected.c_str(), fileName.c_str());
		return false;
	}
	return true;
}



// actually write the data

bool readMlcpTestDataAsText(const std::string& fileName, MlcpTestData* testData)
{
	FILE* in = fopen(fileName.c_str(), "rt");
	if (! in)
	{
		fprintf(stderr, "File '%s' could not be opened to read the current MLCP for GTest\n", fileName.c_str());
		return false;
	}

	testData->description.clear();

	std::string line;
	if (! getLine(fileName, in, &line))
	{
		fprintf(stderr, "Failed to read first line from the file.\n");
		return false;
	}

	while ((line.length() > 0) && (line[0] == '#'))
	{
		if (testData->description.length() > 0)
		{
			testData->description += "\n";
		}
		if ((line.length() > 1) && (line[1] == ' '))
		{
			testData->description += line.substr(2);
		}
		else
		{
			testData->description += line.substr(1);
		}

		if (! getLine(fileName, in, &line))
		{
			fprintf(stderr, "Failed to read header comment block from the file.\n");
			return false;
		}
	}

	// We have already read the next line, so we just need to parse it.
	if (! extractWordList(fileName, line, TEXT_LABEL_FLAGS_LIST, &(testData->flags)))
	{
		return false;
	}

	if (! readInt(fileName, in, TEXT_LABEL_NUM_DEGREES_OF_FREEDOM, &(testData->numDegreesOfFreedom)))
	{
		return false;
	}

	int numConstraints;
	if (! readInt(fileName, in, TEXT_LABEL_NUM_CONSTRAINTS, &numConstraints))
	{
		return false;
	}

	int numAtomicConstraints;
	if (! readInt(fileName, in, TEXT_LABEL_NUM_ATOMIC_CONSTRAINTS, &numAtomicConstraints))
	{
		return false;
	}

	std::vector<std::string> constraintTypeNames;
	if (! readWordList(fileName, in, TEXT_LABEL_CONSTRAINT_TYPES_LIST, &constraintTypeNames))
	{
		return false;
	}
	if (static_cast<int>(constraintTypeNames.size()) != numConstraints)
	{
		fprintf(stderr, "Expected %d constraint types, saw %u\n  in file '%s'\n",
				numConstraints, static_cast<unsigned int>(constraintTypeNames.size()), fileName.c_str());
		return false;
	}
	testData->problem.constraintTypes.resize(numConstraints);
	for (size_t i = 0;  i < testData->problem.constraintTypes.size();  ++i)
	{
		SurgSim::Math::MlcpConstraintType currentType =
			SurgSim::Math::getMlcpConstraintTypeValue(constraintTypeNames[i]);
		if (currentType == SurgSim::Math::MLCP_INVALID_CONSTRAINT)
		{
			fprintf(stderr, "Unexpected constraint type string: '%s'\n  in file '%s'\n",
					constraintTypeNames[i].c_str(), fileName.c_str());
			return false;
		}
		testData->problem.constraintTypes[i] = currentType;
	}

	{
		Eigen::VectorXd b;
		Eigen::MatrixXd A;
		Eigen::VectorXd mu;
		Eigen::VectorXd expectedLambda;

		if (! readEigenVector(fileName, in, TEXT_LABEL_E_VIOLATIONS_VECTOR, &b))
		{
			return false;
		}

		if (! readEigenMatrix(fileName, in, TEXT_LABEL_HCHt_MLCP_MATRIX, &A))
		{
			return false;
		}

		if (! readEigenVector(fileName, in, TEXT_LABEL_MU_FRICTION_VECTOR, &mu))
		{
			return false;
		}

		if (! readEigenVector(fileName, in, TEXT_LABEL_LAMBDA_VECTOR, &expectedLambda))
		{
			return false;
		}

		testData->problem.b = b;
		testData->problem.A = A;
		testData->problem.mu = mu;
		testData->expectedLambda = expectedLambda;
	}

	if ((testData->problem.b.rows() != numAtomicConstraints) || (testData->problem.b.cols() != 1))
	{
		fprintf(stderr, "Expected %dx%d vector E, saw %dx%d\n  in file '%s'\n",
				numAtomicConstraints, 1, static_cast<int>(testData->problem.b.rows()),
				static_cast<int>(testData->problem.b.cols()), fileName.c_str());
		return false;
	}
	if ((testData->problem.A.rows() != numAtomicConstraints) || (testData->problem.A.cols() != numAtomicConstraints))
	{
		fprintf(stderr, "Expected %dx%d matrix A, saw %dx%d\n  in file '%s'\n",
				numAtomicConstraints, numAtomicConstraints, static_cast<int>(testData->problem.A.rows()),
				static_cast<int>(testData->problem.A.cols()), fileName.c_str());
		return false;
	}
	if ((testData->problem.mu.rows() != numConstraints) || (testData->problem.mu.cols() != 1))
	{
		fprintf(stderr, "Expected %dx%d vector mu, saw %dx%d\n  in file '%s'\n",
				numConstraints, 1, static_cast<int>(testData->problem.mu.rows()),
				static_cast<int>(testData->problem.mu.cols()), fileName.c_str());
		return false;
	}
	if ((testData->expectedLambda.rows() != numAtomicConstraints) || (testData->expectedLambda.cols() != 1))
	{
		fprintf(stderr, "Expected %dx%d vector lambda, saw %dx%d\n  in file '%s'\n",
				numAtomicConstraints, 1, static_cast<int>(testData->expectedLambda.rows()),
				static_cast<int>(testData->expectedLambda.cols()), fileName.c_str());
		return false;
	}

	if (! getLine(fileName, in, &line) || ! checkInputLine(fileName, TEXT_LABEL_END_OF_FILE, line))
	{
		fprintf(stderr, "Expected %d constraint types, saw %u\n", numConstraints,
				static_cast<unsigned int>(constraintTypeNames.size()));
		return false;
	}

	if (ferror(in))
	{
		fprintf(stderr, "Unexpected error near the end of file '%s'\n", fileName.c_str());
		return false;

	}
	if (fclose(in))
	{
		fprintf(stderr, "Error closing file '%s' for reading.\n", fileName.c_str());
		return false;
	}

	return true;
}
