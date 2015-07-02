#!/usr/bin/python

# This file is a part of the OpenSurgSim project.
# Copyright 2012-2013, SimQuest Solutions Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert a TetGen set of files .node/.ele into a PLY file readable by OSS

Typical usage:
  TetGen_to_PLY.py input.node input.ele [input.face] [input.fixedNodes] output.ply
"""

import csv
import argparse

if __name__ == '__main__':
	parser = argparse.ArgumentParser(
	description="Convert a TetGen set of filename into a PLY file readable by OSS.")
	parser.add_argument('nodes', help='Filename for the nodes input.')
	parser.add_argument('elements', help='Filename for the tetrahedrons input.')
	parser.add_argument('--faces', help='Filename for the faces input.')
	parser.add_argument('--fixedNodes', help='Filename for the fixed node indices.')
	parser.add_argument('--MassDensity', help='Mass density, default.', default='2000.0')
	parser.add_argument('--YoungModulus', help='Young modulus.', default='1e7')
	parser.add_argument('--PoissonRatio', help='Poisson ratio.', default='0.45')
	parser.add_argument('output', help='Filename for the PLY output.')
	args = parser.parse_args()

	numNodes = 0        # Number of nodes, will be read from the header of the .node file
	numElements = 0     # Number of elements, will be read from the header of the .ele file (support triangle (3) and tetrahedron(4) elements)
	elementSize = 0     # Element size 3 for triangle, 4 for tetrahedron, will be read from the header of the .ele file
	numFaces = 0        # Number of triangulated faces (if a .face is provided, number of entries in this file)
	numFixedNodes = 0   # Number of fixed nodes (if a .fixedNode is provided, number of entries in this file)

	with open(args.output, 'wb') as csvOutputFile:
		writer = csv.writer(csvOutputFile, delimiter = ' ', quoting=csv.QUOTE_NONE)

		writer.writerow(['ply'])
		writer.writerow(['format', 'ascii', '1.0'])
		writer.writerow(['comment', 'Created', 'by', 'hand'])

		with open(args.nodes, 'rb') as csvNodeFile:
			reader = csv.reader(csvNodeFile, delimiter = ' ', skipinitialspace = True)
			row = reader.next()
			numNodes = row[0]
			# Enforcing the need for 3 pieces of information per node (x, y, z)
			if not row[1] == '3':
				raise Exception('Invalid node information in ' + args.node + '. Node dimension (expecting 3) was ' + row[1])
		writer.writerow(['element', 'vertex', numNodes])
		writer.writerow(['property', 'double', 'x'])
		writer.writerow(['property', 'double', 'y'])
		writer.writerow(['property', 'double', 'z'])

		with open(args.elements, 'rb') as csvElementFile:
			reader = csv.reader(csvElementFile, delimiter = ' ', skipinitialspace = True)
			row = reader.next()
			numElements = row[0]
			if not row[1] == '4' and not row[1] == '3':
				raise Exception('Invalid triangle/tetrahedron information in ' + args.ele + '. Element dimension (expecting 3 or 4) was ' + row[1])
		if row[1] == '4':
			writer.writerow(['element', '3d_element', numElements])
			elementSize = 4
		if row[1] == '3':
			writer.writerow(['element', '2d_element', numElements])
			elementSize = 3
		writer.writerow(['property', 'list', 'uint', 'uint', 'vertex_indices'])

		if args.faces:
			with open(args.faces, 'rb') as csvFaceFile:
				reader = csv.reader(csvFaceFile, delimiter = ' ', skipinitialspace = True)
				row = reader.next()
				numFaces = row[0]
			writer.writerow(['element', 'face', numFaces])
			writer.writerow(['property', 'list', 'uint', 'uint', 'vertex_indices'])

		if args.fixedNodes:
			with open(args.fixedNodes, 'rb') as csvFixedNodeFile:
				readerFixedNodes = csv.reader(csvFixedNodeFile)
				for row in readerFixedNodes:
					numFixedNodes = numFixedNodes + 1
			writer.writerow(['element', 'boundary_condition', numFixedNodes])
			writer.writerow(['property', 'uint', 'vertex_index'])

		# Extra parameter (thickness) if the element is a triangle
		if elementSize == 3:
			writer.writerow(['element', 'thickness', 1])
			writer.writerow(['property', 'double', 'value'])

		writer.writerow(['element', 'material', 1])
		writer.writerow(['property', 'double', 'mass_density'])
		writer.writerow(['property', 'double', 'poisson_ratio'])
		writer.writerow(['property', 'double', 'young_modulus'])
		writer.writerow(['end_header'])

		# Parse the .node file to format the nodes (x,y,z)
		with open(args.nodes, 'rb') as csvNodeFile:
			reader = csv.reader(csvNodeFile, delimiter = ' ', skipinitialspace = True)
			rowId = 0
			# Write all nodes
			for row in reader:
				# Skip the commented lines (especially the last line of the .node generated by TetGen)
				if row[0][0] == '#':
					continue
				# Skip the first line (header information), detected by the number of nodes being different than the expected node index
				if not int(row[0]) == rowId:
					continue
				writer.writerow(row[1:])
				rowId = rowId + 1

		# Parse the .ele file to format the tetrahedrons
		with open(args.elements, 'rb') as csvElementFile:
			reader = csv.reader(csvElementFile, delimiter = ' ', skipinitialspace = True)
			rowId = 0
			# Write all tetrahedrons
			for row in reader:
				# Skip the commented lines (especially the last line of the .ele generated by TetGen)
				if row[0][0] == '#':
					continue
				# Skip the first line (header information), detected by the number of elements being different than the expected element index
				if not int(row[0]) == rowId:
					continue
				row[0] = elementSize
				writer.writerow(row)
				rowId = rowId + 1

		# Parse the .face file to format the triangulated faces (if a file is specified)
		if args.faces:
			with open(args.faces, 'rb') as csvFaceFile:
				reader = csv.reader(csvFaceFile, delimiter = ' ', skipinitialspace = True)
				rowId = 0
				# Write all faces
				for row in reader:
					# Skip the commented lines (especially the last line of the .face generated by TetGen)
					if row[0][0] == '#':
						continue
					# Skip the first line (header information), detected by the number of faces being different than the expected face index
					if not int(row[0]) == rowId:
						continue
					row[0] = 3
					writer.writerow(row[:4])
					rowId = rowId + 1

		# Write the fixed nodes (boundary conditions) if any
		if args.fixedNodes:
			with open(args.fixedNodes, 'rb') as csvFixedNodeFile:
				readerFixedNodes = csv.reader(csvFixedNodeFile, delimiter = ' ', skipinitialspace = True)
				for row in readerFixedNodes:
					writer.writerow(row)

		# Write a default thickness if the element is a triangle
		# We should have something closer to 1/100th of the mesh size if we wanted to have a more automated tool
		if elementSize == 3:
			writer.writerow(['0.01'])

		# Write the material (default parameters)
		writer.writerow([args.MassDensity, args.PoissonRatio, args.YoungModulus])
