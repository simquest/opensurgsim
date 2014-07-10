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

"""Merge a list of input vertices and information from an .obj file

Give a file with a list of input vertices, look through the obj file and find
corresponding vertices, add texture information to the input vertices and store
the resulting list in an output file. The order in the vertex file is maintained
and vertices that cannot be found in the obj file will receive default information
This can be used to add information to a .ply file where the order of the vertices
is already established
E.g. 

vertex file (fragment from .ply file)
1.0 1.0 1.0
2.0 2.0 2.0 
3.0 3.0 3.0

obj file (fragment from .obj file)
v 1.0 1.0 -1.0
v 2.0 2.0 -2.0
vt 1.0 1.0
vt 0.5 0.5

output (fragment, paste this into the vertex section of the .ply file)
1.0 1.0 1.0 1.0 1.0
2.0 2.0 2.0 0.5 0.5
3.0 3.0 3.0 0.0 0.0

Typical usage:
  merge.py input.txt object.obj output.txt
"""

import csv
import argparse

def find(item, vertices) :
	index = 0
	for vertex in vertices:
		if (item == vertex) :
			return index
		index += 1
	return -1
	
vertices = []
uvs = []
uvIndices = []


if __name__ == '__main__':
	parser = argparse.ArgumentParser(
	description="Merge a list of input vertices and information from an .obj file.")
	parser.add_argument('vertices', help='Filename for input vertices.')
	parser.add_argument('obj', help='The .obj file to use.' )
	parser.add_argument('output', help='Filename for output.')
	args = parser.parse_args()

	# this can be a basic obj file, we are only looking at 'v' and 'vt' for now
	with open(args.obj, 'rb') as csvfile:
		reader = csv.reader(csvfile, delimiter = ' ')
		
		vertexIndex = 0
		for row in reader: 
			if (row[0] == 'v') :
				#Go from obj to ply coordinate system, change the sign on the z coord
				if (row[3][0] == '-'):
					row[3] = row[3][1:]
				else:
					row[3] = '-'+row[3]
					
				#swap y and z
				tmp = row[3]
				row[3] = row[2]
				row[2] = tmp			
				vertices.append(row[1:])
			elif (row[0] == 'vt') :
				uvs.append(row[1:])
				uvIndices.append(vertexIndex)
				vertexIndex += 1
			elif (row[0] == 'f') :
				# These are either 
				# f v v v
				# f v/vt v/vt v/vt
				# f v/vt/vn v/vt/vn v/vt/vn
				for item in row[1:]:
					indices = item.split("/")
					if (len(indices) > 1) :
						# .obj indices start with 1
						# put the index of the texture coordinate into the indices map
						uvIndices[int(indices[0])-1] = int(indices[1])-1
	
	result = []
	 
	# Reference is just a list of vertices x,y,z separated by ' '
	# The comparison is done via the strings of the numbers, no accuracy needed, this relies
	# on the fact that the output format is the same and no transformation was done on the
	# vertices
	with open(args.vertices, 'rb') as csvfile:
		reader = csv.reader(csvfile, delimiter = ' ')
		count = 0
		for row in reader:
			texCoord = ["0.0","0.0"]
			index = find(row, vertices)
			if (index != -1) :
				texCoord = uvs[uvIndices[index]]
			row.append(texCoord[0])
			row.append(texCoord[1])
			result.append(row)

			
	with open(args.output, 'wb') as outfile:
		writer = csv.writer(outfile, delimiter = ' ', quoting=csv.QUOTE_NONE)
		for row in result:
			writer.writerow(row)