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

"""Alphabetize the CMake lists of headers and sources.

Typical usage:
  Tools/alphabetize-cmakelists.py SurgSim/CMakeLists.py
or:
  find . -name 'build*' -prune -o -name CMakeLists.txt -print \
    | xargs python Tools/alphabetize-cmakelists.py
"""

import argparse
import sys
import re

def slurp_lines(file):
  try:
    with open(file, 'r') as f:
      return f.read().splitlines()
  except IOError as e:
    print >> sys.stderr, e
    return None

def spew_lines(file, lines):
  try:
    with open(file, 'wb') as f:
      for line in lines:
        f.write(line + "\n")
      return True
  except IOError as e:
    print >> sys.stderr, e
    return False

def filename_sort_filter(file):
  return re.sub(r'Tests?$', '', re.sub(r'\.(?:h|cpp)$', '', file)).lower()

def alphabetize(file, lines):
  if lines is None:
    return None
  (result, indent, var_name, content) = ([], '', None, None)
  for line in lines:
    if content is not None:
      (xline, had_closing_paren) = re.subn(r'\)\s*$', '', line)
      if re.search(r'[(){}]', xline):
        print >> sys.stderr, file + ": Bad line '" + line + "'"
        # put the content lines back unmodified
        result.extend(content)
        result.append(line)
        continue
      content.append(xline)
      if had_closing_paren:
        content = content[1:]  # strip the original set(... line
        content = filter(lambda x: len(x), map(lambda x: x.strip(), content))
        #print var_name + ":", content
        content = sorted(content, key=filename_sort_filter)
        result.append(indent + "set(" + var_name)
        result.extend(map(lambda x: indent + "\t" + x, content))
        result.append(indent + ")")
        content = None
    else:
      match = re.search(
        r'^(\s*)[Ss][Ee][Tt]\s*\(\s*(\w+(?:HEADERS|SOURCES))\s*$', line)
      if match:
        indent = match.group(1)
        var_name = match.group(2)
        content = [ line ]  # include original line, will be removed later
      else:
        result.append(line)
  if result == lines:
    return None
  return result

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description='Alphabetize CMake lists of headers and sources.')
  parser.add_argument('files', metavar='FILE', nargs='*',
                      help='The file names to reformat.')
  args = parser.parse_args()

  touched = False
  for file in args.files:
    if not re.search(r'\.(?:txt|cmake)$', file):
      print >> sys.stderr, file + ": not a CMake input file!"
    else:
      lines = alphabetize(file, slurp_lines(file))
      if lines is not None:
        spew_lines(file, lines)
        print "Updated", file
        touched = True

  if not touched:
    print "{}: Nothing to update!".format(sys.argv[0])
