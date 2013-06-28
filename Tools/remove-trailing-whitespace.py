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

"""Remove all trailing whitespace from a file.

Typical usage:
  Tools/remove-trailing-whitespace.py Foo.h
or:
  find SurgSim \( -name '*.h' -o -name '*.cpp' \) -print \
    | xargs python Tools/remove-trailing-whitespace.py
"""

import argparse
import sys
import re

def slurp_raw_lines(file):
  try:
    with open(file, 'rb') as f:
      return f.readlines()
  except IOError as e:
    print >> sys.stderr, e
    return None

def spew_raw_lines(file, lines):
  try:
    with open(file, 'wb') as f:
      for line in lines:
        f.write(line)
      return True
  except IOError as e:
    print >> sys.stderr, e
    return False

def update(file, lines):
  if lines is None:
    return None
  eol = "\n"
  if len(lines) and re.search(r'\r\n$', lines[0]):
    eol = "\r\n"
  result = map(lambda x: x.rstrip() + eol, lines)
  # If the trailing newline was missing, but there's no other trailing
  # whitespace, leave well enough alone.
  if len(lines) and lines[-1] == lines[-1].rstrip():
    result[-1] = lines[-1]
  if result == lines:
    return None
  return result

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description="Remove all trailing whitespace from a file.")
  parser.add_argument('files', metavar='FILE', nargs='*',
                      help='The file names to modify.')
  args = parser.parse_args()

  touched = False
  for file in args.files:
    lines = update(file, slurp_raw_lines(file))
    if lines is not None:
      spew_raw_lines(file, lines)
      print "Updated", file
      touched = True

  if not touched:
    print "{}: Nothing to update!".format(sys.argv[0])
