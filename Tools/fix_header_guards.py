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

"""Fix the header include guards in files."""

import argparse
import sys
import subprocess
import re
import os

def slurp_numbered_lines(file):
  try:
    with open(file, 'r') as f:
      lines = list(enumerate(f.read().splitlines(), start=0))
      if not lines:
        print >> sys.stderr, file + ": warning, file is empty."
      return lines
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

HEADER_GUARD_DIRECTORY_PREFIX_CACHE = {}

def get_header_guard_directory_prefix(dir_path):
  if dir_path not in HEADER_GUARD_DIRECTORY_PREFIX_CACHE:
    prefix = ""
    path = dir_path
    while not (os.path.exists(os.path.join(path, 'LICENSE')) or
               os.path.exists(os.path.join(path, '.git'))):
      (parent, dirname) = os.path.split(path)
      assert parent != path
      prefix = re.sub(r'\W+', '', dirname).upper() + '_' + prefix
      path = parent
    HEADER_GUARD_DIRECTORY_PREFIX_CACHE[dir_path] = prefix

  return HEADER_GUARD_DIRECTORY_PREFIX_CACHE[dir_path]

def get_expected_header_guard(file):
  (dir_path, filename) = os.path.split(os.path.abspath(file))
  guard = re.sub(r'\W+', '_', filename).upper()

  while not (os.path.exists(os.path.join(dir_path, 'LICENSE')) or
             os.path.exists(os.path.join(dir_path, '.git'))):
    (parent, dirname) = os.path.split(dir_path)
    assert parent != dir_path
    guard = re.sub(r'\W+', '', dirname).upper() + '_' + guard
    dir_path = parent

  return guard

def fix_header_guards(flags, file, lines):
  ifndef = filter(lambda x: re.search(r'^\s*#\s*ifndef\b', x[1]), lines)
  if not ifndef:
    print >> sys.stderr, file + ": no #ifndef lines!"
    return None

  guard_expected = get_expected_header_guard(file)

  lines[ifndef[0][0]] = (ifndef[0][0],
                         re.sub(r'^(\s*#\s*ifndef)\s+\w+((?:\s.*)?)$',
                                r'\1 ' + guard_expected + r'\2',
                                ifndef[0][1]))

  define = filter(lambda x: re.search(r'^\s*#\s*define\b', x[1]), lines)
  if not define:
    print >> sys.stderr, file + ": no #define lines!"
    return None

  lines[define[0][0]] = (define[0][0],
                         re.sub(r'^(\s*#\s*define)\s+\w+((?:\s.*)?)$',
                                r'\1 ' + guard_expected + r'\2',
                                define[0][1]))

  endif = filter(lambda x: re.search(r'^\s*#\s*endif\b', x[1]), lines)
  if not endif:
    print >> sys.stderr, file + ": no #endif lines!"
    return None

  update_endif = False
  if not args.lenient_endif:
    update_endif = True
  if (re.search(r'^\s*#\s*endif\s*//', endif[-1][1]) and
      not re.search(r'^\s*#\s*endif\s+//\s+' + guard_expected + r'\s*$',
                    endif[-1][1])):
    update_endif = True  # The comment exists but isn't right.

  if update_endif:
    lines[endif[-1][0]] = (endif[-1][0],
                           re.sub(r'^(\s*#\s*endif)\b.*$',
                                  r'\1  // ' + guard_expected,
                                  endif[-1][1]))
  return lines

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description='Fix the header include guards in files.')
  parser.add_argument('--lenient-endif', action='store_true',
                      help='Fix #endif comments only if present and wrong.')
  parser.add_argument('files', metavar='FILE', nargs='*',
                      help='The file names to check.')
  args = parser.parse_args()

  for file in args.files:
    if not re.search(r'\.h$', file):
      print >> sys.stderr, file + ": Ignored."
      continue

    lines = slurp_numbered_lines(file)
    if not lines:
      continue
    updated = fix_header_guards(args, file,
                                map(lambda x: (x[0], x[1]), lines))
    if not updated:
      continue
    if updated == lines:
      continue
    output = map(lambda x: x[1] + "\n", updated)
    spew_raw_lines(file, output)
    print >> sys.stderr, file + ": Updated."
