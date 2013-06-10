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

"""Runs AStyle to reformat the specified source files."""

import argparse
import sys
import os
import subprocess

ASTYLE_FLAGS = ('--indent=force-tab=4', '--style=allman', '--add-brackets',
                '--unpad-paren', '--pad-header', '--align-pointer=type',
                '--indent-preprocessor',
                '--min-conditional-indent=0', '--max-instatement-indent=60',
                # The following flags require AStyle 2.03 or later:
                '--max-code-length=120', '--break-after-logical',
                )

def adjust_environment():
  """Adjust the environment with some likely AStyle locations."""
  if 'SYSTEMDRIVE' in os.environ:
    for prefix in [os.environ['SYSTEMDRIVE'], 'c:', 'd:', 's:']:
      for location in 'SimQuest', 'simquest', 'tools':
        path = prefix + '/' + location + "/AStyle/bin"
        if os.path.isdir(path):
          os.environ['PATH'] += os.pathsep + path

def run_astyle(files):
  command = ['astyle']
  command.extend(ASTYLE_FLAGS)
  command.extend(files)
  try:
    retcode = subprocess.call(command)
    if retcode < 0:
      print sys.argv[0] + ": AStyle terminated by signal", -retcode
      sys.exit(1)
    elif retcode > 0:
      print sys.argv[0] + ": AStyle returned status ", retcode
      sys.exit(1)
  except OSError as e:
    print sys.argv[0] + ": AStyle execution failed:", e
    sys.exit(1)

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description='Reformat source files using AStyle with our options.')
  parser.add_argument('files', metavar='FILE', nargs='*',
                      help='The file names to reformat.')
  args = parser.parse_args()

  adjust_environment()

  if len(args.files) == 0:
    print "{}: Nothing to reformat!".format(sys.argv[0])
  else:
    run_astyle(args.files)

"""
if [ "$#" -eq 0 ]; then
    echo "`basename "$0"`: Nothing to reformat!" >&2
else
fi
"""
