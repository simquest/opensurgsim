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

"""Runs Google's cpplint.py on C++ files.

This script tries to flag violations of the OpenSurgSim coding
standards in C++ code.

This is certainly not a substitute for looking at the code by hand:
many violations will not be caught by the script, and some amount of
flagged code will turn out to be correct.  But it does make it easier
to notice certain common errors.
"""

import argparse
import sys
import subprocess
import re

STANDARD_WARNING_FORMAT = \
    "{file}:{line}: {text} [{category}] [{level}]"
VISUAL_STUDIO_WARNING_FORMAT = \
    "{file}({line}): {warning} {vscategory}[{level}]: {text}"
WARNING_FORMAT = STANDARD_WARNING_FORMAT

def emit_warning(fields):
  fields = dict(fields)
  fmt = WARNING_FORMAT
  # add a "vscategory" field which is like "category" but never empty
  if 'category' in fields and fields['category'] is not None:
    fields['vscategory'] = fields['category']
  else:
    fmt = re.sub(r'\s*\[\{category[^\}]*\}\w\]', '', fmt)
    fmt = re.sub(r'\s*\{category[^\}]*\}', '', fmt)
    fields['category'] = '???'  # panic button
    fields['vscategory'] = 'RL'
  if 'level' not in fields:
    fmt = re.sub(r'\s*\[\{level[^\}]*\}\]', '', fmt)
    fmt = re.sub(r'\s*\{level[^\}]*\}', '', fmt)
    fields['level'] = '???'  # panic button
  if 'line' not in fields:
    fmt = re.sub(r'\s*\{line[^\}]*\}:', '', fmt)
    fmt = re.sub(r'\s*\(\{line[^\}]*\}\)', '', fmt)
    fmt = re.sub(r'\s*\{line[^\}]*\}', '', fmt)
    fields['line'] = '???'  # panic button
  if 'warning' not in fields:
    fields['warning'] = 'warning'
  print fmt.format(**fields)

def emit_error(fields):
  fields = dict(fields)
  fields['warning'] = 'error'
  emit_warning(fields)

def run_cpplint(script, filter, files):
  if not files:
    return False
  argv = [sys.executable, script]
  if filter is not None:
    argv.extend(['--filter', filter])
  argv.extend(files)

  try:
    cmd = subprocess.Popen(argv, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
  except OSError as e:
    print "Failed to run '" + script + "':\n\t", e
    return False

  saw_output = False
  for line in cmd.stdout:
    saw_output = True
    line = line.rstrip()
    match = re.search(r'^(.*?):(\d+):\s+(.*?)\s+\[(\S+)\]\s+\[(\d+)\]\s*$',
                      line)
    if match:
      fields = dict(zip(['file', 'line', 'text', 'category', 'level'],
                        match.groups()))
      emit_warning(fields)
    elif re.search(r'\[(\S+)\]\s+\[(\d+)\]\s*$', line):
      emit_error({'file': "run-lint",
                  'text': ("failed to parse likely warning line: '" +
                           line + "'")})
    elif re.search(r':\d+:', line):
      emit_error({'file': "run-lint",
                  'text': ("failed to parse line with likely line number: '" +
                           line + "'")})
    else:
      print line

  # The output is done, so the command should be finished.
  assert cmd.wait() is not None
  if not saw_output:
    emit_error({'file': "run-lint", 'text': "cpplint produced no output!"})
    return False
  return cmd.returncode == 0

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description='Check source files for coding standard violations.')
  parser.add_argument('--cpplint-script', metavar='PATH_TO_SCRIPT',
                      help='The path to Google\'s cpplint.py script.',
                      default='cpplint.py')
  parser.add_argument('--cpplint-filter', '--filter', metavar='FILTER',
                      help=('The --filter argument to pass to cpplint.'))
  parser.add_argument('--no-cpplint', action='store_true',
                      help='Do not run the cpplint.py script.')
  parser.add_argument('--format',
                      help='The format string for warnings and errors.')
  parser.add_argument('--visual-studio', '--vs', action='store_true',
                      help='Use defaults tailored for Visual Studio.')
  parser.add_argument('files', metavar='FILE', nargs='*',
                      help='The file names to check.')
  args = parser.parse_args()

  if args.format is not None:
    WARNING_FORMAT = args.format
  elif args.visual_studio:
    WARNING_FORMAT = VISUAL_STUDIO_WARNING_FORMAT
  else:
    WARNING_FORMAT = STANDARD_WARNING_FORMAT

  ok = True

  if not args.no_cpplint:
    if not run_cpplint(args.cpplint_script, args.cpplint_filter, args.files):
      ok = False

  if not ok:
    sys.exit(1)
