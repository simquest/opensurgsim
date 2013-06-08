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

"""Runs Google's cpplint.py and some other checks on C++ files.

This script tries to flag violations of the OpenSurgSim coding
standards in C++ code.

This is certainly not a substitute for looking at the code by hand:
many violations will not be caught by the script, and some amount of
flagged code will turn out to be correct.  But it does make it easier
to notice certain common errors.
"""

# TODO(bert): add some more checks.

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
    fmt = re.sub(r'\s*\[\{category[^\}]*\}\]', '', fmt)
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

def slurp_lines(file):
  try:
    with open(file, 'r') as f:
      return f.read().splitlines()
  except IOError as e:
    print >> sys.stderr, e
    #emit_error({'file': file, 'text': str(e)})
    return None

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

def check_header_guard(flags, file):
  lines = slurp_lines(file)
  if not lines:
    emit_warning({'file': file, 'category': "opensurgsim/no_lines",
                  'text': "file is missing or empty."})
    return False

  lines = list(enumerate(lines))

  ifndef = filter(lambda x: re.search(r'^\s*#\s*ifndef\b', x[1]), lines)
  if not ifndef:
    emit_warning({'file': file, 'category': "opensurgsim/no_ifndef",
                  'text': "no #ifndef lines!"})
    return False
#  elif len(ifndef) > 1:
#    emit_warning({'file': file, 'line': ifndef[1][0],
#                  'category': "opensurgsim/several_ifndef",
#                  'text': "multiple #ifndef lines; using first."})

  guard = ifndef[0][1]
  guard = re.sub(r'//.*', '', guard)
  guard = re.sub(r'/\*.*?\*/', '', guard)
  guard_match = re.search(r'^\s*#\s*ifndef\s+(\w+)\s*$', guard)
  if not guard_match:
    emit_error({'file': file, 'line': ifndef[0][0],
                'category': "opensurgsim/internal",
                'text': "script error: failed to parse #ifndef line!"})
    return False
  guard = guard_match.group(1)

  namespace = filter(lambda x: re.search(r'^\s*namespace\b', x[1]), lines)
  if not namespace:
    emit_warning({'file': file, 'category': "opensurgsim/no_namespace",
                  'text': "no 'namespace' lines!"})
  indented_namespace = filter(lambda x: re.search(r'^\s', x[1]), namespace)
  if indented_namespace:
    emit_warning({'file': file, 'line': indented_namespace[0][0],
                  'category': "opensurgsim/namespace_indented",
                  'text': "one or more namespace declarations are indented!"})
  curly_namespace = filter(lambda x: re.search(r'{$', x[1]), namespace)
  if curly_namespace:
    emit_warning({'file': file, 'line': curly_namespace[0][0],
                  'category': "opensurgsim/namespace_brace",
                  'text': "'{' on the same line as a namespace declaration!"})
    namespace = map(lambda x: (x[0], re.sub(r'\s*{$', '', x[1])), namespace)
  namespace = map(lambda x: re.sub(r'^\s*namespace\b', '', x[1]).strip(),
                  namespace)

  namespace_re = "".join(
    map(lambda x: "(?:" + re.sub(r'[^\w\d]+', '', x.upper()) + "_)?",
        namespace))
  if len(namespace_re):
    namespace_re += r'(?<=.)'

  file_guard_re = re.sub(r'\W+', '_',
                         re.sub(r'.*/', '', file)).upper()
  guard_re = r'^' + namespace_re + file_guard_re + r'$'

  if not re.search(guard_re, guard):
    emit_warning({'file': file, 'line': ifndef[0][0],
                  'category': "opensurgsim/header_guard",
                  'text': ("unexpected guard '{}'!  expected /{}/."
                           .format(guard, guard_re)) })

  define = filter(lambda x: re.search(r'^\s*#\s*define\b', x[1]), lines)
  if not define:
    emit_warning({'file': file, 'category': "opensurgsim/no_define",
                  'text': "no #define lines!"})
  else:
    def_match = re.match(r'^\s*#\s*define\s+(\w+)\s*$',
                         re.sub(r'/\*.*?\*/', '',
                                re.sub(r'//.*', '', define[0][1])))
    if not def_match:
      emit_error({'file': file, 'line': define[0][0],
                  'category': "opensurgsim/internal",
                  'text': "script error: failed to parse #define line!"})
    elif def_match.group(1) != guard:
      emit_warning({'file': file, 'line': define[0][0],
                    'category': "opensurgsim/guard_mismatch",
                    'text': "#define doesn't match #ifndef!"})

  endif = filter(lambda x: re.search(r'^\s*#\s*endif\b', x[1]), lines)
  if not endif:
    emit_warning({'file': file, 'category': "opensurgsim/no_endif",
                  'text': "no #endif lines!"})
  elif re.search(r'^\s*#\s*endif\s*$', endif[-1][1]):
    if flags.missing_endif_comments:
      emit_warning({'file': file, 'line': endif[-1][0],
                    'category': "opensurgsim/endif_no_comment",
                    'text': "#endif with no comment."})
  else:
    if re.search(r'^\s*#\s*endif\s*///', endif[-1][1]):
      emit_warning({'file': file, 'line': endif[-1][0],
                    'category': "opensurgsim/endif_doxygen",
                    'text': "#endif with a Doxygen style /// comment."})
    endif_match = re.search(r'^\s*#\s*endif\s*//+\s*(.*?)\s*$', endif[-1][1])
    if not endif_match:
      emit_error({'file': file, 'line': endif[-1][0],
                  'category': "opensurgsim/internal",
                  'text': "script error: failed to parse #endif comment."})
    elif endif_match.group(1) != guard:
      emit_warning({'file': file, 'line': endif[-1][0],
                    'category': "opensurgsim/endif_mismatch",
                    'text': ("#endif comment doesn't match #ifndef! (" + \
                               endif_match.group(1) + ")")})

def check_guards(flags, files):
  return all(map(lambda f: check_header_guard(flags, f), files))

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description='Check source files for coding standard violations.')
  parser.add_argument('--cpplint-script', metavar='PATH_TO_SCRIPT',
                      help='The path to Google\'s cpplint.py script.',
                      default='cpplint.py')
  parser.add_argument('--cpplint-filter', '--filter', metavar='FILTER',
                      help='The --filter argument to pass to cpplint.')
  parser.add_argument('--no-cpplint', action='store_false', dest='do_cpplint',
                      help='Do not run the cpplint.py script.')
  parser.add_argument('--missing-endif-comments', action='store_true',
                      help='Flag missing include guard #endif comments, too.')
  parser.add_argument('--ignore-guards',
                      action='store_false', dest='do_check_guards',
                      help='Do not check the header file include guards.')
  parser.add_argument('--ignore-ext',
                      action='store_false', dest='do_check_extension',
                      help='Do not check the file extensions.')
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

  if args.do_check_extension:
    bad_ext = filter(lambda x: not re.search(r'\.(?:h|cpp)$', x), args.files)
    for file in bad_ext:
      emit_warning({'file': file,
                    'text': "unknown extension (expected .h or .cpp)"})
      ok = False

  if args.do_cpplint:
    if not run_cpplint(args.cpplint_script, args.cpplint_filter,
                       filter(lambda x: re.search(r'\.(?:h|cpp)$', x),
                              args.files)):
      ok = False

  if args.do_check_guards:
    if not check_guards(args,
                        filter(lambda x: re.search(r'\.h$', x), args.files)):
      ok = False

  if not ok:
    sys.exit(1)
