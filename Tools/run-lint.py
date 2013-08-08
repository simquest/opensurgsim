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
import os
from fix_header_guards import get_expected_header_guard

STANDARD_WARNING_FORMAT = \
    "{file}:{line}:{col}: {text} [{category}] [{level}]"
VISUAL_STUDIO_WARNING_FORMAT = \
    "{file}({line},{col}): {warning} {vscategory}[{level}]: {text}"
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
  if 'col' not in fields:
    fmt = re.sub(r'\s*\{col[^\}]*\}:', '', fmt)
    fmt = re.sub(r'\s*,\{col[^\}]*\}\)', ')', fmt)
    fmt = re.sub(r'\s*\{col[^\}]*\}', '', fmt)
    fields['col'] = '???'  # panic button
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

def slurp_numbered_lines(file):
  try:
    with open(file, 'r') as f:
      lines = list(enumerate(f.read().splitlines(), start=1))
      if not lines:
        emit_warning({'file': file, 'category': "opensurgsim/no_lines",
                      'text': "file is empty."})
      return lines
  except IOError as e:
    print >> sys.stderr, e
    emit_error({'file': file, 'category': "opensurgsim/no_file",
                'text': "file is missing or could not be opened."})
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

def check_header_guard(flags, file, lines):
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

  guard_expected = get_expected_header_guard(file)

  if guard != guard_expected:
    emit_warning({'file': file, 'line': ifndef[0][0],
                  'category': "opensurgsim/header_guard",
                  'text': ("unexpected guard '{}'!  expected '{}'."
                           .format(guard, guard_expected)) })

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

def find_column_char(text, column, tab_width=4):
  """Find the character that corresponds to the specified column.

  In the absence of tabs in the text, the return value will be equal
  to the column.  When tabs are present, it will be smaller.

  """
  text = text[:column]
  while len(text.expandtabs(tab_width)) > column:
    text = text[:(len(text)-1)]
  return len(text)

def get_column(flags, text, column):
  if flags.visual_studio:
    return find_column_char(text, column)
  else:
    return column

def check_length(flags, file, lines):
  xlines = map(lambda x: (x[0], re.sub(r'\r?\n$', '', x[1].expandtabs(4))),
               lines)
  for bad in filter(lambda x: len(x[1]) > flags.max_line_length, xlines):
    col = get_column(flags, lines[bad[0]-1][1], flags.max_line_length)
    emit_warning({'file': file, 'line': bad[0], 'col': col,
                  'category': "opensurgsim/too_long",
                  'text': ("the line is {} characters long, which is longer"
                           " than {}!"
                           .format(len(bad[1]), flags.max_line_length)) })

def get_listed_files(flags, file, lines):
  flines = filter(lambda x: re.search(r'^\s.*\.(h|cpp)\s*$', x[1]), lines)
  for f in flines:
    if not re.search(r'^\t\S', f[1]):
      emit_warning({'file': file, 'line': f[0],
                    'category': "opensurgsim/cmake_file_indent",
                    'text': "file name indentation is not a single tab."})
    if re.search(r'\s$', f[1]):
      emit_warning({'file': file, 'line': f[0],
                    'category': "opensurgsim/cmake_file_trailing",
                    'text': "file name is followed by trailing whitespace."})
  files = map(lambda x: (x[0], x[1].strip()), flines)
  for f in files:
    if re.search(r'\$', f[1]):
      emit_warning({'file': file, 'line': f[0],
                    'category': "opensurgsim/cmake_file_variable",
                    'text': "file name contains a variable!"})
    if re.search(r'\s', f[1]):
      emit_warning({'file': file, 'line': f[0],
                    'category': "opensurgsim/cmake_file_whitespace",
                    'text': "file name contains whitespace!"})
  dir_path = os.path.dirname(file)
  files = map(lambda x: (x[0], os.path.normpath(os.path.join(dir_path, x[1]))),
              files)
  for f in files:
    if not os.path.exists(f[1]):
      emit_warning({'file': file, 'line': f[0],
                    'category': "opensurgsim/cmake_missing",
                    'text': ("file " + f[1] +
                             " does not exist but is listed in CMakeLists!") })
  return map(lambda x: x[1], files)

def find_responsible_cmakelists(file):
  dir_path = os.path.dirname(file)
  while True:
    guess = os.path.join(dir_path, 'CMakeLists.txt')
    if os.path.exists(guess):
      return guess
    parent = os.path.dirname(dir_path)
    if parent == dir_path:
      # uh-oh, we didn't find ANY CMakeLists.txt file?!?
      return file
    dir_path = parent

def check_file_lists(cmakelists_files, found_files):
  while cmakelists_files or found_files:
    if (cmakelists_files and found_files and
        cmakelists_files[0] == found_files[0]):
      # lists match.  remove matching elements and keep going.
      del cmakelists_files[0]
      del found_files[0]
    elif not found_files or cmakelists_files[0] < found_files[0]:
      # in CMakeLists but not on disk.  should already have been handled.
      del cmakelists_files[0]
    elif not cmakelists_files or found_files[0] < cmakelists_files[0]:
      # in the list but not in CMakeLists.
      emit_warning({'file': find_responsible_cmakelists(found_files[0]),
                    'category': "opensurgsim/cmake_missing",
                    'text': ("file " + found_files[0] +
                             " not present in CMakeLists.txt!") })
      del found_files[0]
    else:
      assert False, "should not have reached here"


if __name__ == '__main__':
  parser = argparse.ArgumentParser(
    description='Check source files for coding standard violations.')
  parser.add_argument('--traverse', metavar='DIR', action='append',
                      help='Find all applicable files in DIR and its subdirs.')
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
  parser.add_argument('--max-line-length', type=int, default=120,
                      help='Maximum allowed line length for C++ code.')
  parser.add_argument('--ignore-length',
                      action='store_false', dest='do_check_length',
                      help='Do not check the line length.')
  parser.add_argument('--ignore-file-lists',
                      action='store_false', dest='do_check_file_lists',
                      help='Do not check the files in CMakeLists.txt.')
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

  if args.traverse is not None:
    for traverse_dir in args.traverse:
      for current_dir, subdirs, files in os.walk(traverse_dir):
        args.files.extend(map(lambda x: os.path.join(current_dir, x),
                              filter(lambda x: re.search(r'\.(?:h|cpp)$', x),
                                     files)))
        if 'CMakeLists.txt' in files:
          args.files.append(os.path.join(current_dir, 'CMakeLists.txt'))
        # always skip .git, build directories, ThirdParty...
        for bad in filter(lambda x:
                            re.search(r'^(?:\.git|build.*|ThirdParty)$', x),
                          subdirs):
          subdirs.remove(bad)

  ok = True

  if args.do_check_extension:
    bad_ext = filter(lambda x:
                       not re.search(r'\.(?:h|cpp)$|CMakeLists\.txt$', x),
                     args.files)
    for file in bad_ext:
      emit_warning({'file': file,
                    'text': "unknown extension (expected .h or .cpp)"})
      ok = False

  if args.do_cpplint:
    # TODO(advornik): Eventually, the file list will grow so long that
    # we start running into the OS arg list limits, and we will need
    # to start splitting it into several separate invocations of
    # cpplint.  But we don't have that problem yet.
    if not run_cpplint(args.cpplint_script, args.cpplint_filter,
                       filter(lambda x: re.search(r'\.(?:h|cpp)$', x),
                              args.files)):
      ok = False

  cmakelists_files = None
  for file in args.files:
    lines = None

    if args.do_check_guards and re.search(r'\.h$', file):
      if lines is None:
        lines = slurp_numbered_lines(file)
        if not lines:
          continue
      if not check_header_guard(args, file, lines):
        ok = False

    if args.do_check_length and re.search(r'\.(h|cpp)$', file):
      if lines is None:
        lines = slurp_numbered_lines(file)
        if not lines:
          continue
      if not check_length(args, file, lines):
        ok = False

    if args.do_check_file_lists and re.search(r'CMakeLists\.txt$', file):
      if lines is None:
        lines = slurp_numbered_lines(file)
        if not lines:
          continue
        if cmakelists_files is None:
            cmakelists_files = []
        cmakelists_files.extend(get_listed_files(args, file, lines))
        # cmakelists_files will be checked later...

  if args.do_check_file_lists and cmakelists_files is not None:
    cmakelists_files = sorted(cmakelists_files)
    found_files = sorted(map(lambda x: os.path.normpath(x),
                              filter(lambda x: re.search(r'\.(?:h|cpp)$', x),
                                     args.files)))
    check_file_lists(cmakelists_files, found_files)


#  if not ok:
#    sys.exit(1)
