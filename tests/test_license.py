#!/usr/bin/env python

# Copyright 1996-2020 Cyberbotics Ltd.
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

"""Test that checks that all the source files have the Apache 2 license."""

import unittest
import datetime
import os
import fnmatch

from io import open

APACHE2_LICENSE_C = """/*
 * Copyright 1996-20XX Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */""".replace('20XX', str(datetime.datetime.now().year))

APACHE2_LICENSE_CPP = """// Copyright 1996-20XX Cyberbotics Ltd.
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
// limitations under the License.""".replace('20XX', str(datetime.datetime.now().year))

APACHE2_LICENSE_PYTHON = """# Copyright 1996-20XX Cyberbotics Ltd.
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
# limitations under the License.""".replace('20XX', str(datetime.datetime.now().year))

PYTHON_OPTIONAL_HEADER = """#!/usr/bin/env python

"""
PYTHON3_OPTIONAL_HEADER = """#!/usr/bin/env python3

"""


class TestLicense(unittest.TestCase):
    """Unit test for checking that all the source files have the Apache 2 license."""

    def setUp(self):
        """Get all the source files which require a license check."""

        skippedDirectoryPaths = [
        ]
        skippedFilePaths = [
        ]
        skippedDirectories = [
            'build',
            'src-gen'
        ]

        extensions = ['*.c', '*.cpp', '*.cc', '*.h', '*.hpp', '*.hh', '*.py']

        self.repoRootPath = os.path.abspath(os.path.join(os.path.realpath(__file__), '..', '..'))

        self.sources = []
        for rootPath, dirNames, fileNames in os.walk(self.repoRootPath):
            shouldContinue = False
            for path in skippedDirectoryPaths:
                if rootPath.startswith(self.repoRootPath + os.sep + path.replace('/', os.sep)):
                    shouldContinue = True
                    break
            currentDirectories = rootPath.replace(self.repoRootPath, '').split(os.sep)
            for directory in skippedDirectories:
                if directory in currentDirectories:
                    shouldContinue = True
                    break
            if fileNames == '__init__.py':
                shouldContinue = True
            if shouldContinue:
                continue
            for extension in extensions:
                for fileName in fnmatch.filter(fileNames, extension):
                    if os.path.join(self.repoRootPath, fileName).replace(os.sep, '/') in skippedFilePaths:
                        continue
                    file = os.path.join(rootPath, fileName)
                    self.sources.append(file)

    def test_sources_have_license(self):
        """Test that sources have the license."""
        for source in self.sources:
            with open(source, 'r', encoding='utf-8') as content_file:
                content = content_file.read()
                if source.endswith('.c') or source.endswith('.h'):
                    self.assertTrue(
                        content.startswith(APACHE2_LICENSE_C),
                        msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' %
                            (source, APACHE2_LICENSE_C)
                    )
                elif source.endswith('.cpp') or source.endswith('.hpp') or source.endswith('.cc') or source.endswith('.hh') or source.endswith('.java'):
                    self.assertTrue(
                        content.startswith(APACHE2_LICENSE_CPP),
                        msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' %
                            (source, APACHE2_LICENSE_CPP)
                    )
                elif source.endswith('.py') or source.endswith('Makefile'):
                    self.assertTrue(
                        content.startswith(APACHE2_LICENSE_PYTHON) or
                        content.startswith(PYTHON_OPTIONAL_HEADER + APACHE2_LICENSE_PYTHON) or
                        content.startswith(PYTHON3_OPTIONAL_HEADER + APACHE2_LICENSE_PYTHON),
                        msg='Source file "%s" doesn\'t contain the correct Apache 2.0 License:\n%s' %
                            (source, APACHE2_LICENSE_PYTHON)
                    )
                else:
                    self.assertTrue(
                        False,
                        msg='Unsupported file extension "%s".' % source
                    )


if __name__ == '__main__':
    unittest.main()