# -*- coding: utf-8 -*-
from setuptools import setup, Extension, find_packages
import os.path
import re
__author__ = 'vahid'

# reading bcm2835 version (same way sqlalchemy does)
with open(os.path.join(os.path.dirname(__file__), 'bcm2835', '__init__.py')) as v_file:
    package_version = re.compile(r".*__version__ = '(.*?)'", re.S).match(v_file.read()).group(1)

long_description = """
Raspberry Pi bcm2835 api and c extension for python
===================================================
"""

setup(
    name='bcm2835',
    version=package_version,
    author='Vahid Mardani',
    author_email='vahid.mardani@gmail.com',
    url='http://github.com/pylover/python-bcm2835',
    description='Raspberry Pi bcm2835 api and c extension for python',
    long_description=long_description,
    license='MIT',
    install_requires=[],
    packages=find_packages(),
    ext_modules=[Extension('gpio', ['bcm2835/gpio.c'])],
    # entry_points={
    #     'console_scripts': [
    #         'raspy.gpio= raspy.gpio:main'
    #     ]
    # },
    classifiers=[
        'Development Status :: 1 - Planning',
#       'Development Status :: 2 - Pre-Alpha',
#       'Development Status :: 3 - Alpha',
#       'Development Status :: 4 - Beta',
#       'Development Status :: 5 - Production/Stable',
#       'Development Status :: 6 - Mature',
#       'Development Status :: 7 - Inactive',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Intended Audience :: System Administrators',
        'Intended Audience :: End Users/Desktop',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Natural Language :: English',
        'Operating System :: POSIX',
        'Programming Language :: Python',
        'Programming Language :: C',
        'Topic :: Communications :: Email',
        'Topic :: Education',
        'Topic :: Software Development :: Libraries',
        'Topic :: Utilities',
        ],
    )
