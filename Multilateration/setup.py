import setuptools
from distutils.core import setup

setup(
    name='Multilateration',
    version='0.2.0',
    author='AlexisTM',
    author_email='alexis.paques@gmail.com',
    packages=['multilateration'],
    scripts=[],
    url='https://github.com/AlexisTM/Multilateration',
    license='LICENSE.txt',
    description='Multilateration library for 2D, 3D setups and later 2D5 (3D with some axis fixed).',
    long_description=open('README.md').read(),
)
