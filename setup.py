from setuptools import find_packages, setup

from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='modbot_planner',
    packages=find_packages(include=['modbot_planner.py']),
    version='0.9.5',
    description='Library for different components for the modobot',
    long_description=long_description,
    long_description_content_type='text/markdown'
    author='Somnath Kumar',
    license='MIT',
    install_requires=[],
)