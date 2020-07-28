import os
import setuptools


# Utility function to read a file at same level as this one.
def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


def read_requirements():
    return read('requirements.txt').split('\n')


setuptools.setup(
    name="embark-pymaputils",
    version="0.0.5",
    description="Embark Python Map Utils",
    url="https://github.com/embarktrucks/pymaputils",
    packages=setuptools.find_packages('src'),
    package_dir={'': 'src'},
    classifiers=[
        "Programming Language :: Python :: 2.7",
    ],
    python_requires='>=2.7',
    install_requires=read_requirements()
)
