import setuptools

setuptools.setup(
    name="embark-pymaputils",
    version="0.0.1",
    description="Embark Python mapp utils",
    url="https://github.com/embarktrucks/pymaputils",
    packages=setuptools.find_packages('src', exclude=['scripts']),
    package_dir={'': 'src'},
    classifiers=[
        "Programming Language :: Python :: 2",
        "Operating System :: OS Independent",
    ],
    python_requires='>=2.7',
    # TODO gather requirements and add install_requires
)
