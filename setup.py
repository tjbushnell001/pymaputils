import setuptools

setuptools.setup(
    name="embark-pymaputils",
    version="0.0.1",
    description="Embark Python Map Utils",
    url="https://github.com/embarktrucks/pymaputils",
    packages=setuptools.find_packages('src'),
    package_dir={'': 'src'},
    classifiers=[
        "Programming Language :: Python :: 2.7",
    ],
    python_requires='>=2.7',
    install_requires=[
        "enum34==1.1.6",
        "geojson==2.0.0",
        "geopy==1.11.0",
        "numpy==1.14.2",
        "Shapely==1.7.0",
        "utm==0.4.2"
    ]
)
