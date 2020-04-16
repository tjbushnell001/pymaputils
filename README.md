# pymaputils
Python Map Utilities

Generate into a package with:
```
python setup.py sdist bdist_wheel
```

## Setup & Dependencies
See `requirements.txt`.

## Testing
From outside a container:
```
cd test  # must be in test dir
./run_tests
```
From inside a container (with the necessary dependencies):
```
# from top level dir or test dir
python -m unittest discover
```
