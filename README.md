# pymaputils

## Setup & Dependencies
See `test/Pipfile`.

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