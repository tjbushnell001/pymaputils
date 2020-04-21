# pymaputils
Python Map Utilities

## Setup & Dependencies
See `requirements.txt`.

#### Usage as a submodule
Use `git submodule add <github-url>` to add to an existing repo. 
User is responsible for managing dependencies and paths.

See [submodule doc](https://git-scm.com/docs/git-submodule) 
and [examples](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

#### Usage as a standalone package
Generate into a package with:
```
python setup.py sdist bdist_wheel
```

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
