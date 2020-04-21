# pymaputils
Python Map Utilities

## Setup & Dependencies

#### Usage as a submodule
Use `git submodule add <github-url>` to add to an existing repo. 
User is responsible for managing dependencies and paths.

See [submodule doc](https://git-scm.com/docs/git-submodule) 
and [examples](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

#### Usage as a standalone package
Generate into a wheel package with:
```
python setup.py bdist_wheel
```

Then install into desired environment with:
```
pip install <path/to/wheel/name-of-generated-wheel.whl>
```

Hint: In databricks, consider using:
```
dbutils.library.install('dbfs:path/tp/wheel/name-of-generated-wheel.whl')
```

#### Dependencies
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
