# pymaputils
Python Map Utilities

Centralizing Embark's Python mapping utilities to be used across 
projects and repos.

## How do I use this library?
There are several options, some of which are better or worse depending
on your use case. If you need guidance, contact Eng Ops.
- **Clone into the relevant repo as a git submodule**
    - Allows for concurrent development in `pymaputils` and parent repo.
    - Requires redundant PR workflow (i.e. in `pymaputils` and
      in parent repo(s)).
    - Allows for testing flexibility.
    - Keeps parent repo source up to date, but may be less stable.
- **Pack into a pip-installable .whl file**
    - Easy to use in Databricks.
    - Automatic dependency management. (Thanks `install_requires`!)
    - Source isn't changing; user knows what to expect.
    - No accessibility of raw source code.    
- **Copy source directly (not recommended)**
    - Don't need to hassle with any setup; possibly convenient for quick
      prototyping/scripting.
    - Assumes changes will NOT be made to source and/or any changes made
      will NOT need to persist.

## Setup & Dependencies

#### Usage as a submodule
Ensure that `pymaputils` is not already a submodule in the target repo. 

Then use `git submodule add <github-url>` to add to an existing repo.

When used as a submodule, `pymaputils` should generally be on 
`master` branch.  

See [submodule doc](https://git-scm.com/docs/git-submodule) 
and [examples](https://git-scm.com/book/en/v2/Git-Tools-Submodules).

User is responsible for managing dependencies and paths. 

User is responsible for ensuring changes are compatible with 
all repos using `pymaputils` as a submodule and that all relevant 
repos with submodules are updated. Contact Eng Ops if you need guidance.

#### Usage as a standalone package
`cd` to `pymaputils` root directory.

Generate into a wheel package with:
```
python setup.py bdist_wheel
```

This will install .whl file to 
`path/to/pymaputils/dist/<name-of-generated-wheel.whl`.

You may need to copy the wheel to a more appropriate directory.

Install into desired environment with the following:

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
From outside a container (you must have pipenv installed):
```
cd test  # must be in test dir
./run_tests
```
From inside a container (with the necessary dependencies):
```
# from top level dir or test dir
python -m unittest discover
```
If you are running tests for the purpose of testing your
changes locally, you should generally be running the tests from 
inside the relevant container. An exception to this may be making
changes to `pymaputils` that are oblivious to any outside code. 

We use `pipenv` to manage a lightweight reproducible environment 
that is the same locally and for automated CI tests.

See [pipenv docs](https://pipenv.pypa.io/en/latest/) and/or 
[pipenv github](https://github.com/pypa/pipenv).

## A Note on Versioning and Tags
`pymaputils` uses version tags primarily as a bookkeeping device to 
track batched changes (rather than using PR numbers and/or commit hashes). 
They should correlate with improved stability and additional functionality, 
but that is not their primary purpose (for now).
