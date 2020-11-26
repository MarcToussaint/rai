# RAI bare code

This repo contains core sources related to Robotic AI. First users are
not recommended to use this repo alone.  Please have a look at example
projects that use this bare code as a submodule and expose and explain
one particular functionality. E.g., KOMO or rai-python.

'bare code' means that this repo contains only sources, a minimal
Ubuntu-specific build system, and only minimal tests. For integration
in other projects, use it as a submodule and integrate it in your own
out-of-source build system.

## Brief history

Parts of the code have there origin at around 2004 (Edinburgh). The
code grew over the years to a large repo with many projects from all
lab members, but a somewhat consistent scope of code shared between
projects. This repo exports a selection of the code shared between
projects and contains a set of representations and methods for
Robotics, ML and AI. As the functionality is diverse I don't even try
to explain.

## Documentation

I only recently started better documentation of some parts. So far, bits and pieces of documentation are scattered:
* The [Wiki page](../../wiki) contains an introduction to KOMO. There is also an older KOMO tech report on arxiv: <https://arxiv.org/abs/1407.0414>
* The [rai-maintenence help](https://github.com/MarcToussaint/rai-maintenance/tree/master/help) contains various info
* The [rai-python Jupyter notebooks](https://github.com/MarcToussaint/rai-python/tree/master/docs) also help understanding the underlying code
* The [test main.cpp files](test/) might help as well

## Quick Start

```
git clone git@github.com:MarcToussaint/rai.git
# OR, if you don't have a github account:
git clone https://github.com/MarcToussaint/rai.git
cd rai

# The following two commands depend on the config.mk -- see below
make -j1 printUbuntuAll    # for your information: what the next step will install
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt

make -j4
make -j4 tests bin
make runTests      # compile and run the essential tests

# when interested in the python bindings:
pip3 install --user pybind11 jupyter nbconvert matplotlib
```

## Dependencies

To change the dependencies edit the `config.mk` in the root directory:
When a flag is set =0, this forces that this package is not
used. Otherwise (when set =0 is commented), a sub-folder Makefile may
set it equal to 1 and links to this package. After this you definitely
need to recompile some components. In doubt
```
make cleanAll
make -j4
```

If you pull an update, it might help to create Makefile.dep files
throught the project using
```
make dependAll
make -j4
```

## Maintenance & Alternative Build Systems

The rai repo has its own old-fashioned GNU make build system, which is convenient, modular and flexible for me, and compiles each source directory into its own shared lib. But I expect serious users to replace this by their own out-of-source build system.

In [rai-maintenence](https://github.com/MarcToussaint/rai-maintenance/tree/master/buildAlternatives) there is a minimalistic CMake example, showing how to compile selected sources into a single shared lib.

In [rai-maintenence](https://github.com/MarcToussaint/rai-maintenance/tree/master/docker) there are also examples for testing in docker (Ubuntu 18.04 and 16.04).

To install globally: (default path is ~/z.LOCAL)
```
sudo make install INSTALL_PATH=/usr/local
```
