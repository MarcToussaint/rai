# RAI bare code

This repo contains core sources related to Robotics & AI. Users are
not recommended to use this repo alone.  Please have a look at example
projects that use this bare code as a submodule and expose and explain
some subset of functionalities. Esp. the
[robotic python lib](https://github.com/MarcToussaint/robotic/), which
now co-installs C++ headers and a compiled shared lib.

'bare code' means that this repo contains only sources, a minimal
Ubuntu-specific build system, and development tests. It is mostly used
as submodule in other integrated projects, with their own
out-of-source build system.

## Brief history

Parts of the code have there origin at around 2004 (Edinburgh). The
code grew over the years to a large repo with many projects from all
lab members, but a somewhat consistent scope of code shared between
projects. This repo includes a selection of the code shared between
projects and contains a set of representations and methods for
Robotics, ML and AI. As the functionality is diverse I don't even try
to explain.

## Repos wrapping rai:

* [Robotic Python Lib](https://pypi.org/project/robotic/)
* [BotOp](https://github.com/MarcToussaint/botop)

## Documentation

The there is no proper documentation of the full rai code. I recommend starting with 
* The [robotic python lib documentation](https://marctoussaint.github.io/robotic/), which explains core features (but certainly not the underlying code base),
* With Doxygen (see [rai-maintenence help](https://github.com/MarcToussaint/rai-maintenance/tree/master/help)) you can get an API.
* The [Wiki page](../../wiki) contains an older introduction to KOMO. There is also an older KOMO tech report on arxiv: <https://arxiv.org/abs/1407.0414>
* Eventually, the [test main.cpp files](test/) help really understanding the use of the C++ code base.

## Quick Start

```
git clone git@github.com:MarcToussaint/rai.git
# OR, if you don't have a github account:
git clone https://github.com/MarcToussaint/rai.git
cd rai

# The following two commands depend on the config.mk -- see below
make -j1 printUbuntuAll    # for your information: what the next step will install
make -j1 installUbuntuAll APTGETYES=--yes # calls sudo apt-get install; remove 'yes' to allow interrupting

make -j4
make -j4 tests bin
make runTests      # compile and run the essential tests
```

## Dependencies

To change the dependencies edit the `config.mk` in `_make`:
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

