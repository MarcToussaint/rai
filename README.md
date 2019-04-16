# RAI bare code

This repo contains core sources related to Robotic AI. First users are not recommended to use this repo alone.
Please have a look at example projects that use this bare code as a submodule and expose and explain one
particular functionality. The first example projects to come are: KOMO & LGP.

'bare code' means that this repo contains only sources and a minimal Ubuntu-specific build system, but no examples
and only minimal tests. Embedding projects provide examples and more thorough testing. For integration in other
projects, use it as a submodule and integrate it in your own out-of-source build system.

## Brief history

Parts of the code have there origin at around 2004 (Edinburgh). The code grew over the years to a large repo with
many projects from all lab members, but a somewhat consistent scope of code shared between projects. This repo exports a
selection of the code shared between projects and contains a set of representations and methods for Robotics, ML and AI.
As the functionality is diverse I don't even try to explain. The example projects will hopefully demonstrate it.

## Quick Start

```
git clone git@github.com:MarcToussaint/rai.git
# OR, if you don't have a github account:
git clone https://github.com/MarcToussaint/rai.git
cd rai

# OPTIONAL! Often you will only need to compile some components and don't need all Ubuntu packages
make -j1 printUbuntuAll    # to just print Ubuntu package dependencies per component
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt

make -j4
make runTests      # compile and run the essential tests
make bin           # compile rai/bin/kinEdit and similar
```

To change the dependencies:
```
cp build/config.mk.default config.mk
```
Then edit the `config.mk` in the root directory. After this you definitely need to recompile some components. In doubt
```
make cleanAll
make -j4
```
