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
