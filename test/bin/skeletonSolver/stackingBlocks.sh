#!/bin/bash

BIN=../../../bin

$BIN/skeletonSolver \
    -confFile '"../rai-robotModels/scenarios/workshopTable.g"' \
    -sktFile stackingBlocks.skt \
    -mode sequence \
    -collisions true
