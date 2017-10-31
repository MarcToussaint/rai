#pragma once

#include <Core/array.h>

struct Msg_MotionReference{
    arr path;
    arr tau;
    bool append=false;
};
