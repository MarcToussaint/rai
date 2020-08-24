/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#pragma once

#include "../Core/array.h"

typedef struct _GtkWidget GtkWidget;

void gtkLock(bool checkInitialized=true);
void gtkUnlock();
void gtkCheckInitialized();
void gtkEnterCallback();
void gtkLeaveCallback();

int gtkPopupMenuChoice(StringL& choices);
GtkWidget* gtkTopWindow(const char* title);

void gtkProcessEvents();
