/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */





#include <Core/util_t.h>
#include <Core/array_t.h>
#include <Core/thread.h>
#include "gtk.h"
#include <sys/syscall.h>

#ifdef MT_GTK

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>
#include <GL/glut.h>
#include <X11/Xlib.h>

struct GtkThread *global_gtkThread=NULL;
Mutex callbackMutex;

void gtkEnterCallback() {  callbackMutex.lock();  }
void gtkLeaveCallback() {  callbackMutex.unlock();  }

void gtkLock(bool checkInitialized) {
  if(checkInitialized) gtkCheckInitialized();
  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
  gdk_threads_enter();
}

void gtkUnlock() {
  if(callbackMutex.state && callbackMutex.state==syscall(SYS_gettid)) return;
  gdk_threads_leave();
}

struct GtkThread:Thread {
  GtkThread():Thread("GTK thread") {}
  virtual void open() {}
  virtual void step() { gtk_main(); }
  virtual void close() { gdk_threads_leave(); }
};

void gtkCheckInitialized() {
#if 0
  static bool isInitialized=false;
  if(!isInitialized) {
    isInitialized=true;
    int argc=1;
    char **argv = new char*[1];
    argv[0] = (char*)"x.exe";
    
    XInitThreads();
    g_thread_init(NULL); 1
    gdk_threads_init();
//    gdk_threads_enter();
    gtk_init(&argc, &argv);
    gtk_gl_init(&argc, &argv);
    glutInit(&argc, argv);
  }
#else
  static Mutex m;
  if(!global_gtkThread) {
    m.lock();
    if(!global_gtkThread) {
      int argc=1;
      char **argv = new char*[1];
      argv[0] = (char*)"x.exe";
  
      XInitThreads();
//      g_thread_init(NULL);
      gdk_threads_init();
      gdk_threads_enter();
      gtk_init(&argc, &argv);
      gtk_gl_init(&argc, &argv);
      glutInit(&argc, argv);
  
      global_gtkThread = new GtkThread();
      global_gtkThread -> threadStep();
    }
    m.unlock();
  }
#endif
}




ConditionVariable menuChoice(-1);
static void menuitem_response(int choice) { menuChoice.setValue(choice); }

int gtkPopupMenuChoice(StringL& choices) {
  //create menu
  GtkWidget *menu = gtk_menu_new();
  gtk_menu_popup(GTK_MENU(menu), NULL, NULL, NULL, NULL, 0, gtk_get_current_event_time());
  for_list(MT::String,  s,  choices) {
    GtkWidget *item = gtk_menu_item_new_with_label(s->p);
    gtk_container_add(GTK_CONTAINER(menu), item);
    gtk_signal_connect_object(GTK_OBJECT(item), "activate",
                              GTK_SIGNAL_FUNC(menuitem_response), (gpointer)s_COUNT);
  }
  menuChoice.setValue(-1);
  gtk_widget_show_all(menu);
  gtk_menu_shell_select_first(GTK_MENU_SHELL(menu), false);
  menuChoice.waitForValueNotEq(-1);
  gtk_widget_destroy(menu);
  int choice = menuChoice.getValue();
  return choice>=0?choice:0;
}

GtkWidget *gtkTopWindow(const char* name) {
  gtkCheckInitialized();
  gtkLock();
  GtkWidget *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(win), name);
  gtk_window_set_default_size(GTK_WINDOW(win), 300, 300);
  //gtk_container_set_reallocate_redraws(GTK_CONTAINER(container), TRUE);
  gtkUnlock();
  return win;
}

#else //MT_GTK
#endif

