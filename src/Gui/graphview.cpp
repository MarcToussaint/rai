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
#include <Gui/gtk.h>

#if defined MT_GTK and defined MT_GRAPHVIZ

#include <gtk/gtk.h>
// #define WITH_CGRAPH
#include <graphviz/gvc.h>
#include <graphviz/gvplugin_device.h>
#undef MIN
#undef MAX

#include "graphview.h"

#define INFO(x) //printf("CALLBACK: %s\n",#x);

extern "C" {
  GVJ_t *gvjobs_first(GVC_t * gvc);
}

struct sGraphView {
  KeyValueGraph *G;
  GraphView *p;
  MT::String title;
  
  // on gtk side
  GtkWidget *drawingarea,*container;
  
  // on graphviz side
  graph_t *gvGraph;
  MT::Array<Agnode_t *> gvNodes;
  GVC_t *gvContext;
  GVJ_t *gvJob() { return gvjobs_first(gvContext); }
  
  void init();
  void updateGraphvizGraph();
  
  static bool on_drawingarea_expose_event(GtkWidget *widget,    GdkEventExpose  *event,    gpointer     user_data);
  static bool on_drawingarea_motion_notify_event(GtkWidget *widget,       GdkEventMotion  *event,       gpointer     user_data);
  static bool on_container_delete_event(GtkWidget *widget,   GdkEvent    *event,   gpointer     user_data);
  static bool on_drawingarea_configure_event(GtkWidget *widget,   GdkEventConfigure *event,   gpointer     user_data);
  static bool on_drawingarea_button_press_event(GtkWidget *widget,      GdkEventButton  *event,      gpointer     user_data);
  static bool on_drawingarea_button_release_event(GtkWidget *widget,    GdkEventButton  *event,    gpointer     user_data);
  static bool on_drawingarea_scroll_event(GtkWidget   *widget, GdkEventScroll    *event,    gpointer     user_data);
  
};

GraphView::GraphView(KeyValueGraph& G, const char* title, void *container) {
  gtkCheckInitialized();
  
  s = new sGraphView;
  s->p=this;
  s->title=title;
  s->container=GTK_WIDGET(container);
  s->G = &G;
  s->init();
}

GraphView::~GraphView() {
  delete s;
}


void GraphView::update() {
  gtkLock();
  s->updateGraphvizGraph();
  gvLayoutJobs(s->gvContext, s->gvGraph);
  gvRenderJobs(s->gvContext, s->gvGraph);
  gtkUnlock();
}

void GraphView::watch() {
  update();
if(MT::getInteractivity()){
  gtk_main();
}
}

#define STR(s) (char*)s

MT::String label(Item *it){
  MT::String label;
  if(it->keys.N) {
    label <<it->keys(0);
    for(uint j=1; j<it->keys.N; j++) label <<'\n' <<it->keys(j);
  } else {
    label <<'-';
  }
  return label;
}

void sGraphView::updateGraphvizGraph() {
//  aginit();
  gvGraph = agopen(STR("new_graph"), Agdirected, NULL);
  agattr(gvGraph, AGRAPH,STR("rankdir"), STR("LR"));
  agattr(gvGraph, AGRAPH,STR("ranksep"), STR("0.05"));
  
  agattr(gvGraph, AGNODE, STR("label"), STR(""));
  agattr(gvGraph, AGNODE, STR("shape"), STR(""));
  agattr(gvGraph, AGNODE, STR("fontsize"), STR("11"));
  agattr(gvGraph, AGNODE, STR("width"), STR(".3"));
  agattr(gvGraph, AGNODE, STR("height"), STR(".3"));
  
  agattr(gvGraph, AGEDGE, STR("label"), STR(""));
  agattr(gvGraph, AGEDGE, STR("arrowhead"), STR("none"));
  agattr(gvGraph, AGEDGE, STR("arrowsize"), STR(".5"));
  agattr(gvGraph, AGEDGE, STR("fontsize"), STR("6"));
  
  gvNodes.resize(G->N);
  
  //first add `nodes' (items without links)
  for_list(Item,  e,  (*G)) {
    e->index=e_COUNT;
    CHECK(e_COUNT==e->index,"");
    gvNodes(e_COUNT) = agnode(gvGraph, STRING(e->index <<'_' <<label(e)), true);
    if(e->keys.N) agset(gvNodes(e_COUNT), STR("label"), label(e));
    if(e->parents.N) {
      agset(gvNodes(e_COUNT), STR("shape"), STR("box"));
      agset(gvNodes(e_COUNT), STR("fontsize"), STR("6"));
      agset(gvNodes(e_COUNT), STR("width"), STR(".1"));
      agset(gvNodes(e_COUNT), STR("height"), STR(".1"));
    } else {
      agset(gvNodes(e_COUNT), STR("shape"), STR("ellipse"));
    }
  }
  
  //now all others
  { for_list(Item, e, (*G)) {
    /*if(e->parents.N==2){ //is an edge
      gvNodes(i) = (Agnode_t*)agedge(gvGraph, gvNodes(e->parents(0)->id), gvNodes(e->parents(1)->id)); //, STRING(i <<"_" <<e->name), true);
    }else*/ if(e->parents.N) {
      for_list(Item, n, e->parents) {
        Agedge_t *ge;
        if(n->index<e->index)
          ge=agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), STRING(label(n) <<"--" <<label(e)), true);
        else
          ge=agedge(gvGraph, gvNodes(e->index), gvNodes(n->index), STRING(label(e) <<"--" <<label(n)), true);
        agset(ge, STR("label"), STRING(e_COUNT));
      }
    }
    }}
  
  cout <<gvNodes <<endl;
}

void sGraphView::init() {
  gtkLock();
  gvContext = ::gvContext();
  char *bla[] = {STR("dot"), STR("-Tx11"), NULL};
  gvParseArgs(gvContext, 2, bla);
  
  if(!container) {
    container = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    g_object_set_data(G_OBJECT(container), "GraphvizGtk", (gpointer) this);
    gtk_window_set_title(GTK_WINDOW(container), title);
  }
  
  drawingarea = gtk_drawing_area_new();
  g_object_set_data(G_OBJECT(drawingarea), "GraphvizGtk", (gpointer) this);
  gtk_widget_show(drawingarea);
  gtk_container_add(GTK_CONTAINER(container), drawingarea);
  gtk_widget_set_size_request(drawingarea, 300, 300);
  gtk_widget_set_events(drawingarea, GDK_EXPOSURE_MASK | GDK_POINTER_MOTION_MASK | GDK_BUTTON_MOTION_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_ENTER_NOTIFY_MASK | GDK_LEAVE_NOTIFY_MASK);
  
  g_signal_connect((gpointer) container, "delete_event",  G_CALLBACK(on_container_delete_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "expose_event",  G_CALLBACK(on_drawingarea_expose_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "motion_notify_event",  G_CALLBACK(on_drawingarea_motion_notify_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "configure_event",  G_CALLBACK(on_drawingarea_configure_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "button_press_event",  G_CALLBACK(on_drawingarea_button_press_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "button_release_event",  G_CALLBACK(on_drawingarea_button_release_event),  NULL);
  g_signal_connect((gpointer) drawingarea, "scroll_event",  G_CALLBACK(on_drawingarea_scroll_event),  NULL);
  
  gtk_widget_show(container);
  gtkUnlock();
}

bool sGraphView::on_drawingarea_expose_event(GtkWidget *widget, GdkEventExpose  *event, gpointer user_data) {
  sGraphView *gv;
  GVJ_t *job;
  cairo_t *cr;
  
  INFO(on_drawingarea_expose_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  cr = gdk_cairo_create(gtk_widget_get_window(widget));
  
  job->context = (void*)cr;
  job->external_context = TRUE;
  job->width = widget->allocation.width; //gtk_widget_get_allocated_width(widget);
  job->height = widget->allocation.height; //gtk_widget_get_allocated_height(widget);
  if(job->has_been_rendered)
    (job->callbacks->refresh)(job);
  else
    (job->callbacks->refresh)(job);
    
  cairo_destroy(cr);
  
  if(job->current_obj) {
    if(agobjkind(job->current_obj)==AGNODE || agobjkind(job->current_obj)==AGEDGE) {
      int i=gv->gvNodes.findValue((Agnode_t*)job->current_obj);
      if(i<0) {
        MT_MSG("???");
      } else {
        cout <<"current object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  if(job->selected_obj) {
    if(agobjkind(job->selected_obj)==AGNODE) {
      int i=gv->gvNodes.findValue((Agnode_t*)job->selected_obj);
      if(i<0) {
        MT_MSG("???");
      } else {
      
        cout <<"selected object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  
  return FALSE;
}


bool sGraphView::on_drawingarea_motion_notify_event(GtkWidget       *widget,                   GdkEventMotion  *event,                   gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  //pointf pointer;
  
  INFO(on_drawingarea_motion_notify_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  job->pointer.x = event->x;
  job->pointer.y = event->y;
  (job->callbacks->motion)(job, job->pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

bool sGraphView::on_container_delete_event(GtkWidget       *widget,       GdkEvent        *event,       gpointer         user_data) {

  INFO(on_container_delete_event);
  
  gtk_main_quit();
  return FALSE;
}


bool sGraphView::on_drawingarea_configure_event(GtkWidget       *widget,               GdkEventConfigure *event,               gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  double zoom_to_fit;
  
  INFO(on_drawingarea_configure_event);
  
  /*FIXME - should allow for margins */
  /*      - similar zoom_to_fit code exists in: */
  /*      plugin/gtk/callbacks.c */
  /*      plugin/xlib/gvdevice_xlib.c */
  /*      lib/gvc/gvevent.c */
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
//  if(!job->has_been_rendered) {
//    zoom_to_fit = 1.0;
//    MT::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
//    if(zoom_to_fit < 1.0)  /* don't make bigger */
//      job->zoom *= zoom_to_fit;
//  } else if(job->fit_mode) {
    zoom_to_fit = MT::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
    job->zoom *= zoom_to_fit;
//  }
  if(event->width > (int)job->width || event->height > (int)job->height)
    job->has_grown = TRUE;
  job->width = event->width;
  job->height = event->height;
  job->needs_refresh = TRUE;
  
  return FALSE;
}


bool sGraphView::on_drawingarea_button_press_event(GtkWidget       *widget,                  GdkEventButton  *event,                  gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_button_press_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_press)(job, event->button, pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

bool sGraphView::on_drawingarea_button_release_event(GtkWidget       *widget,                    GdkEventButton  *event,                    gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_button_release_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_release)(job, event->button, pointer);
  
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}


bool sGraphView::on_drawingarea_scroll_event(GtkWidget       *widget,            GdkEventScroll        *event,            gpointer         user_data) {
  sGraphView *gv;
  GVJ_t *job;
  pointf pointer;
  
  INFO(on_drawingarea_scroll_event);
  
  gv = (sGraphView*)g_object_get_data(G_OBJECT(widget),"GraphvizGtk");
  job = gv->gvJob();
  if(!job) return false;
  pointer.x = event->x;
  pointer.y = event->y;
  switch(((GdkEventScroll *)event)->direction) {
    case GDK_SCROLL_UP:
      (job->callbacks->button_press)(job, 4, pointer);
      break;
    case GDK_SCROLL_DOWN:
      (job->callbacks->button_press)(job, 5, pointer);
      break;
    case GDK_SCROLL_LEFT:
    case GDK_SCROLL_RIGHT:
      break;
  }
  gtk_widget_queue_draw(widget);
  
  return FALSE;
}

#undef STR

#else //defined MT_GTK and defined MT_GRAPHVIZ

#include "graphview.h"
GraphView::GraphView(KeyValueGraph& G, const char* title, void *container) { NICO }
GraphView::~GraphView() { NICO }
void GraphView::watch() { NICO }
void GraphView::update() { NICO }

#endif


//===========================================================================
//
// explicit instantiations
//

#if defined MT_GTK and defined MT_GRAPHVIZ
template MT::Array<Agnode_t*>::Array();
template MT::Array<Agnode_t*>::~Array();
#endif
