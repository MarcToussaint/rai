/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#include "graphview.h"
#include "../Core/util.ipp"
#include "../Core/array.ipp"
#include "../Gui/gtk.h"

#if defined RAI_GTK and defined RAI_GRAPHVIZ

#include <graphviz/graphviz_version.h>
#if defined PACKAGE_URL //check the graphviz version (awkward...)

#include <gtk/gtk.h>
#include <graphviz/gvc.h>
#include <graphviz/gvplugin_device.h>
#undef MIN
#undef MAX

#define INFO(x) if(gv->p->verbose) printf("CALLBACK: %s\n",#x);

extern "C" {
  GVJ_t* gvjobs_first(GVC_t* gvc);
}

struct sGraphView {
  Graph* G;
  GraphView* p;
  rai::String title;

  // on gtk side
  GtkWidget* drawingarea, *container;

  // on graphviz side
  graph_t* gvGraph;
  rai::Array<Agnode_t*> gvNodes;
  GVC_t* gv_context;
  GVJ_t* gvJob() { return gvjobs_first(gv_context); }

  sGraphView():gvGraph(nullptr) {}
  void init();
  void updateGraphvizGraph(bool isSubGraph=false);
  void writeFile(const char* filename);

  static bool on_drawingarea_expose_event(GtkWidget* widget,    GdkEventExpose*  event,    gpointer     user_data);
  static bool on_drawingarea_motion_notify_event(GtkWidget* widget,       GdkEventMotion*  event,       gpointer     user_data);
  static bool on_container_delete_event(GtkWidget* widget,   GdkEvent*    event,   gpointer     user_data);
  static bool on_drawingarea_configure_event(GtkWidget* widget,   GdkEventConfigure* event,   gpointer     user_data);
  static bool on_drawingarea_button_press_event(GtkWidget* widget,      GdkEventButton*  event,      gpointer     user_data);
  static bool on_drawingarea_button_release_event(GtkWidget* widget,    GdkEventButton*  event,    gpointer     user_data);
  static bool on_drawingarea_scroll_event(GtkWidget*   widget, GdkEventScroll*    event,    gpointer     user_data);
};

GraphView::GraphView(Graph& G, const char* title, void* container)
  : verbose(false) {
  gtkCheckInitialized();

  self = make_unique<sGraphView>();
  self->p=this;
  self->title=title;
  self->container=GTK_WIDGET(container);
  self->G = &G;
  self->init();
}

GraphView::~GraphView() {
}

void GraphView::update() {
  gtkLock();
  self->updateGraphvizGraph();
  gvLayoutJobs(self->gv_context, self->gvGraph);
  gvRenderJobs(self->gv_context, self->gvGraph);
  gtkUnlock();
  gtkProcessEvents();
}

void GraphView::watch() {
  update();
  if(rai::getInteractivity()) {
    gtk_main();
  }
}

void GraphView::writeFile(const char* filename) {
  self->writeFile(filename);
}

#define STR(s) (char*)s

rai::String label(Node* it) {
  rai::String label;
#if 1
  if(it->keys.N) {
    label <<it->keys(0);
    for(uint j=1; j<it->keys.N; j++) label <<'\n' <<it->keys(j);
  }
  if(it->keys.N) label <<'\n';
  label <<'=';
  it->writeValue(label);
#else
  label <<it->index;
#endif
  return label;
}

void sGraphView::updateGraphvizGraph(bool isSubGraph) {
  if(gvGraph) agclose(gvGraph);

  if(!isSubGraph) {
    gvGraph = agopen(STR("new_graph"), Agdirected, nullptr);
    agattr(gvGraph, AGRAPH, STR("rankdir"), STR("LR"));
    agattr(gvGraph, AGRAPH, STR("ranksep"), STR("0.05"));

    agattr(gvGraph, AGNODE, STR("label"), STR(""));
    agattr(gvGraph, AGNODE, STR("shape"), STR(""));
    agattr(gvGraph, AGNODE, STR("fontsize"), STR("11"));
    agattr(gvGraph, AGNODE, STR("width"), STR(".3"));
    agattr(gvGraph, AGNODE, STR("height"), STR(".3"));

    agattr(gvGraph, AGEDGE, STR("label"), STR(""));
//    agattr(gvGraph, AGEDGE, STR("arrowhead"), STR("none"));
    agattr(gvGraph, AGEDGE, STR("arrowsize"), STR(".5"));
    agattr(gvGraph, AGEDGE, STR("fontsize"), STR("6"));

    uint nNodes = G->index(true);
    gvNodes.resize(nNodes);
  }

  //first add `nodes' for all items
  for(Node* e: *G) {
    gvNodes(e->index) = agnode(gvGraph, STRING(e->index), true);
    agset(gvNodes(e->index), STR("label"), label(e));
    if(e->parents.N) {
      agset(gvNodes(e->index), STR("shape"), STR("box"));
      agset(gvNodes(e->index), STR("fontsize"), STR("6"));
      agset(gvNodes(e->index), STR("width"), STR(".1"));
      agset(gvNodes(e->index), STR("height"), STR(".1"));
    } else {
      agset(gvNodes(e->index), STR("shape"), STR("ellipse"));
    }
  }

  //now add 'edges' for all relations
  for(Node* e: (*G)) {
    /*if(e->parents.N==2){ //is an edge
      gvNodes(i) = (Agnode_t*)agedge(gvGraph, gvNodes(e->parents(0)->id), gvNodes(e->parents(1)->id)); //, STRING(i <<"_" <<e->name), true);
    }else*/
    if(e->parents.N) {
      uint linkId=0;
      for(Node* n: e->parents) {
        if(n->index<e->index) {
//          ge=agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), STRING(label(n) <<"--" <<label(e)), true);
          agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), nullptr, true);
        } else {
//          ge=agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), STRING(label(n) <<"--" <<label(e)), true);
          agedge(gvGraph, gvNodes(n->index), gvNodes(e->index), nullptr, true);
        }
//        agset(ge, STR("label"), STRING(linkId));

        linkId++;
      }
    }
  }

  if(!isSubGraph) {
    G->index(false);
  }
}

void sGraphView::writeFile(const char* filename) {
  gtkLock();
  GVC_t* gvc = gvContext();
  updateGraphvizGraph();
  gvLayout(gvc, gvGraph, "dot");
  gvRenderFilename(gvc, gvGraph, "canon", filename);
  gvFreeLayout(gvc, gvGraph);
  gvFreeContext(gvc);
  gtkUnlock();
}

void sGraphView::init() {
  gtkLock();
  gv_context = ::gvContext();
  char* bla[] = {STR("dot"), STR("-Tx11"), nullptr};
  gvParseArgs(gv_context, 2, bla);

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

  g_signal_connect((gpointer) container, "delete_event",  G_CALLBACK(on_container_delete_event),  nullptr);
  g_signal_connect((gpointer) drawingarea, "expose_event",  G_CALLBACK(on_drawingarea_expose_event),  nullptr);
  g_signal_connect((gpointer) drawingarea, "motion_notify_event",  G_CALLBACK(on_drawingarea_motion_notify_event),  nullptr);
  g_signal_connect((gpointer) drawingarea, "configure_event",  G_CALLBACK(on_drawingarea_configure_event),  nullptr);
  g_signal_connect((gpointer) drawingarea, "button_press_event",  G_CALLBACK(on_drawingarea_button_press_event),  nullptr);
  g_signal_connect((gpointer) drawingarea, "button_release_event",  G_CALLBACK(on_drawingarea_button_release_event),  nullptr);
  g_signal_connect((gpointer) drawingarea, "scroll_event",  G_CALLBACK(on_drawingarea_scroll_event),  nullptr);

  gtk_widget_show(container);
  gtkUnlock();
}

bool sGraphView::on_drawingarea_expose_event(GtkWidget* widget, GdkEventExpose*  event, gpointer user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_drawingarea_expose_event);

  GVJ_t* job = gv->gvJob();
  cairo_t* cr = gdk_cairo_create(gtk_widget_get_window(widget));

  job->context = (void*)cr;
  job->external_context = TRUE;
  job->width = widget->allocation.width; //gtk_widget_get_allocated_width(widget);
  job->height = widget->allocation.height; //gtk_widget_get_allocated_height(widget);
  if(job->has_been_rendered) {
    if(job->callbacks)(job->callbacks->refresh)(job);
  } else {
    if(job->callbacks)(job->callbacks->refresh)(job);
  }

  cairo_destroy(cr);

  if(job->current_obj) {
    if(agobjkind(job->current_obj)==AGNODE || agobjkind(job->current_obj)==AGEDGE) {
      int i=gv->gvNodes.findValue((Agnode_t*)job->current_obj);
      if(i<0 || i>=(int)gv->G->N) {
        RAI_MSG("This is no object id:" <<i);
      } else {
        cout <<"current object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }
  if(job->selected_obj) {
    if(agobjkind(job->selected_obj)==AGNODE) {
      int i=gv->gvNodes.findValue((Agnode_t*)job->selected_obj);
      if(i<0) {
        RAI_MSG("???");
      } else {

        cout <<"selected object:" <<i <<' ' <<*(*gv->G)(i) <<endl;
      }
    }
  }

  return FALSE;
}

bool sGraphView::on_drawingarea_motion_notify_event(GtkWidget*       widget,                   GdkEventMotion*  event,                   gpointer         user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_drawingarea_motion_notify_event);

  GVJ_t* job = gv->gvJob();
  if(!job) return false;
  job->pointer.x = event->x;
  job->pointer.y = event->y;
  if(job->callbacks)(job->callbacks->motion)(job, job->pointer);

  gtk_widget_queue_draw(widget);

  return FALSE;
}

bool sGraphView::on_container_delete_event(GtkWidget*       widget,       GdkEvent*        event,       gpointer         user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_container_delete_event);

  gtk_main_quit();
  return FALSE;
}

bool sGraphView::on_drawingarea_configure_event(GtkWidget*       widget,               GdkEventConfigure* event,               gpointer         user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_drawingarea_configure_event);

  /*FIXME - should allow for margins */
  /*      - similar zoom_to_fit code exists in: */
  /*      plugin/gtk/callbacks.c */
  /*      plugin/xlib/gvdevice_xlib.c */
  /*      lib/gvc/gvevent.c */

  GVJ_t* job = gv->gvJob();
  if(!job) return false;

  double zoom_to_fit;
//  if(!job->has_been_rendered) {
//    zoom_to_fit = 1.0;
//    rai::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
//    if(zoom_to_fit < 1.0)  /* don't make bigger */
//      job->zoom *= zoom_to_fit;
//  } else if(job->fit_mode) {
  zoom_to_fit = rai::MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
  job->zoom *= zoom_to_fit;
//  }
  if(event->width > (int)job->width || event->height > (int)job->height)
    job->has_grown = TRUE;
  job->width = event->width;
  job->height = event->height;
  job->needs_refresh = TRUE;

  return FALSE;
}

bool sGraphView::on_drawingarea_button_press_event(GtkWidget*       widget,                  GdkEventButton*  event,                  gpointer         user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_drawingarea_button_press_event);

  GVJ_t* job = gv->gvJob();
  if(!job) return false;

  pointf pointer;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_press)(job, event->button, pointer);

  gtk_widget_queue_draw(widget);

  return FALSE;
}

bool sGraphView::on_drawingarea_button_release_event(GtkWidget* widget, GdkEventButton*  event, gpointer user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_drawingarea_button_release_event);

  GVJ_t* job = gv->gvJob();
  if(!job) return false;

  pointf pointer;
  pointer.x = event->x;
  pointer.y = event->y;
  (job->callbacks->button_release)(job, event->button, pointer);

  gtk_widget_queue_draw(widget);

  return FALSE;
}

bool sGraphView::on_drawingarea_scroll_event(GtkWidget*       widget,            GdkEventScroll*        event,            gpointer         user_data) {
  sGraphView* gv = (sGraphView*)g_object_get_data(G_OBJECT(widget), "GraphvizGtk");
  INFO(on_drawingarea_scroll_event);

  GVJ_t* job = gv->gvJob();
  if(!job) return false;

  pointf pointer;
  pointer.x = event->x;
  pointer.y = event->y;
  switch(((GdkEventScroll*)event)->direction) {
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

#else //for bad versions
GraphView::GraphView(Graph& G, const char* title, void* container) { NICO }
GraphView::~GraphView() { NICO }
void GraphView::watch() { NICO }
void GraphView::update() { NICO }
#endif

#else //defined RAI_GTK and defined RAI_GRAPHVIZ
#include "graphview.h"

namespace rai {
struct sGraphView {};

GraphView::GraphView(Graph& G, const char* title, void* container) { NICO }
GraphView::~GraphView() { NICO }
void GraphView::watch() { NICO }
void GraphView::update() { NICO }
}
#endif

//===========================================================================
//
// explicit instantiations
//

#if defined RAI_GTK and defined RAI_GRAPHVIZ and defined PACKAGE_URL
template rai::Array<Agnode_t*>::Array();
template rai::Array<Agnode_t*>::~Array();
#endif
