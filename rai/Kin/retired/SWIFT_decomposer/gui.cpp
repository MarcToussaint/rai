/*************************************************************************\

  Copyright 2001 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify OR distribute this software and its
  documentation for educational, research and non-profit purposes, without
  fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             S. Ehmann, M. Lin
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919) 962-1749

  EMail:               geom@cs.unc.edu
                       ehmann@cs.unc.edu
                       lin@cs.unc.edu

\**************************************************************************/


//////////////////////////////////////////////////////////////////////////////
//
// gui.cpp
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
using std::cerr;
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "../SWIFT/SWIFT.h"
#include "../SWIFT/SWIFT_mesh.h"
#include "../SWIFT/SWIFT_mesh_utils.h"
#include "../SWIFT/SWIFT_fileio.h"

#ifdef DECOMP_GRAPHICS
#include <GL/gl.h>
#include <GL/glu.h>
#include <togl.h>
#include <viewer.h>
#include "guidraw.h"
#endif

#include "convex.h"
#include "io.h"
#include "gui.h"

// Comand line options
#ifdef DECOMP_GRAPHICS
extern bool g;
#endif

extern bool jitter;
extern SWIFT_Real jampl;

extern bool ef;
extern SWIFT_Real edge_flip_tol;
extern char* ef_filename;

extern bool dfs;
extern bool bfs;
extern char* decomp_filename;
extern bool w;
extern bool one_piece;

extern bool hierarchy;
extern char* hier_filename;
extern SPLIT_TYPE split;


#ifdef DECOMP_GRAPHICS
// Tk vars
Togl* t;

// User interaction variables
SWIFT_Real mouse_x, mouse_y;
int which_mouse;

// User interface state
int backface;
int wireframe;
int color;
int axes;

int explode;
int prevdh;
int dh;
const int DRAW_DATA_NONE = 0;
const int DRAW_DECOMPOSITION = 1;
const int DRAW_HIERARCHY = 2;
int edge_conv;
int vfaces;
int save_vfaces;

int tcolor;
int uleaves;

bool dragging;
#endif

// The mesh being decomposed
SWIFT_Tri_Mesh* mesh;

// The edge convexities
SWIFT_Array<bool> edge_convexities;

// The convex piece ids
SWIFT_Array<int> piece_ids;

// The number of pieces
int num_pieces;

// The convex piece geometries
SWIFT_Array< SWIFT_Array<int> > model_faces;
SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> > virtual_faces;
SWIFT_Array<SWIFT_Tri_Face*> st_faces;
SWIFT_Array<SWIFT_Tri_Edge*> st_twins;

#ifdef DECOMP_GRAPHICS
// The convex piece display ids and the centers of mass
SWIFT_Array<int> which_cps;
SWIFT_Array<SWIFT_Triple> piece_coms;
#endif

// File readers and writers
bool already_decomp;
bool already_hier;

#ifdef DECOMP_GRAPHICS
// Drawing variables
int level;
SWIFT_Array<SWIFT_BV*> which_pieces;
SWIFT_Array<SWIFT_BV*> leaves;
SWIFT_Triple explode_center;
#endif
int num_leaves;


//////////////////////////////////////////////////////////////////////////////
// Local routines
//////////////////////////////////////////////////////////////////////////////
#ifdef DECOMP_GRAPHICS
void Initialize_For_New_Model( )
{
    if( mesh != nullptr ) {
        SWIFT_Triple mint, maxt;
        mesh->Compute_Bounding_Box( mint, maxt );
        explode_center = 0.5 * (mint + maxt);
        maxt -= mint;
        mint = explode_center-maxt;
        maxt += explode_center;
        Set_Model_Dim( mint.X()*2.0, maxt.X()*2.0, mint.Y()*2.0, maxt.Y()*2.0,
                       mint.Z()*2.0, maxt.Z()*2.0 );
    }

    Save_Camera( 0 );
}
#endif

void Decompose_Mesh( )
{
    Convex_Initialize( mesh );

    // Decompose the polyhedron
    if( dfs ) {
        num_pieces = Decompose_DFS( mesh, piece_ids, model_faces,
                                                        virtual_faces, false );
    } else if( bfs ) {
        num_pieces = Decompose_BFS( mesh, piece_ids, model_faces,
                                                        virtual_faces, false );
    } else {
        num_pieces = Decompose_Cresting_BFS( mesh, piece_ids, model_faces,
                                                              virtual_faces );
    }
}

#ifdef DECOMP_GRAPHICS
void Compute_Piece_Centers_Of_Mass( )
{
    int i, j;
    SWIFT_Real area;
    SWIFT_Real total_area;
    SWIFT_Triple areav;
    SWIFT_Triple com;

    if( model_faces.Length() != 0 ) {
        piece_coms.Create( model_faces.Length() );
        for( i = 0; i < model_faces.Length(); i++ ) { 
            com.Set_Value( 0.0, 0.0, 0.0 );
            total_area = 0.0;
            for( j = 0; j < model_faces[i].Length(); j++ ) {
                areav = (mesh->Faces()[model_faces[i][j]].Edge1().Origin()->Coords() -
                         mesh->Faces()[model_faces[i][j]].Edge2().Origin()->Coords()) %
                        (mesh->Faces()[model_faces[i][j]].Edge1().Origin()->Coords() -
                         mesh->Faces()[model_faces[i][j]].Edge3().Origin()->Coords());
                area = 0.5 * areav.Length();
                total_area += area;
                com += area * (mesh->Faces()[model_faces[i][j]].Edge1().Origin()->Coords() +
                               mesh->Faces()[model_faces[i][j]].Edge2().Origin()->Coords() +
                               mesh->Faces()[model_faces[i][j]].Edge3().Origin()->Coords() );
            }
            for( j = 0; j < virtual_faces[i].Length(); j++ ) {
                areav = (virtual_faces[i][j].Edge1().Origin()->Coords() -
                        virtual_faces[i][j].Edge2().Origin()->Coords()) %
                        (virtual_faces[i][j].Edge1().Origin()->Coords() -
                        virtual_faces[i][j].Edge3().Origin()->Coords());
                area = 0.5 * areav.Length();
                total_area += area;
                com += area * (virtual_faces[i][j].Edge1().Origin()->Coords() +
                               virtual_faces[i][j].Edge2().Origin()->Coords() +
                               virtual_faces[i][j].Edge3().Origin()->Coords() );
            }

            piece_coms[i] = com / (3.0 * total_area);
        }
    }
}
#endif

#ifdef DECOMP_GRAPHICS
void Compute_Leaves( SWIFT_BV* piece )
{
    int i;

    if( piece == mesh->Root() ) {
        leaves.Destroy();
        leaves.Create( num_leaves );
        leaves.Set_Length( 0 );
    }

    if( piece->Is_Leaf() ) {
        leaves.Add( piece );
    } else {
        for( i = 0; i < piece->Num_Children(); i++ ) {
            Compute_Leaves( piece->Children()[i] );
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Togl Callback routines
//////////////////////////////////////////////////////////////////////////////
void Create_CB( struct Togl *togl )
{
    // Setup back-face culling
    glEnable( GL_CULL_FACE );
    glFrontFace( GL_CCW );
    glCullFace( GL_BACK );

    // Setup the depth buffer
    glEnable( GL_DEPTH_TEST );
    glDepthMask( GL_TRUE );

    // Setup the background color
    glClearColor( 0.0, 0.0, 0.0, 0.0 );

    // Setup shading model
    glShadeModel( GL_FLAT );

    // Setup polygon offsetting
    glPolygonOffset( 1.0, 1.0 );

    // Setup polygon mode
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

    // Setup lighting
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glEnable( GL_LIGHT1 );
    glEnable( GL_LIGHT2 );

    GLfloat light_model_ambient[4] = {0.2, 0.2, 0.2, 1.0};

    glLightModelfv( GL_LIGHT_MODEL_AMBIENT, light_model_ambient );
    glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );
    glLightModeli( GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE );

    GLfloat light_diffuse0[] = {0.2, 0.2, 0.2, 1.0};

    glLightfv( GL_LIGHT0, GL_DIFFUSE, light_diffuse0 );
    Create_Light( 0, 0.0, 0.0, 1.0, 1, 1 );

    // Set up viewing
    glViewport( 0, 0, Togl_Width(togl), Togl_Height(togl) );

    t = togl;
}

void Redraw_CB( struct Togl *togl )
{
    GLfloat mat[4] = { 0.0, 0.0, 0.0, 1.0 };

    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mat );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, mat );
    glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION, mat );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Set the focus to the draw area
    Tcl_Eval( Togl_Interp( togl ), "focus .view" );

    // Set the radio buttons correctly
    if( !dfs ) {
        Tcl_Eval( Togl_Interp(t),
                  ".cp1.p3.f1.f1.r1 configure -state disabled" );
    }
    if( !bfs ) {
        Tcl_Eval( Togl_Interp(t),
                  ".cp1.p3.f1.f1.r2 configure -state disabled" );
    }
    Tcl_Eval( Togl_Interp(t), ".cp1.p3.f1.f1.r3 configure -state disabled" );
    Tcl_Eval( Togl_Interp(t), ".cp1.p3.f1.f1.r4 configure -state disabled" );

    Set_OpenGL_Camera( );

    // Draw the axes
    if( axes ) {
        mat[0] = 0.0, mat[1] = 2.0, mat[2] = 0.0;
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mat );

        Draw_World_Axes();

        mat[0] = 0.0, mat[1] = 2.0, mat[2] = 2.0;
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mat );

        Draw_Model_Axes();
    }

    if( wireframe ) {
        // Draw the triangles in black pushed back
        glPolygonOffset( 2.0, 2.0 );
        glEnable( GL_POLYGON_OFFSET_FILL );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        mat[0] = 0.0; mat[1] = 0.0; mat[2] = 0.0;
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mat );
        if( dh == DRAW_HIERARCHY )  {
            Draw_Hierarchy( mesh, false, (bool)vfaces, (bool)tcolor,
                            (bool)explode, explode_center, piece_ids,
                            which_pieces );
        } else {
                Draw_Convex_Pieces( mesh, false, (bool)vfaces, (bool)tcolor,
                                    (bool)explode, explode_center, piece_coms,
                                    piece_ids, model_faces, virtual_faces,
                                    which_cps );
        }
        glDisable( GL_POLYGON_OFFSET_FILL );

        if( !edge_conv ) {
            // Then draw wireframe in off-white pushed back but not as far
            glPolygonOffset( 1.0, 1.0 );
            glEnable( GL_POLYGON_OFFSET_LINE );
            glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
            mat[0] = 2.0; mat[1] = 2.0; mat[2] = 2.0;
            glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mat );
            if( dh == DRAW_HIERARCHY )  {
                Draw_Hierarchy( mesh, (bool)color, (bool)vfaces, (bool)tcolor,
                                (bool)explode, explode_center, piece_ids,
                                which_pieces );
            } else {
                    Draw_Convex_Pieces( mesh, (bool)color, (bool)vfaces,
                                        (bool)tcolor, (bool)explode,
                                        explode_center, piece_coms, piece_ids,
                                        model_faces, virtual_faces, which_cps );
            }
            glDisable( GL_POLYGON_OFFSET_LINE );
        }
    } else {
        // Draw the triangles like normal pushed back
        glEnable( GL_POLYGON_OFFSET_FILL );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        mat[0] = 0.6; mat[1] = 0.6; mat[2] = 0.6;
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat );
        glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, mat );
        glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 2.0 );
        if( dh == DRAW_HIERARCHY )  {
            Draw_Hierarchy( mesh, true, (bool)vfaces, (bool)tcolor,
                            (bool)explode, explode_center,
                            piece_ids, which_pieces );
        } else {
                Draw_Convex_Pieces( mesh, true, (bool)vfaces, (bool)tcolor,
                                    (bool)explode, explode_center, piece_coms,
                                    piece_ids, model_faces, virtual_faces,
                                    which_cps );
        }
        glDisable( GL_POLYGON_OFFSET_FILL );
    }


    if( edge_conv ) {
        Draw_Edge_Convexity( mesh, edge_convexities );
    }


    // Draw this frame
    Togl_SwapBuffers( togl );
}

void Resize_CB( struct Togl *togl )
{
    Togl_PostRedisplay( togl );
}

void Destroy_CB( struct Togl *togl )
{
    // Do nothing here
}

//////////////////////////////////////////////////////////////////////////////
// Button Callback routines
//////////////////////////////////////////////////////////////////////////////

int Center_World_CB( Togl *togl, int argc, const char *argv[] )
{
    Jump_To_Camera( 0 );
    Togl_PostRedisplay( t );

    return TCL_OK;
}

int Set_Center_CB( Togl *togl, int argc, const char *argv[] )
{
    Save_Camera( 1 );

    return TCL_OK;
}

int Center_User_CB( Togl *togl, int argc, const char *argv[] )
{
    Jump_To_Camera( 1 );
    Togl_PostRedisplay( t );
    return TCL_OK;
}

//////////////////////////////////////////////////////////////////////////////
// Toggle/Radio Callback routines
//////////////////////////////////////////////////////////////////////////////
int Backface_Cull_CB( Togl *togl, int argc, const char *argv[] )
{
    if( backface ) {
        glEnable( GL_CULL_FACE );
    } else {
        glDisable( GL_CULL_FACE );
    }

    Togl_PostRedisplay( t );
    return TCL_OK;
}

int Redisplay_CB( Togl *togl, int argc, const char *argv[] )
{
    Togl_PostRedisplay( t );
    return TCL_OK;
}

int Convex_Pieces_CB( Togl *togl, int argc, const char *argv[] )
{
    if( argv[2][0] == 'A' || argv[2][0] == 'a' ) {
        // Want to draw all pieces
        int i;
        which_cps.Set_Length( which_cps.Max_Length() );
        for( i = 0; i < which_cps.Length(); i++ ) {
            which_cps[i] = i;
        }
    } else {
        // Parse the string to determine which ones to draw
        long upper, lower;
        const char* str = argv[2];
        char* endp;
        int i;
        SWIFT_Array<int> which_cps_back = which_cps;

        which_cps.Set_Length( 0 );

        while( *str != '\0' && which_cps.Length() != which_cps.Max_Length() ) {
            if( isdigit( *str ) ) {
                // Read the next segment
                lower = strtol( str, &endp, 10 );
                str = endp;
                if( lower < 0 ) {
                    lower = 0;
                }
                if( lower >= which_cps.Max_Length() ) {
                    lower = which_cps.Max_Length()-1;
                }
                upper = lower;
                if( *str == '-' ) {
                    str++;
                    if( isdigit( *str ) ) {
                        upper = strtol( str, &endp, 10 );
                        str = endp;
                        if( upper < 0 ) {
                            upper = 0;
                        }
                        if( upper >= which_cps.Max_Length() ) {
                            upper = which_cps.Max_Length()-1;
                        }
                        if( upper < lower ) {
                            int j = lower;
                            lower = upper;
                            upper = j;
                        }
                    } else {
                        cerr << "Error: Expecting number after '-'" << endl;
                        which_cps = which_cps_back;
                        break;
                    }
                }

                // Save the segment
                for( i = lower; i <= upper &&
                     which_cps.Length() != which_cps.Max_Length(); i++
                ) {
                    which_cps.Add( i );
                }

                // Done this segment
                if( *str == ',' ) {
                    str++;
                } else {
                    // Assume that we found the end of the list
                    break;
                }
            } else {
                break;
            }
        }
    }

    Togl_PostRedisplay( t );
    return TCL_OK;
}

int Upper_Leaves_CB( Togl *togl, int argc, const char *argv[] )
{
    int i;

    if( uleaves ) {
        // Include all leaves above the current level
        for( i = 0; i < num_leaves; i++ ) {
            if( leaves[i]->Level() < level ) {
                which_pieces.Add_Grow( leaves[i], 10 );
            }
        }
    } else {
        // Remove all leaves above the current level
        SWIFT_Array<SWIFT_BV*> new_which_pieces;

        new_which_pieces.Create( which_pieces.Length() );
        new_which_pieces.Set_Length( 0 );
        for( i = 0; i < which_pieces.Length(); i++ ) {
            if( !which_pieces[i]->Is_Leaf() ||
                which_pieces[i]->Level() == level
            ) {
                // Keep this piece
                new_which_pieces.Add( which_pieces[i] );
            }
        }

        which_pieces.Destroy();
        which_pieces = new_which_pieces;
        new_which_pieces.Nullify();
    }

    Togl_PostRedisplay( t );
    return TCL_OK;
}

int Decomp_Hier_CB( Togl *togl, int argc, const char *argv[] )
{
    if( prevdh == dh ) {
        // No change
        return TCL_OK;
    }

    if( !hierarchy ) {
        // Force the menu to stay at decomp
        Tcl_Eval( Togl_Interp(togl), "set dh 1" );
        Tcl_Eval( Togl_Interp(togl), "pack forget .cp1.p3.f2" );
        Tcl_Eval( Togl_Interp(togl),
              "pack .cp1.p3.f1 -side top -fill x -expand true" );
        Tcl_Eval( Togl_Interp(togl), "focus .view; .view datasel" );
        Tcl_Eval( Togl_Interp(togl), "set editframe \"Decomposition (d)\"" );
        return TCL_OK;
    }

    // Restore the vfaces setting
    if( vfaces != save_vfaces ) {
        int temp = vfaces;
        vfaces = save_vfaces;
        save_vfaces = temp;
        Tcl_Eval( Togl_Interp(togl), ".cp1.p3.f99.cb1 toggle" );
    }

    prevdh = dh;

    Togl_PostRedisplay( t );
    return TCL_OK;
}

//////////////////////////////////////////////////////////////////////////////
// Key Callback routines
//////////////////////////////////////////////////////////////////////////////
int Key_A_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), "set which_cps A" );
    Tcl_Eval( Togl_Interp(togl),
              "global which_cps; focus .view; .view drawcps $which_cps" );

    return TCL_OK;
}

int Key_B_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), ".cp1.p2.f1.cb0 toggle" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view backfacecull" );

    return TCL_OK;
}

int Key_C_CB( Togl *togl, int argc, const char *argv[] )
{
    if( dragging ) {
        return TCL_OK;
    }

    // Same as pressing the center user button
    return Center_User_CB( togl, argc, argv );
}

int Key_Shift_C_CB( Togl *togl, int argc, const char *argv[] )
{
    if( dragging ) {
        return TCL_OK;
    }

    // Same as pressing the center world button
    return Center_World_CB( togl, argc, argv );
}

int Key_D_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), "set dh 1" );
    Tcl_Eval( Togl_Interp(togl), "pack forget .cp1.p3.f2;" );
    Tcl_Eval( Togl_Interp(togl),
              "pack .cp1.p3.f1 -side top -fill x -expand true;" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view datasel;" ); 
    Tcl_Eval( Togl_Interp(togl), "set editframe \"Decomposition (d)\"" );

    return TCL_OK;
}

int Key_H_CB( Togl *togl, int argc, const char *argv[] )
{
    if( !hierarchy ) {
        return TCL_OK;
    }

    Tcl_Eval( Togl_Interp(togl), "set dh 2" );
    Tcl_Eval( Togl_Interp(togl), "pack forget .cp1.p3.f1;" );
    Tcl_Eval( Togl_Interp(togl),
              "pack .cp1.p3.f2 -side top -fill x -expand true;" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view datasel;" ); 
    Tcl_Eval( Togl_Interp(togl), "set editframe \"Hierarchy (h)\"" );

    return TCL_OK;
}

int Key_J_CB( Togl *togl, int argc, const char *argv[] )
{
    if( dh == DRAW_HIERARCHY ) {
        int i, j;
        bool advanced = false;
        SWIFT_Array<SWIFT_BV*> new_which_pieces( 100 );

        new_which_pieces.Set_Length( 0 );
        for( i = 0; i < which_pieces.Length(); i++ ) {
            if( uleaves && which_pieces[i]->Is_Leaf() ) {
                // Keep this leaf
                new_which_pieces.Add_Grow( which_pieces[i], 10 );
            } else {
                for( j = 0; j < which_pieces[i]->Num_Children(); j++ ) {
                    new_which_pieces.Add_Grow(
                                        which_pieces[i]->Children()[j], 10 );
                    advanced = true;
                }
            }
        }

        if( advanced ) {
            level++;
            which_pieces.Destroy();
            which_pieces = new_which_pieces;
            new_which_pieces.Nullify();
        }
    } else {
            // Show previous convex piece and set text field
            char temp[80];

            if( which_cps.Length() == 0 ) {
                which_cps.Set_Length( 1 );
                which_cps[0] = 0;
            } else {
                which_cps[0] = (which_cps[0] == 0 ?
                                   which_cps.Max_Length()-1 : which_cps[0]-1);
                which_cps.Set_Length( 1 );
            }
            sprintf( temp, "set which_cps %d", which_cps[0] );
            Tcl_Eval( Togl_Interp( togl ), temp );
    }

    Togl_PostRedisplay( togl );
    return TCL_OK;
}

int Key_K_CB( Togl *togl, int argc, const char *argv[] )
{
    if( dh == DRAW_HIERARCHY ) {
        if( which_pieces.Length() != 1 ) {
            // Go up the hierarchy
            int i;
            SWIFT_Array<SWIFT_BV*> new_which_pieces( which_pieces.Length() );
            SWIFT_BV* parent;

            new_which_pieces.Set_Length( 0 );
            parent = nullptr;
            for( i = 0; i < which_pieces.Length(); i++ ) {
                if( uleaves && which_pieces[i]->Is_Leaf() &&
                    which_pieces[i]->Level() < level
                ) {
                    new_which_pieces.Add( which_pieces[i] );
                } else if( parent != which_pieces[i]->Parent() ) {
                    parent = which_pieces[i]->Parent();
                    new_which_pieces.Add( parent );
                }
            }

            level--;

            // Include the new leaves at this level if uleaves is false
            if( !uleaves ) {
                for( i = 0; i < num_leaves; i++ ) {
                    if( leaves[i]->Level() == level ) {
                        new_which_pieces.Add_Grow( leaves[i], 10 );
                    }
                }
            }

            which_pieces.Destroy();
            which_pieces = new_which_pieces;
            new_which_pieces.Nullify();
        }
    } else {
            // Show next convex piece and set text field
            char temp[80];

            if( which_cps.Length() == 0 ) {
                which_cps.Set_Length( 1 );
                which_cps[0] = 0;
            } else {
                which_cps[0] = (which_cps[0] == which_cps.Max_Length()-1 ?
                                                0 : which_cps[0]+1);
                which_cps.Set_Length( 1 );
            }
            sprintf( temp, "set which_cps %d", which_cps[0] );
            Tcl_Eval( Togl_Interp( togl ), temp );
    }

    Togl_PostRedisplay( togl );
    return TCL_OK;
}


int Key_Q_CB( Togl *togl, int argc, const char *argv[] )
{
    if( dragging ) {
        return TCL_OK;
    }

    mesh->Prepare_For_Delete();
    // Exit uncleanly
    exit( 0 );

    return TCL_OK;
}

int Key_S_CB( Togl *togl, int argc, const char *argv[] )
{
    if( dragging ) {
        return TCL_OK;
    }

    // Same as pressing the save center button
    return Set_Center_CB( togl, argc, argv );
}

int Key_T_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), ".cp1.p3.f99.cb2 toggle" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view drawtcol" );

    return TCL_OK;
}

int Key_V_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), ".cp1.p3.f99.cb1 toggle" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view drawvfs" );

    return TCL_OK;
}

int Key_W_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), ".cp1.p2.f1.cb1 toggle" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view wireframe" );

    return TCL_OK;
}

int Key_X_CB( Togl *togl, int argc, const char *argv[] )
{
    Tcl_Eval( Togl_Interp(togl), ".cp1.p3.f99.cb0 toggle" );
    Tcl_Eval( Togl_Interp(togl), "focus .view; .view drawexpl" );

    return TCL_OK;
}

int Key_Plus_CB( Togl *togl, int argc, const char *argv[] )
{
    return Key_K_CB( togl, argc, argv );
}

int Key_Minus_CB( Togl *togl, int argc, const char *argv[] )
{
    return Key_J_CB( togl, argc, argv );
}


//////////////////////////////////////////////////////////////////////////////
// Mouse helper routines
//////////////////////////////////////////////////////////////////////////////
inline void Get_Mouse_Pos( const char* xstr, const char* ystr,
                           SWIFT_Real& x, SWIFT_Real& y )
{
    Tcl_Interp *interp = Togl_Interp(t);
    int ix, iy;

    Tcl_GetInt(interp, xstr, &ix);
    Tcl_GetInt(interp, ystr, &iy);

    x = (SWIFT_Real)ix;
    y = (SWIFT_Real)(Togl_Height(t) - iy);
}

int Mouse_1_Up_CB( Togl *togl, int argc, const char *argv[] );
int Mouse_2_Up_CB( Togl *togl, int argc, const char *argv[] );
int Mouse_3_Up_CB( Togl *togl, int argc, const char *argv[] );
void Ensure_Mouse_Released( )
{
    if( dragging ) {
        // Another mouse button is already down.  Simulate a release of it
        if( which_mouse == 1 ) {
            Mouse_1_Up_CB( t, 0, nullptr );
        } else if( which_mouse == 2 ) {
            Mouse_2_Up_CB( t, 0, nullptr );
        } else {
            Mouse_3_Up_CB( t, 0, nullptr );
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Mouse 1 Callback routines
//////////////////////////////////////////////////////////////////////////////
int Mouse_1_Down_CB( Togl *togl, int argc, const char *argv[] )
{
    Get_Mouse_Pos( argv[2], argv[3], mouse_x, mouse_y );
    Ensure_Mouse_Released( );
    which_mouse = 1;
    dragging = true;
    Start_Plain_Mouse1( mouse_x / (SWIFT_Real)Togl_Width( t ),
                        mouse_y / (SWIFT_Real)Togl_Height( t ) );
    return TCL_OK;
}

int Mouse_1_Shift_Down_CB( Togl *togl, int argc, const char *argv[] )
{
    Get_Mouse_Pos( argv[2], argv[3], mouse_x, mouse_y );
    Ensure_Mouse_Released( );
    which_mouse = 1;
    dragging = true;
    Start_Shift_Mouse1( mouse_x / (SWIFT_Real)Togl_Width( t ),
                        mouse_y / (SWIFT_Real)Togl_Height( t ) );
    return TCL_OK;
}


int Mouse_1_Up_CB( Togl *togl, int argc, const char *argv[] )
{
    dragging = false;
    return TCL_OK;
}

//////////////////////////////////////////////////////////////////////////////
// Mouse 2 Callback routines
//////////////////////////////////////////////////////////////////////////////
int Mouse_2_Down_CB( Togl *togl, int argc, const char *argv[] )
{
    Get_Mouse_Pos( argv[2], argv[3], mouse_x, mouse_y );
    Ensure_Mouse_Released( );
    which_mouse = 2;
    dragging = true;
    Start_Plain_Mouse2( mouse_x / (SWIFT_Real)Togl_Width( t ),
                        mouse_y / (SWIFT_Real)Togl_Height( t ) );
    return TCL_OK;
}

int Mouse_2_Shift_Down_CB( Togl *togl, int argc, const char *argv[] )
{
    Get_Mouse_Pos( argv[2], argv[3], mouse_x, mouse_y );
    Ensure_Mouse_Released( );
    which_mouse = 2;
    dragging = true;
    Start_Shift_Mouse2( mouse_x / (SWIFT_Real)Togl_Width( t ),
                        mouse_y / (SWIFT_Real)Togl_Height( t ) );
    return TCL_OK;
}


int Mouse_2_Up_CB( Togl *togl, int argc, const char *argv[] )
{
    dragging = false;
    return TCL_OK;
}

//////////////////////////////////////////////////////////////////////////////
// Mouse 3 Callback routines
//////////////////////////////////////////////////////////////////////////////
int Mouse_3_Down_CB( Togl *togl, int argc, const char *argv[] )
{
    Get_Mouse_Pos( argv[2], argv[3], mouse_x, mouse_y );
    Ensure_Mouse_Released( );
    which_mouse = 3;
    dragging = true;
    Start_Plain_Mouse3( mouse_x / (SWIFT_Real)Togl_Width( t ),
                        mouse_y / (SWIFT_Real)Togl_Height( t ) );
    return TCL_OK;
}

int Mouse_3_Shift_Down_CB( Togl *togl, int argc, const char *argv[] )
{
    Get_Mouse_Pos( argv[2], argv[3], mouse_x, mouse_y );
    Ensure_Mouse_Released( );
    which_mouse = 3;
    dragging = true;
    Start_Shift_Mouse3( mouse_x / (SWIFT_Real)Togl_Width( t ),
                        mouse_y / (SWIFT_Real)Togl_Height( t ) );
    return TCL_OK;
}


int Mouse_3_Up_CB( Togl *togl, int argc, const char *argv[] )
{
    dragging = false;
    return TCL_OK;
}

//////////////////////////////////////////////////////////////////////////////
// Mouse Motion Callback routines
//////////////////////////////////////////////////////////////////////////////
// Mouse motion with a mouse button pressed down
int Mouse_Drag_CB( Togl *togl, int argc, const char *argv[] )
{
    SWIFT_Real dx, dy;
    Get_Mouse_Pos( argv[2], argv[3], dx, dy );

        Adjust( dx / (SWIFT_Real)Togl_Width( togl ),
                dy / (SWIFT_Real)Togl_Height( togl ) );
        Togl_PostRedisplay( t );

    mouse_x = dx;
    mouse_y = dy;

    return TCL_OK;
}

#endif

//////////////////////////////////////////////////////////////////////////////
// Exported routines
//////////////////////////////////////////////////////////////////////////////

void Gui_Init_Before_TclTk( char* filename )
{
#ifdef DECOMP_GRAPHICS
    if( g ) {
        // toggle and radio button variables
        backface = 1;
        wireframe = 0;
        color = 1;
        axes = 1;

        explode = 0;
        prevdh = DRAW_DECOMPOSITION;
        dh = DRAW_DECOMPOSITION;
        edge_conv = 0;
        vfaces = 0;
        save_vfaces = 1;    // turn vfaces on by default for the hierarchy

        tcolor = 0;
        uleaves = 0;

        level = 0;

        // Mode variables
        dragging = false;

        VIEWER_Initialize();
    }
#endif

    mesh = nullptr;

    Mesh_Utils_Initialize();

    if( filename != nullptr ) {
        int i, j, k;

        if( !Load_File( filename, mesh, split, already_decomp, already_hier,
                        piece_ids, model_faces, virtual_faces )
        ) {
            cerr << "Exiting..." << endl;
            exit( 0 );
            return;
        }

        if( already_hier ) {
            // Have to compute the mesh geometry
            mesh->Compute_All_Hierarchy_Geometry();
        }

        mesh->Compute_Edge_Convexities( edge_convexities );
        if( !already_decomp ) {
            if( jitter ) {
                cerr << "Jittering with amplitude = " << jampl << endl << endl;
                Jitter( mesh, jampl );
            }
            if( ef ) {
                // Flip edges
                cerr << "Flipping edges with tolerance = " << edge_flip_tol
                     << endl << endl;
                Edge_Flip( mesh, edge_flip_tol );
                if( ef_filename != nullptr ) {
                    cerr << "Saving edge flipped mesh" << endl << endl;
                    Save_Model_File( ef_filename, mesh );
                }
            }
            if( one_piece ) {
                cerr << "Creating one piece" << endl;
                Create_One_Piece( mesh, piece_ids, model_faces, virtual_faces );
                num_pieces = 1;
            } else {
                Decompose_Mesh( );
            }

            // Write the result to a file if that option is on
            if( w ) {
                cerr << "Saving decomposition result" << endl << endl;
                Save_Decomposition_File( decomp_filename, mesh, piece_ids,
                                         model_faces, virtual_faces );
            }
        } else if( !already_hier ) {
            num_pieces = model_faces.Length();
        } else {
            num_pieces = (mesh->Num_BVs()+1)/2;
        }

        if( hierarchy ) {
            // Create the bounding volume hierarchy
            num_leaves = num_pieces;
            if( !already_hier ) {
                cerr << "Creating convex hierarchy" << endl;
                mesh->Create_BV_Hierarchy( split, piece_ids, model_faces,
                                           virtual_faces, st_faces, st_twins );
                if( hier_filename != nullptr ) {
                    cerr << "Saving convex hierarchy" << endl << endl;
                    Save_Hierarchy_File( hier_filename, mesh,
                                         st_faces, st_twins );
                }
            }
#ifdef DECOMP_GRAPHICS
        } else {
            if( g ) {
                // Compute the virtual face normals
                for( i = 0; i < virtual_faces.Length(); i++ ) {
                    for( j = 0; j < virtual_faces[i].Length(); j++ ) {
                        virtual_faces[i][j].Edge1().Compute_Direction_Length();
                        virtual_faces[i][j].Edge2().Compute_Direction_Length();
                        virtual_faces[i][j].Edge3().Compute_Direction_Length();
                        virtual_faces[i][j].Compute_Plane_From_Edges();
                    }
                }
            }
#endif
        }
        cerr << "COM = " << mesh->Center_Of_Mass() << endl;

#ifdef DECOMP_GRAPHICS
        if( g ) {
            if( hierarchy ) {
                // Hierarchy has been created.

                if( already_hier ) {
                    // Create the piece_ids and the virtual faces
                    piece_ids.Create( mesh->Num_Faces() );
                    virtual_faces.Create( num_pieces );
                    for( i = 0, k = 0; i < mesh->Num_BVs(); i++ ){
                        if( !mesh->BVs()[i].Is_Leaf() ) {
                            continue;
                        }
                        for( j = 0; j < mesh->BVs()[i].Num_Other_Faces(); j++ ){
                            piece_ids[mesh->Face_Id(
                                    mesh->BVs()[i].Other_Faces()[j] )] = k;
                        }
                        virtual_faces[k].Create( mesh->BVs()[i].Num_Faces() );
                        for( j = 0; j < virtual_faces[k].Length(); j++ ) {
                            virtual_faces[k][j] = mesh->BVs()[i].Faces()[j];
                            virtual_faces[k][j].Edge1().Nullify_Twins();
                            virtual_faces[k][j].Edge2().Nullify_Twins();
                            virtual_faces[k][j].Edge3().Nullify_Twins();
                        }
                        k++;
                    }

                    // Create the model faces
                    model_faces.Create( num_pieces );
                    for( i = 0; i < mesh->Num_Faces(); i++ ) {
                        model_faces[piece_ids[i]].Add_Grow( i, 10 );
                    }
                }

                Compute_Leaves( mesh->Root() );

                which_pieces.Create( 1 );
                which_pieces[0] = mesh->Root();
            }

            which_cps.Create( num_pieces );
            for( i = 0; i < num_pieces; i++ ) {
                which_cps[i] = i;
            }
            Compute_Piece_Centers_Of_Mass();
            Initialize_For_New_Model();
            Save_Camera( 1 );
        }
#endif
    } else {
        cerr << "No filename given to initialize -- Exiting..." << endl;
    }
}

#ifdef DECOMP_GRAPHICS
void Gui_Init_After_TclTk( Tcl_Interp *interp )
{
    // Togl callbacks
    Togl_CreateFunc( Create_CB );
    Togl_DisplayFunc( Redraw_CB );
    Togl_ReshapeFunc( Resize_CB );
    Togl_DestroyFunc( Destroy_CB );

    // Button callbacks
    Togl_CreateCommand( "centerw", Center_World_CB );
    Togl_CreateCommand( "scenter", Set_Center_CB );
    Togl_CreateCommand( "centeru", Center_User_CB );

    // Toggle/Radio button callbacks
    Togl_CreateCommand( "backfacecull", Backface_Cull_CB );
    Togl_CreateCommand( "wireframe", Redisplay_CB );
    Togl_CreateCommand( "drawcolor", Redisplay_CB );
    Togl_CreateCommand( "drawaxes", Redisplay_CB );
    Togl_CreateCommand( "drawexpl", Redisplay_CB );
    Togl_CreateCommand( "draweconv", Redisplay_CB );
    Togl_CreateCommand( "drawvfs", Redisplay_CB );
    Togl_CreateCommand( "drawcps", Convex_Pieces_CB );
    Togl_CreateCommand( "drawtcol", Redisplay_CB );
    Togl_CreateCommand( "drawul", Upper_Leaves_CB );
    Togl_CreateCommand( "datasel", Decomp_Hier_CB );

    // Key press callbacks for draw area
    Togl_CreateCommand( "a_key", Key_A_CB );
    Togl_CreateCommand( "b_key", Key_B_CB );
    Togl_CreateCommand( "c_key", Key_C_CB );
    Togl_CreateCommand( "C_key", Key_Shift_C_CB );
    Togl_CreateCommand( "d_key", Key_D_CB );
    Togl_CreateCommand( "h_key", Key_H_CB );
    Togl_CreateCommand( "j_key", Key_J_CB );
    Togl_CreateCommand( "k_key", Key_K_CB );
    Togl_CreateCommand( "q_key", Key_Q_CB );
    Togl_CreateCommand( "s_key", Key_S_CB );
    Togl_CreateCommand( "t_key", Key_T_CB );
    Togl_CreateCommand( "v_key", Key_V_CB );
    Togl_CreateCommand( "w_key", Key_W_CB );
    Togl_CreateCommand( "x_key", Key_X_CB );

    Togl_CreateCommand( "minus_key", Key_Minus_CB );
    Togl_CreateCommand( "plus_key", Key_Plus_CB );

    // Mouse action callbacks for draw area

    // Mouse 1
    Togl_CreateCommand( "mouse1Down", Mouse_1_Down_CB );
    Togl_CreateCommand( "mouse1ShiftDown", Mouse_1_Shift_Down_CB );
    Togl_CreateCommand( "mouse1Up", Mouse_1_Up_CB );

    // Mouse 2
    Togl_CreateCommand( "mouse2Down", Mouse_2_Down_CB );
    Togl_CreateCommand( "mouse2ShiftDown", Mouse_2_Shift_Down_CB );
    Togl_CreateCommand( "mouse2Up", Mouse_2_Up_CB );

    // Mouse 3
    Togl_CreateCommand( "mouse3Down", Mouse_3_Down_CB );
    Togl_CreateCommand( "mouse3ShiftDown", Mouse_3_Shift_Down_CB );
    Togl_CreateCommand( "mouse3Up", Mouse_3_Up_CB );

    Togl_CreateCommand( "mouseDrag", Mouse_Drag_CB );

    // Link toggle/radio button variables
    Tcl_LinkVar( interp, "backface", (char *) &backface, TCL_LINK_BOOLEAN );
    Tcl_LinkVar( interp, "wire", (char *) &wireframe, TCL_LINK_BOOLEAN );
    Tcl_LinkVar( interp, "color", (char *) &color, TCL_LINK_BOOLEAN );
    Tcl_LinkVar( interp, "axes", (char *) &axes, TCL_LINK_BOOLEAN );
    Tcl_LinkVar( interp, "explode", (char *) &explode, TCL_LINK_BOOLEAN);
    Tcl_LinkVar( interp, "dh", (char *) &dh, TCL_LINK_INT);
    Tcl_LinkVar( interp, "econv", (char *) &edge_conv, TCL_LINK_BOOLEAN );
    Tcl_LinkVar( interp, "vfaces", (char *) &vfaces, TCL_LINK_BOOLEAN );
    Tcl_LinkVar( interp, "tcolor", (char *) &tcolor, TCL_LINK_BOOLEAN);
    Tcl_LinkVar( interp, "uleaves", (char *) &uleaves, TCL_LINK_BOOLEAN);
}
#endif


