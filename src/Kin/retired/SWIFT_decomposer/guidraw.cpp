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
// guidraw.C
//
//////////////////////////////////////////////////////////////////////////////


#ifdef WIN32
#include <windows.h>
#endif
#ifdef DECOMP_GRAPHICS
#include <GL/gl.h>
#include <GL/glu.h>
#include <togl.h>

#include "../SWIFT/SWIFT.h"
#include "../SWIFT/SWIFT_mesh.h"

#include "guidraw.h"


extern SWIFT_Tri_Mesh* mesh;

#define RED 0
#define GREEN 1
#define BLUE 2
#define CYAN 3
#define YELLOW 4
#define MAGENTA 5
#define GREY 6
#define LIGHT_GREY 7
#define DARK_GREY 8

static const float colors[9][3] = { {0.6, 0.0, 0.0},
                                    {0.0, 0.6, 0.0},
                                    {0.0, 0.0, 0.6},
                                    {0.0, 0.6, 0.6},
                                    {0.6, 0.6, 0.0},
                                    {0.6, 0.0, 0.6},
                                    {0.6, 0.6, 0.6},
                                    {0.8, 0.8, 0.8},
                                    {0.4, 0.4, 0.4} };

// Color setting routines
inline void Set_Color( int which )
{
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colors[which] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, colors[which] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, 2.0 );

    glColor3f( colors[which][0], colors[which][1], colors[which][2] );
}

inline void Set_Selection_Color( )
{
    Set_Color( MAGENTA );
}

inline void Set_Convex_Color( )
{
    Set_Color( GREEN );
}

inline void Set_Concave_Color( )
{
    Set_Color( RED );
}

inline void Set_Point_Color( )
{
    Set_Color( YELLOW );
}

inline void Set_Point2_Color( )
{
    Set_Color( MAGENTA );
}

inline void Set_Point3_Color( )
{
    Set_Color( CYAN );
}

inline void Set_Contained_Virtual_Color( )
{
    Set_Color( DARK_GREY );
}

inline void Set_Free_Virtual_Color( )
{
    Set_Color( LIGHT_GREY );
}

inline void Set_Virtual_Color( )
{
    Set_Color( GREY );
}

inline void Set_Original_Color( )
{
    Set_Color( GREEN );
}

inline void Set_Contained_Color( )
{
    Set_Color( YELLOW );
}

inline void Set_Free_Color( )
{
    Set_Color( RED );
}

// Drawing routines
void Draw_Triangle_Mesh( SWIFT_Tri_Mesh* m, bool color
                         , SWIFT_Array<int>& piece_ids )
{
    int i;

    if( m != nullptr ) {
        glBegin( GL_TRIANGLES );
        if( color ) {
            for( i = 0; i < m->Faces().Length(); i++ ) {
                Set_Color( piece_ids[i] % 6 );
                m->Faces()[i].Draw();
            }
        } else {
            for( i = 0; i < m->Faces().Length(); i++ ) {
                m->Faces()[i].Draw();
            }
        }
        glEnd();
    }
}

void Draw_Vertices( SWIFT_Tri_Mesh* m )
{
    if( m != nullptr ) {
        int i;
        glDisable( GL_LIGHTING );
        glPointSize( 5.0 );
        Set_Selection_Color( );
        glBegin( GL_POINTS );
        for( i = 0; i < m->Vertices().Length(); i++ ) {
            m->Vertices()[i].Draw();
        }
        glEnd();
        glPointSize( 1.0 );
        glEnable( GL_LIGHTING );
    }
}

void Draw_Edge_Convexity( SWIFT_Tri_Mesh* m, SWIFT_Array<bool>& ecs )
{
    int i, j;

    if( m != nullptr ) {
        glDisable( GL_LIGHTING );
        glLineWidth( 2.0 );

        glBegin( GL_LINES );
        for( i = 0, j = 0; j < m->Faces().Length(); j++, i += 3 ) {
            if( ecs[i] ) {
                Set_Convex_Color( );
            } else {
                Set_Concave_Color( );
            }
            m->Faces()[j].Edge1().Draw();
            if( ecs[i+1] ) {
                Set_Convex_Color( );
            } else {
                Set_Concave_Color( );
            }
            m->Faces()[j].Edge2().Draw();
            if( ecs[i+2] ) {
                Set_Convex_Color( );
            } else {
                Set_Concave_Color( );
            }
            m->Faces()[j].Edge3().Draw();
        }
        glEnd();

        glLineWidth( 1.0 );
        glEnable( GL_LIGHTING );
    }
}

void Draw_Convex_Pieces( SWIFT_Tri_Mesh* m, bool color, bool vfaces, bool tcol,
                         bool explode, SWIFT_Triple& expl_cen,
                         SWIFT_Array<SWIFT_Triple>& coms,
                         SWIFT_Array<int>& piece_ids,
                         SWIFT_Array< SWIFT_Array<int> >& mfs,
                         SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs,
                         SWIFT_Array<int>& wcps )
{
    int i, j;

    if( m != nullptr ) {
        glMatrixMode( GL_MODELVIEW );
        for( i = 0; i < wcps.Length(); i++ ) {
            if( explode ) {
                SWIFT_Triple trans = coms[wcps[i]] - expl_cen;
                glPushMatrix();
                glTranslated( trans.X(), trans.Y(), trans.Z() );
            }
            glBegin( GL_TRIANGLES );

            if( color ) {
                if( tcol ) {
                    Set_Original_Color();
                } else {
                    Set_Color( piece_ids[ mfs[wcps[i]][0] ]%6 );
                }
            }
            for( j = 0; j < mfs[wcps[i]].Length(); j++ ) {
//fs[mfs[wcps[i]][j]] = 1;
                m->Faces()[ mfs[wcps[i]][j] ].Draw();
            }

            if( vfaces ) {
                if( color ) {
                    if( tcol ) {
                        Set_Contained_Color();
                    } else {
                        Set_Virtual_Color();
                    }
                }
                for( j = 0; j < vfs[wcps[i]].Length(); j++ ) {
                    vfs[wcps[i]][j].Draw();
                }
            }

            glEnd();
            if( explode ) {
                glPopMatrix();
            }
        }
    }
}

void Draw_Hierarchy( SWIFT_Tri_Mesh* m, bool color, bool vfaces, bool tcol,
                     bool explode, SWIFT_Triple& expl_cen,
                     SWIFT_Array<int>& piece_ids,
                     SWIFT_Array<SWIFT_BV*>& which_pieces )
{
    int i, j;

    glMatrixMode( GL_MODELVIEW );
    for( i = 0; i < which_pieces.Length(); i++ ) {
        glPushMatrix();
        if( explode ) {
            SWIFT_Triple trans = which_pieces[i]->Center_Of_Mass() - expl_cen;
            glTranslated( trans.X(), trans.Y(), trans.Z() );
        }

        glBegin( GL_TRIANGLES );
        if( color ) {
            if( vfaces ) {
                if( tcol ) {
                    for( j = 0; j < which_pieces[i]->Other_Faces().Length();
                         j++
                    ) {
                        if( which_pieces[i]->Other_Faces()[j]->Is_Original() ) {
                            Set_Original_Color();
                        } else if( which_pieces[i]->Other_Faces()[j]->
                                                            Is_Contained()
                        ) {
                            Set_Contained_Color();
                        } else {
                            Set_Free_Color();
                        }
                        which_pieces[i]->Other_Faces()[j]->Draw();
                    }
                } else {
                    for( j = 0; j < which_pieces[i]->Other_Faces().Length();
                         j++
                    ) {
                        if( which_pieces[i]->Other_Faces()[j]->Is_Original() ) {
                            if( m->Face_In_Range(
                                        which_pieces[i]->Other_Faces()[j] )
                            ) {
                                Set_Color( piece_ids[ m->Face_Id(
                                    which_pieces[i]->Other_Faces()[j] ) ] % 6 );
                            } else {
                                if( which_pieces[i]->Other_Faces()[j]->
                                    Edge1().Twin() != nullptr
                                ) {
                                    Set_Color( piece_ids[ m->Face_Id(
                                        which_pieces[i]->Other_Faces()[j]->
                                        Edge1().Twin()->Twin()->Adj_Face() ) ]
                                                % 6 );
                                } else if( which_pieces[i]->Other_Faces()[j]->
                                           Edge2().Twin() != nullptr
                                ) {
                                    Set_Color( piece_ids[ m->Face_Id(
                                        which_pieces[i]->Other_Faces()[j]->
                                        Edge2().Twin()->Twin()->Adj_Face() ) ]
                                                % 6 );
                                } else if( which_pieces[i]->Other_Faces()[j]->
                                           Edge3().Twin() != nullptr
                                ) {
                                    Set_Color( piece_ids[ m->Face_Id(
                                        which_pieces[i]->Other_Faces()[j]->
                                        Edge3().Twin()->Twin()->Adj_Face() ) ]
                                                % 6 );
                                }
                            }
                        } else if( which_pieces[i]->Other_Faces()[j]->
                                                            Is_Contained()
                        ) {
                            Set_Contained_Virtual_Color();
                        } else {
                            Set_Free_Virtual_Color();
                        }
                        which_pieces[i]->Other_Faces()[j]->Draw();
                    }
                }
            } else {
                if( tcol ) {
                    Set_Original_Color();
                    for( j = 0; j < which_pieces[i]->Other_Faces().Length();
                         j++
                    ) {
                        if( which_pieces[i]->Other_Faces()[j]->Is_Original() ) {
                            which_pieces[i]->Other_Faces()[j]->Draw();
                        }
                    }
                } else {
                    for( j = 0; j < which_pieces[i]->Other_Faces().Length();
                         j++
                    ) {
                        if( which_pieces[i]->Other_Faces()[j]->Is_Original() ) {
                            Set_Color( piece_ids[ m->Face_Id(
                                    which_pieces[i]->Other_Faces()[j] ) ] % 6 );
                            which_pieces[i]->Other_Faces()[j]->Draw();
                        }
                    }
                }
            }
        } else {
            for( j = 0; j < which_pieces[i]->Other_Faces().Length(); j++ ) {
                which_pieces[i]->Other_Faces()[j]->Draw();
            }
        }

        if( vfaces ) {
            if( color ) {
                if( tcol ) {
                    if( which_pieces[i]->Is_Leaf() ) {
                        Set_Contained_Color();
                    } else {
                        Set_Free_Color();
                    }
                } else {
                    if( which_pieces[i]->Is_Leaf() ) {
                        Set_Contained_Virtual_Color();
                    } else {
                        Set_Free_Virtual_Color();
                    }
                }
            }
            // Draw the free faces of this piece
            for( j = 0; j < which_pieces[i]->Faces().Length(); j++ ) {
                which_pieces[i]->Faces()[j].Draw();
            }
        }
        glEnd();
        glPopMatrix();
    }
}


#endif


