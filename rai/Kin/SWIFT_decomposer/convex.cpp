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
// convex.C
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
using std::cerr;
#include <stdlib.h>



#include "../SWIFT/SWIFT.h"
#include "../SWIFT/SWIFT_common.h"
#include "../SWIFT/SWIFT_mesh.h"
#include "../SWIFT/SWIFT_mesh_utils.h"

#include "cvxutils.h"
#include "convex.h"
#include "io.h"


const int UNMARK_CUTOFF = 300;

SWIFT_Array<SWIFT_Tri_Edge*[3]> twins;


void Convex_Initialize( SWIFT_Tri_Mesh* m )
{
    int i;
    Convex_Utilities_Initialize( m );

    // Store the mesh's twin info in the twin's list
    twins.Create( m->Num_Faces() );
    for( i = 0; i < m->Num_Faces(); i++ ) {
        twins[i][0] = m->Faces()[i].Edge1().Twin();
        twins[i][1] = m->Faces()[i].Edge2().Twin();
        twins[i][2] = m->Faces()[i].Edge3().Twin();
    }
}

void Create_One_Piece( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
                       SWIFT_Array< SWIFT_Array<int> >& mfs,
                       SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs )
{
    int i;

    piece_ids.Create( m->Num_Faces() );
    mfs.Create( 1 );
    mfs[0].Create( m->Num_Faces() );
    for( i = 0; i < m->Num_Faces(); i++ ) {
        mfs[0][i] = i;
        piece_ids[i] = 0;
    }

    vfs.Create( 1 );
}

int Decompose_DFS( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
                   SWIFT_Array< SWIFT_Array<int> >& mfs,
                   SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs,
                   bool random )
{
    // Start performing DFS on the dual graph maintaining a convex hull along
    // the way.

    cerr << endl << "Starting ";
    if( random ) { 
        cerr << "randomized ";
    }
    cerr << "DFS decomposition" << endl;

    const unsigned int max_faces_in_a_chull = (m->Num_Vertices() - 2) << 1;
    int i, j, k, l, p;
    int created_faces = 0;
    int top, id;
    // The faces stack
    SWIFT_Array<SWIFT_Tri_Face*> sfs;
    // Keeps track of all the faces that were marked as failed so that they can
    // be unmarked efficiently.
    SWIFT_Array<SWIFT_Tri_Face*> mark_failed;
    // The current convex hull
    SWIFT_Array<SWIFT_Tri_Face> chull;
    // Pointers to faces indicating whether the face on the convex hull is a
    // model face or a virtual face (entry is nullptr)
    SWIFT_Array<SWIFT_Tri_Face*> cfs;
    // Which faces on the original model are allowed to be added
    SWIFT_Array<bool> fallowed;
    // Which vertices exist on the convex hull
    SWIFT_Array<bool> cvs;
    // Ids of vertices belonging to the convex hull 
    SWIFT_Array<int> cvs_idx;
    // Ids of faces that are added at each iteration
    SWIFT_Array<int> addedfs;
    // The model face ids that belong to a single convex hull
    SWIFT_Array<int> temp_mfs_1d;
    // The model face ids that belong to each convex hull
    SWIFT_Array< SWIFT_Array<int> > temp_mfs_2d;

    sfs.Create( m->Num_Faces() );
    mark_failed.Create( m->Num_Faces() );
    chull.Create( max_faces_in_a_chull );
    cfs.Create( max_faces_in_a_chull );
    fallowed.Create( m->Num_Faces() );
    cvs.Create( m->Num_Vertices() );
    cvs_idx.Create( m->Num_Vertices() );
    addedfs.Create( m->Num_Faces() );
    temp_mfs_1d.Create( m->Num_Faces() );
    temp_mfs_2d.Create( m->Num_Faces() );

    vfs.Create( m->Num_Faces() );
    piece_ids.Create( m->Num_Faces() );

    Prepare_Mesh_For_Decomposition( m );

    for( i = 0; i < m->Num_Faces(); i++ ) {
        fallowed[i] = true;
    }
    for( i = 0; i < m->Num_Vertices(); i++ ) {
        cvs[i] = false;
    }
    cvs_idx.Set_Length( 0 );

    id = 0;
    for( p = 0; p < m->Num_Faces(); ) {
        // Try to advance p
        for( ; p < m->Num_Faces() && m->Faces()[p].Marked(); p++ );
        if( p == m->Num_Faces() ) break;
        if( random ) {
            // Find a random i in the range [p,m->Num_Faces()-1]
            while( m->Faces()[i = (int) ((SWIFT_Real)(m->Num_Faces()-p) *
                                         drand48()) + p].Marked() );
        } else {
            i = p;
        }

        top = 0;
        sfs[0] = m->Faces()(i);
        mark_failed.Set_Length( 0 );
        temp_mfs_1d.Set_Length( 0 );

        Create_First_Face( m->Faces()(i), chull, cfs );

        // Unset all the vertex membership flags
        for( j = 0; j < cvs_idx.Length(); j++ ) { 
            cvs[cvs_idx[j]] = false;
        }
        cvs_idx.Set_Length( 0 );

        // Mark the first three vertices as added to the hull
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge1().Origin() ) );
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge2().Origin() ) );
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge3().Origin() ) );
        cvs[cvs_idx[0]] = true;
        cvs[cvs_idx[1]] = true;
        cvs[cvs_idx[2]] = true;

        // Add the first face
        piece_ids[i] = id;
        m->Faces()[i].Mark();
        temp_mfs_1d.Add( i );
        l = 1;
        addedfs.Set_Length( 1 );
        addedfs[0] = i;
        fallowed[i] = false;

        while( top != -1 ) {
            if( sfs[top]->Edge1().Marked() &&
                sfs[top]->Edge1().Twin()->Adj_Face()->Unmarked()
            ) {
                if( Add_To_Convex_Hull( m, chull, cfs, fallowed, cvs, addedfs,
                            sfs[top]->Edge1().Twin()->Adj_Face(),
                            sfs[top]->Edge1P(),
                            sfs[top]->Edge1().Twin()->Prev()->Origin() )
                ) {
                    cvs_idx.Add( m->Vertex_Id(
                                 sfs[top]->Edge1().Twin()->Prev()->Origin() ) );
                    sfs[top+1] = sfs[top]->Edge1().Twin()->Adj_Face();
                    top++;
                    // Mark all the faces that were added to the chull
                    for( j = l; j < addedfs.Length(); j++ ) {
                        fallowed[addedfs[j]] = false;
                        if( m->Faces()[addedfs[j]].Unmarked() ) {
                            m->Faces()[addedfs[j]].Mark();
                            piece_ids[addedfs[j]] = id;
                            temp_mfs_1d.Add( addedfs[j] );
                        }
                    }
                    l = addedfs.Length();
                    continue;
                } else {
                    mark_failed.Add( sfs[top]->Edge1().Twin()->Adj_Face() );
                    sfs[top]->Edge1().Twin()->Adj_Face()->Mark();
                    fallowed[m->Face_Id(sfs[top]->Edge1().Twin()->Adj_Face())]
                        = false;
                }
            }
            if( sfs[top]->Edge2().Marked() &&
                sfs[top]->Edge2().Twin()->Adj_Face()->Unmarked()
            ) {
                if( Add_To_Convex_Hull( m, chull, cfs, fallowed, cvs, addedfs,
                            sfs[top]->Edge2().Twin()->Adj_Face(),
                            sfs[top]->Edge2P(),
                            sfs[top]->Edge2().Twin()->Prev()->Origin() )
                ) {
                    cvs_idx.Add( m->Vertex_Id(
                                 sfs[top]->Edge2().Twin()->Prev()->Origin() ) );
                    sfs[top+1] = sfs[top]->Edge2().Twin()->Adj_Face();
                    top++;
                    // Mark all the faces that were added to the chull
                    for( j = l; j < addedfs.Length(); j++ ) {
                        fallowed[addedfs[j]] = false;
                        if( m->Faces()[addedfs[j]].Unmarked() ) {
                            m->Faces()[addedfs[j]].Mark();
                            piece_ids[addedfs[j]] = id;
                            temp_mfs_1d.Add( addedfs[j] );
                        }
                    }
                    l = addedfs.Length();
                    continue;
                } else {
                    mark_failed.Add( sfs[top]->Edge2().Twin()->Adj_Face() );
                    sfs[top]->Edge2().Twin()->Adj_Face()->Mark();
                    fallowed[m->Face_Id(sfs[top]->Edge2().Twin()->Adj_Face())]
                        = false;
                }
            }
            if( sfs[top]->Edge3().Marked() &&
                sfs[top]->Edge3().Twin()->Adj_Face()->Unmarked()
            ) {
                if( Add_To_Convex_Hull( m, chull, cfs, fallowed, cvs, addedfs,
                            sfs[top]->Edge3().Twin()->Adj_Face(),
                            sfs[top]->Edge3P(),
                            sfs[top]->Edge3().Twin()->Prev()->Origin() )
                ) {
                    cvs_idx.Add( m->Vertex_Id(
                                 sfs[top]->Edge3().Twin()->Prev()->Origin() ) );
                    sfs[top+1] = sfs[top]->Edge3().Twin()->Adj_Face();
                    top++;
                    // Mark all the faces that were added to the chull
                    for( j = l; j < addedfs.Length(); j++ ) {
                        fallowed[addedfs[j]] = false;
                        if( m->Faces()[addedfs[j]].Unmarked() ) {
                            m->Faces()[addedfs[j]].Mark();
                            piece_ids[addedfs[j]] = id;
                            temp_mfs_1d.Add( addedfs[j] );
                        }
                    }
                    l = addedfs.Length();
                    continue;
                } else {
                    mark_failed.Add( sfs[top]->Edge3().Twin()->Adj_Face() );
                    sfs[top]->Edge3().Twin()->Adj_Face()->Mark();
                    fallowed[m->Face_Id(sfs[top]->Edge3().Twin()->Adj_Face())]
                        = false;
                }
            }

            top--;
        }

        // Unmark all the failed faces.
        for( j = 0; j < mark_failed.Length(); j++ ) {
            mark_failed[j]->Unmark();
            fallowed[m->Face_Id( mark_failed[j] )] = true;
        }

        // Copy the virtual faces for this piece
        for( j = 0, k = 0; j < chull.Length(); j++ ) {
            if( chull[j].Unmarked() && cfs[j] == nullptr ) {
                k++;
            }
        }
        created_faces += k;

        vfs[id].Create( k );
        for( j = 0, k = 0; j < chull.Length(); j++ ) {
            if( chull[j].Unmarked() && cfs[j] == nullptr ) {
                vfs[id][k].Set_Normal_N( chull[j].Normal() );
                vfs[id][k].Set_Distance( chull[j].Distance() );
                vfs[id][k].Edge1().Set_Direction_N(
                                                chull[j].Edge1().Direction() );
                vfs[id][k].Edge2().Set_Direction_N(
                                                chull[j].Edge2().Direction() );
                vfs[id][k].Edge3().Set_Direction_N(
                                                chull[j].Edge3().Direction() );
                vfs[id][k].Edge1().Set_Length( chull[j].Edge1().Length() );
                vfs[id][k].Edge2().Set_Length( chull[j].Edge2().Length() );
                vfs[id][k].Edge3().Set_Length( chull[j].Edge3().Length() );
                vfs[id][k].Edge1().Set_Origin( chull[j].Edge1().Origin() );
                vfs[id][k].Edge2().Set_Origin( chull[j].Edge2().Origin() );
                vfs[id][k].Edge3().Set_Origin( chull[j].Edge3().Origin() );
                vfs[id][k].Edge1().Set_Twin( chull[j].Edge1().Twin() );
                vfs[id][k].Edge2().Set_Twin( chull[j].Edge2().Twin() );
                vfs[id][k].Edge3().Set_Twin( chull[j].Edge3().Twin() );
                chull[j].Edge1().Twin()->Set_Twin( vfs[id][k].Edge1P() );
                chull[j].Edge2().Twin()->Set_Twin( vfs[id][k].Edge2P() );
                chull[j].Edge3().Twin()->Set_Twin( vfs[id][k].Edge3P() );
                k++;
            }
        }

        // Copy the model faces for this piece
        temp_mfs_2d[id].Copy_Length( temp_mfs_1d );

        id++;
        if( !random ) {
            p++;
        }
    }
    temp_mfs_2d.Set_Length( id );
    vfs.Set_Length( id );

    // Unmark all the faces and edges
    for( i = 0; i < m->Num_Faces(); i++ ) {
        m->Faces()[i].Unmark();
        m->Faces()[i].Edge1().Unmark();
        m->Faces()[i].Edge2().Unmark();
        m->Faces()[i].Edge3().Unmark();
    }

    // Copy the mfs
    mfs.Copy_Length( temp_mfs_2d );
    for( i = 0; i < temp_mfs_2d.Length(); i++ ) {
        temp_mfs_2d[i].Nullify();
    }

    cerr << "Created " << id << " pieces" << endl;
    cerr << "Original faces = " << m->Num_Faces() << endl;
    cerr << "Created virtual faces = " << created_faces << endl << endl;

    return id;
}

int Decompose_BFS( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
                   SWIFT_Array< SWIFT_Array<int> >& mfs,
                   SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs,
                   bool random )
{
    // Start performing BFS on the dual graph maintaining a convex hull along
    // the way.

    cerr << endl << "Starting ";
    if( random ) { 
        cerr << "randomized ";
    }
    cerr << "BFS decomposition" << endl;

    const unsigned int max_faces_in_a_chull = (m->Num_Vertices() - 2) << 1;
    int i, j, k, l, p;
    int created_faces = 0;
    int front, id;
    bool add_children;
    SWIFT_Tri_Edge* e;
    SWIFT_Tri_Vertex* v;
    SWIFT_Array<SWIFT_Tri_Face*> qfs;   // The queue
    SWIFT_Array<SWIFT_Tri_Face*> qfs_parents;
    SWIFT_Array<int> qmap;
    SWIFT_Array<int> qmap_idx;
    SWIFT_Array<SWIFT_Tri_Face*> mark_failed;
    SWIFT_Array<SWIFT_Tri_Face> chull;
    SWIFT_Array<SWIFT_Tri_Face*> cfs;
    SWIFT_Array<bool> fallowed;
    SWIFT_Array<bool> cvs;
    SWIFT_Array<int> cvs_idx;
    SWIFT_Array<int> addedfs;
    SWIFT_Array<int> temp_mfs_1d;
    SWIFT_Array< SWIFT_Array<int> > temp_mfs_2d;

    qfs.Create( m->Num_Faces() );
    qfs_parents.Create( m->Num_Faces() );
    qmap.Create( m->Num_Faces() );
    qmap_idx.Create( m->Num_Faces() );
    mark_failed.Create( m->Num_Faces() );
    chull.Create( max_faces_in_a_chull );
    cfs.Create( max_faces_in_a_chull );
    fallowed.Create( m->Num_Faces() );
    cvs.Create( m->Num_Vertices() );
    cvs_idx.Create( m->Num_Vertices() );
    addedfs.Create( m->Num_Faces() );
    temp_mfs_1d.Create( m->Num_Faces() );
    temp_mfs_2d.Create( m->Num_Faces() );

    vfs.Create( m->Num_Faces() );
    piece_ids.Create( m->Num_Faces() );

    Prepare_Mesh_For_Decomposition( m );

    for( i = 0; i < m->Num_Vertices(); i++ ) {
        cvs[i] = false;
    }
    for( i = 0; i < m->Num_Faces(); i++ ) {
        fallowed[i] = true;
        piece_ids[i] = -1;
        qmap[i] = -1;
    }
    cvs_idx.Set_Length( 0 );
    qmap_idx.Set_Length( 0 );

    id = 0;
    for( p = 0; p < m->Num_Faces(); ) {
        for( ; p < m->Num_Faces() && m->Faces()[p].Marked(); p++ );
        if( p == m->Num_Faces() ) break;
        if( random ) {
            // Find a random i in the range [p,m->Num_Faces()-1]
            while( m->Faces()[i = (int) ((SWIFT_Real)(m->Num_Faces()-p) *
                                         drand48()) + p].Marked() );
        } else {
            i = p;
        }

        // Unset all the qmappings
        for( j = 0; j < qmap_idx.Length(); j++ ) { 
            qmap[qmap_idx[j]] = -1;
        }
        qmap_idx.Set_Length( 0 );
        qfs.Set_Length( 0 );
        qfs_parents.Set_Length( 0 );
        front = 0;

        if( m->Faces()[i].Edge1().Marked() &&
            m->Faces()[i].Edge1().Twin()->Adj_Face()->Unmarked()
        ) {
            j = m->Face_Id( m->Faces()[i].Edge1().Twin()->Adj_Face() );
            qmap_idx.Add( j );
            qmap[j] = qfs.Length();
            qfs.Add( m->Faces()(j) );
            m->Faces()(j)->Mark();
            qfs_parents.Add( m->Faces()(i) );
        }
        if( m->Faces()[i].Edge2().Marked() &&
            m->Faces()[i].Edge2().Twin()->Adj_Face()->Unmarked()
        ) {
            j = m->Face_Id( m->Faces()[i].Edge2().Twin()->Adj_Face() );
            qmap_idx.Add( j );
            qmap[j] = qfs.Length();
            qfs.Add( m->Faces()(j) );
            m->Faces()(j)->Mark();
            qfs_parents.Add( m->Faces()(i) );
        }
        if( m->Faces()[i].Edge3().Marked() &&
            m->Faces()[i].Edge3().Twin()->Adj_Face()->Unmarked()
        ) {
            j = m->Face_Id( m->Faces()[i].Edge3().Twin()->Adj_Face() );
            qmap_idx.Add( j );
            qmap[j] = qfs.Length();
            qfs.Add( m->Faces()(j) );
            m->Faces()(j)->Mark();
            qfs_parents.Add( m->Faces()(i) );
        }

        mark_failed.Set_Length( 0 );
        temp_mfs_1d.Set_Length( 0 );

        Create_First_Face( m->Faces()(i), chull, cfs );

        // Unset all the vertex membership flags
        for( j = 0; j < cvs_idx.Length(); j++ ) { 
            cvs[cvs_idx[j]] = false;
        }
        cvs_idx.Set_Length( 0 );

        // Mark the first three vertices as added to the hull
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge1().Origin() ) );
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge2().Origin() ) );
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge3().Origin() ) );
        cvs[cvs_idx[0]] = true;
        cvs[cvs_idx[1]] = true;
        cvs[cvs_idx[2]] = true;

        // Add the first face
        piece_ids[i] = id;
        m->Faces()[i].Mark();
        temp_mfs_1d.Add( i );
        l = 1;
        addedfs.Set_Length( 1 );
        addedfs[0] = i;
        fallowed[i] = false;


        // The strategy here is a bit different from that of DFS.  Whatever
        // is at the front of the queue is tested for validity and if so, it
        // is added and the unmarked neighbors are placed at the end of the
        // queue.
        while( front < qfs.Length() ) {

            if( qmap[ m->Face_Id( qfs[front] ) ] >= 0 ) {
                if( qfs[front]->Edge1().Twin() != nullptr &&
                    qfs_parents[front] == qfs[front]->Edge1().Twin()->Adj_Face()
                ) {
                    e = qfs[front]->Edge1().Twin();
                    v = qfs[front]->Edge3().Origin();
                } else if( qfs[front]->Edge2().Twin() != nullptr &&
                    qfs_parents[front] == qfs[front]->Edge2().Twin()->Adj_Face()
                ) {
                    e = qfs[front]->Edge2().Twin();
                    v = qfs[front]->Edge1().Origin();
                } else {
                    e = qfs[front]->Edge3().Twin();
                    v = qfs[front]->Edge2().Origin();
                }
                add_children = Add_To_Convex_Hull( m, chull, cfs, fallowed,
                                                   cvs, addedfs,
                                                   qfs[front], e, v );
                if( add_children ) {
                    // Add the face to the current piece
                    cvs_idx.Add( m->Vertex_Id( v ) );
                    // Mark all the faces that were added to the chull
                    for( j = l; j < addedfs.Length(); j++ ) {
                        fallowed[addedfs[j]] = false;
                        m->Faces()[addedfs[j]].Mark();
                        if( piece_ids[addedfs[j]] == -1 ) {
                            // Remove faces that were added if they exist in q
                            if( qmap[addedfs[j]] == -1 ) {
                                qmap_idx.Add( addedfs[j] );
                            }
                            qmap[addedfs[j]] = -2;
                            piece_ids[addedfs[j]] = id;
                            temp_mfs_1d.Add( addedfs[j] );
                        }
                    }
                    l = addedfs.Length();
                }
            } else {
                add_children = true;
            }

            if( add_children ) {
                // Expand the front by adding unmarked neighbors to the queue
                if( qfs[front]->Edge1().Marked() &&
                    qfs[front]->Edge1().Twin()->Adj_Face()->Unmarked()
                ) {
                    j = m->Face_Id( qfs[front]->Edge1().Twin()->Adj_Face() );
                    if( qmap[j] == -2 ) {
                        qmap[j] = -1;
                    } else {
                        qmap[j] = qfs.Length();
                        qmap_idx.Add( j );
                    }
                    qfs.Add( m->Faces()(j) );
                    m->Faces()(j)->Mark();
                    qfs_parents.Add( qfs[front] );
                }
                if( qfs[front]->Edge2().Marked() &&
                    qfs[front]->Edge2().Twin()->Adj_Face()->Unmarked()
                ) {
                    j = m->Face_Id( qfs[front]->Edge2().Twin()->Adj_Face() );
                    if( qmap[j] == -2 ) {
                        qmap[j] = -1;
                    } else {
                        qmap[j] = qfs.Length();
                        qmap_idx.Add( j );
                    }
                    qfs.Add( m->Faces()(j) );
                    m->Faces()(j)->Mark();
                    qfs_parents.Add( qfs[front] );
                }
                if( qfs[front]->Edge3().Marked() &&
                    qfs[front]->Edge3().Twin()->Adj_Face()->Unmarked()
                ) {
                    j = m->Face_Id( qfs[front]->Edge3().Twin()->Adj_Face() );
                    if( qmap[j] == -2 ) {
                        qmap[j] = -1;
                    } else {
                        qmap[j] = qfs.Length();
                        qmap_idx.Add( j );
                    }
                    qfs.Add( m->Faces()(j) );
                    m->Faces()(j)->Mark();
                    qfs_parents.Add( qfs[front] );
                }
            } else {
                mark_failed.Add( qfs[front] );
            }
            front++;
        }

        // Unmark all the failed faces.
        for( j = 0; j < mark_failed.Length(); j++ ) {
            if( fallowed[m->Face_Id(mark_failed[j])] ) {
                mark_failed[j]->Unmark();
            }
        }

        // Copy the virtual faces for this piece
        for( j = 0, k = 0; j < chull.Length(); j++ ) {
            if( chull[j].Unmarked() && cfs[j] == nullptr ) {
                k++;
            }
        }
        created_faces += k;

        vfs[id].Create( k );
        for( j = 0, k = 0; j < chull.Length(); j++ ) {
            if( chull[j].Unmarked() && cfs[j] == nullptr ) {
                vfs[id][k].Set_Normal_N( chull[j].Normal() );
                vfs[id][k].Set_Distance( chull[j].Distance() );
                vfs[id][k].Edge1().Set_Direction_N(
                                                chull[j].Edge1().Direction() );
                vfs[id][k].Edge2().Set_Direction_N(
                                                chull[j].Edge2().Direction() );
                vfs[id][k].Edge3().Set_Direction_N(
                                                chull[j].Edge3().Direction() );
                vfs[id][k].Edge1().Set_Length( chull[j].Edge1().Length() );
                vfs[id][k].Edge2().Set_Length( chull[j].Edge2().Length() );
                vfs[id][k].Edge3().Set_Length( chull[j].Edge3().Length() );
                vfs[id][k].Edge1().Set_Origin( chull[j].Edge1().Origin() );
                vfs[id][k].Edge2().Set_Origin( chull[j].Edge2().Origin() );
                vfs[id][k].Edge3().Set_Origin( chull[j].Edge3().Origin() );
                vfs[id][k].Edge1().Set_Twin( chull[j].Edge1().Twin() );
                vfs[id][k].Edge2().Set_Twin( chull[j].Edge2().Twin() );
                vfs[id][k].Edge3().Set_Twin( chull[j].Edge3().Twin() );
                chull[j].Edge1().Twin()->Set_Twin( vfs[id][k].Edge1P() );
                chull[j].Edge2().Twin()->Set_Twin( vfs[id][k].Edge2P() );
                chull[j].Edge3().Twin()->Set_Twin( vfs[id][k].Edge3P() );
                k++;
            }
        }

        // Copy the model faces for this piece
        temp_mfs_2d[id].Copy_Length( temp_mfs_1d );

        id++;
        if( !random ) {
            p++;
        }
    }
    temp_mfs_2d.Set_Length( id );
    vfs.Set_Length( id );

    // Unmark all the faces and edges
    for( i = 0; i < m->Num_Faces(); i++ ) {
        m->Faces()[i].Unmark();
        m->Faces()[i].Edge1().Unmark();
        m->Faces()[i].Edge2().Unmark();
        m->Faces()[i].Edge3().Unmark();
    }

    // Copy the mfs
    mfs.Copy_Length( temp_mfs_2d );
    for( i = 0; i < temp_mfs_2d.Length(); i++ ) {
        temp_mfs_2d[i].Nullify();
    }

    cerr << "Created " << id << " pieces" << endl;
    cerr << "Original faces = " << m->Num_Faces() << endl;
    cerr << "Created virtual faces = " << created_faces << endl << endl;

    return id;
}

int Decompose_Cresting_BFS( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
                            SWIFT_Array< SWIFT_Array<int> >& mfs,
                            SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs )
{
    // Start performing BFS on the dual graph maintaining a convex hull along
    // the way.

    cerr << endl << "Starting cresting BFS decomposition" << endl;

    const unsigned int max_faces_in_a_chull = (m->Num_Vertices() - 2) << 1;
    int i, j, k, l;
    int created_faces = 0;
    int front, id;
    bool add_children;
    SWIFT_Tri_Edge* e;
    SWIFT_Tri_Vertex* v;
    SWIFT_Array<SWIFT_Tri_Face*> qfs;   // The queue
    SWIFT_Array<SWIFT_Tri_Face*> qfs_parents;
    SWIFT_Array<int> qmap;
    SWIFT_Array<int> qmap_idx;
    SWIFT_Array<SWIFT_Tri_Face*> mark_failed;
    SWIFT_Array<SWIFT_Tri_Face> chull;
    SWIFT_Array<SWIFT_Tri_Face*> cfs;
    SWIFT_Array<bool> fallowed;
    SWIFT_Array<bool> cvs;
    SWIFT_Array<int> cvs_idx;
    SWIFT_Array<int> addedfs;
    SWIFT_Array<int> temp_mfs_1d;
    SWIFT_Array< SWIFT_Array<int> > temp_mfs_2d;

    // The priority queue
    SWIFT_Array<int> lengths( m->Num_Faces() );
    SWIFT_Array<int> bmap( m->Num_Faces() );
    SWIFT_Array<int> fmap( m->Num_Faces() );

    qfs.Create( m->Num_Faces() );
    qfs_parents.Create( m->Num_Faces() );
    qmap.Create( m->Num_Faces() );
    qmap_idx.Create( m->Num_Faces() );
    mark_failed.Create( m->Num_Faces() );
    chull.Create( max_faces_in_a_chull );
    cfs.Create( max_faces_in_a_chull );
    fallowed.Create( m->Num_Faces() );
    cvs.Create( m->Num_Vertices() );
    cvs_idx.Create( m->Num_Vertices() );
    addedfs.Create( m->Num_Faces() );
    temp_mfs_1d.Create( m->Num_Faces() );
    temp_mfs_2d.Create( m->Num_Faces() );

    vfs.Create( m->Num_Faces() );
    piece_ids.Create( m->Num_Faces() );

    Prepare_Mesh_For_Decomposition( m );

    cvs_idx.Set_Length( 0 );
    qmap_idx.Set_Length( 0 );
    for( i = 0; i < m->Num_Vertices(); i++ ) {
        cvs[i] = false;
    }
    for( i = 0; i < m->Num_Faces(); i++ ) {
        fallowed[i] = true;
        piece_ids[i] = -1;
        qmap[i] = -1;
        bmap[i] = fmap[i] = i;
        if( m->Faces()[i].Edge1().Unmarked() ||
            m->Faces()[i].Edge2().Unmarked() ||
            m->Faces()[i].Edge3().Unmarked()
        ) {
            lengths[i] = 0;
            qmap_idx.Add( i );
        } else {
            lengths[i] = -1;
        }
    }

    id = 0;

    // Calculate distances for each face and create priority queue
    if( !qmap_idx.Empty() ) {
        // This is a convex object
        for( i = 0; i < qmap_idx.Max_Length(); i++ ) {
            if( m->Faces()[qmap_idx[i]].Edge1().Twin() != nullptr ) {
                k = m->Face_Id(
                        m->Faces()[qmap_idx[i]].Edge1().Twin()->Adj_Face() );
                if( lengths[k] == -1 ) {
                    lengths[k] = lengths[qmap_idx[i]]+1;
                    qmap_idx.Add( k );
                }
            }
            if( m->Faces()[qmap_idx[i]].Edge2().Twin() != nullptr ) {
                k = m->Face_Id(
                        m->Faces()[qmap_idx[i]].Edge2().Twin()->Adj_Face() );
                if( lengths[k] == -1 ) {
                    lengths[k] = lengths[qmap_idx[i]]+1;
                    qmap_idx.Add( k );
                }
            }
            if( m->Faces()[qmap_idx[i]].Edge3().Twin() != nullptr ) {
                k = m->Face_Id(
                        m->Faces()[qmap_idx[i]].Edge3().Twin()->Adj_Face() );
                if( lengths[k] == -1 ) {
                    lengths[k] = lengths[qmap_idx[i]]+1;
                    qmap_idx.Add( k );
                }
            }
        }

        Build_Heap( lengths, bmap, fmap );
    }
    qmap_idx.Set_Length( 0 );

    // Process the priority queue by doing BFS
    while( !lengths.Empty() ) {
        i = bmap[0];

        // Unset all the qmappings
        for( j = 0; j < qmap_idx.Length(); j++ ) { 
            qmap[qmap_idx[j]] = -1;
        }
        qmap_idx.Set_Length( 0 );
        qfs.Set_Length( 0 );
        qfs_parents.Set_Length( 0 );
        front = 0;

        if( m->Faces()[i].Edge1().Marked() &&
            m->Faces()[i].Edge1().Twin()->Adj_Face()->Unmarked()
        ) {
            j = m->Face_Id( m->Faces()[i].Edge1().Twin()->Adj_Face() );
            qmap_idx.Add( j );
            qmap[j] = qfs.Length();
            qfs.Add( m->Faces()(j) );
            m->Faces()(j)->Mark();
            qfs_parents.Add( m->Faces()(i) );
        }
        if( m->Faces()[i].Edge2().Marked() &&
            m->Faces()[i].Edge2().Twin()->Adj_Face()->Unmarked()
        ) {
            j = m->Face_Id( m->Faces()[i].Edge2().Twin()->Adj_Face() );
            qmap_idx.Add( j );
            qmap[j] = qfs.Length();
            qfs.Add( m->Faces()(j) );
            m->Faces()(j)->Mark();
            qfs_parents.Add( m->Faces()(i) );
        }
        if( m->Faces()[i].Edge3().Marked() &&
            m->Faces()[i].Edge3().Twin()->Adj_Face()->Unmarked()
        ) {
            j = m->Face_Id( m->Faces()[i].Edge3().Twin()->Adj_Face() );
            qmap_idx.Add( j );
            qmap[j] = qfs.Length();
            qfs.Add( m->Faces()(j) );
            m->Faces()(j)->Mark();
            qfs_parents.Add( m->Faces()(i) );
        }

        mark_failed.Set_Length( 0 );
        temp_mfs_1d.Set_Length( 0 );

        Create_First_Face( m->Faces()(i), chull, cfs );

        // Unset all the vertex membership flags
        for( j = 0; j < cvs_idx.Length(); j++ ) { 
            cvs[cvs_idx[j]] = false;
        }
        cvs_idx.Set_Length( 0 );

        // Mark the first three vertices as added to the hull
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge1().Origin() ) );
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge2().Origin() ) );
        cvs_idx.Add( m->Vertex_Id( m->Faces()[i].Edge3().Origin() ) );
        cvs[cvs_idx[0]] = true;
        cvs[cvs_idx[1]] = true;
        cvs[cvs_idx[2]] = true;

        // Add the first face
        piece_ids[i] = id;
        m->Faces()[i].Mark();
        temp_mfs_1d.Add( i );
        l = 1;
        addedfs.Set_Length( 1 );
        addedfs[0] = i;
        fallowed[i] = false;


        // The strategy here is a bit different from that of DFS.  Whatever
        // is at the front of the queue is tested for validity and if so, it
        // is added and the unmarked neighbors are placed at the end of the
        // queue.
        while( front < qfs.Length() ) {

            if( qmap[ m->Face_Id( qfs[front] ) ] >= 0 ) {
                if( qfs[front]->Edge1().Twin() != nullptr &&
                    qfs_parents[front] == qfs[front]->Edge1().Twin()->Adj_Face()
                ) {
                    e = qfs[front]->Edge1().Twin();
                    v = qfs[front]->Edge3().Origin();
                } else if( qfs[front]->Edge2().Twin() != nullptr &&
                    qfs_parents[front] == qfs[front]->Edge2().Twin()->Adj_Face()
                ) {
                    e = qfs[front]->Edge2().Twin();
                    v = qfs[front]->Edge1().Origin();
                } else {
                    e = qfs[front]->Edge3().Twin();
                    v = qfs[front]->Edge2().Origin();
                }
                add_children = Add_To_Convex_Hull( m, chull, cfs, fallowed,
                                                   cvs, addedfs,
                                                   qfs[front], e, v );
                if( add_children ) {
                    // Add the face to the current piece
                    cvs_idx.Add( m->Vertex_Id( v ) );
                    // Mark all the faces that were added to the chull
                    for( j = l; j < addedfs.Length(); j++ ) {
                        fallowed[addedfs[j]] = false;
                        if( piece_ids[addedfs[j]] == -1 ) {
                            // Remove faces that were added if they exist in q
                            if( qmap[addedfs[j]] == -1 ) {
                                qmap_idx.Add( addedfs[j] );
                            }
                            qmap[addedfs[j]] = -2;
                            piece_ids[addedfs[j]] = id;
                            temp_mfs_1d.Add( addedfs[j] );
                            Delete_From_Heap( lengths, bmap, fmap,
                                              fmap[addedfs[j]] );
                        }
                    }
                    l = addedfs.Length();
                }
            } else {
                add_children = true;
            }

            if( add_children ) {
                // Expand the front by adding unmarked neighbors to the queue
                if( qfs[front]->Edge1().Marked() &&
                    qfs[front]->Edge1().Twin()->Adj_Face()->Unmarked()
                ) {
                    j = m->Face_Id( qfs[front]->Edge1().Twin()->Adj_Face() );
                    if( qmap[j] == -2 ) {
                        qmap[j] = -1;
                    } else {
                        qmap[j] = qfs.Length();
                        qmap_idx.Add( j );
                    }
                    qfs.Add( m->Faces()(j) );
                    m->Faces()(j)->Mark();
                    qfs_parents.Add( qfs[front] );
                }
                if( qfs[front]->Edge2().Marked() &&
                    qfs[front]->Edge2().Twin()->Adj_Face()->Unmarked()
                ) {
                    j = m->Face_Id( qfs[front]->Edge2().Twin()->Adj_Face() );
                    if( qmap[j] == -2 ) {
                        qmap[j] = -1;
                    } else {
                        qmap[j] = qfs.Length();
                        qmap_idx.Add( j );
                    }
                    qfs.Add( m->Faces()(j) );
                    m->Faces()(j)->Mark();
                    qfs_parents.Add( qfs[front] );
                }
                if( qfs[front]->Edge3().Marked() &&
                    qfs[front]->Edge3().Twin()->Adj_Face()->Unmarked()
                ) {
                    j = m->Face_Id( qfs[front]->Edge3().Twin()->Adj_Face() );
                    if( qmap[j] == -2 ) {
                        qmap[j] = -1;
                    } else {
                        qmap[j] = qfs.Length();
                        qmap_idx.Add( j );
                    }
                    qfs.Add( m->Faces()(j) );
                    m->Faces()(j)->Mark();
                    qfs_parents.Add( qfs[front] );
                }
            } else {
                mark_failed.Add( qfs[front] );
            }
            front++;
        }

        // Unmark all the failed faces.
        for( j = 0; j < mark_failed.Length(); j++ ) {
            mark_failed[j]->Unmark();
        }

        // Copy the virtual faces for this piece
        for( j = 0, k = 0; j < chull.Length(); j++ ) {
            if( chull[j].Unmarked() && cfs[j] == nullptr ) {
                k++;
            }
        }
        created_faces += k;

        vfs[id].Create( k );
        for( j = 0, k = 0; j < chull.Length(); j++ ) {
            if( chull[j].Unmarked() && cfs[j] == nullptr ) {
                vfs[id][k].Set_Normal_N( chull[j].Normal() );
                vfs[id][k].Set_Distance( chull[j].Distance() );
                vfs[id][k].Edge1().Set_Direction_N(
                                                chull[j].Edge1().Direction() );
                vfs[id][k].Edge2().Set_Direction_N(
                                                chull[j].Edge2().Direction() );
                vfs[id][k].Edge3().Set_Direction_N(
                                                chull[j].Edge3().Direction() );
                vfs[id][k].Edge1().Set_Length( chull[j].Edge1().Length() );
                vfs[id][k].Edge2().Set_Length( chull[j].Edge2().Length() );
                vfs[id][k].Edge3().Set_Length( chull[j].Edge3().Length() );
                vfs[id][k].Edge1().Set_Origin( chull[j].Edge1().Origin() );
                vfs[id][k].Edge2().Set_Origin( chull[j].Edge2().Origin() );
                vfs[id][k].Edge3().Set_Origin( chull[j].Edge3().Origin() );
                vfs[id][k].Edge1().Set_Twin( chull[j].Edge1().Twin() );
                vfs[id][k].Edge2().Set_Twin( chull[j].Edge2().Twin() );
                vfs[id][k].Edge3().Set_Twin( chull[j].Edge3().Twin() );
                chull[j].Edge1().Twin()->Set_Twin( vfs[id][k].Edge1P() );
                chull[j].Edge2().Twin()->Set_Twin( vfs[id][k].Edge2P() );
                chull[j].Edge3().Twin()->Set_Twin( vfs[id][k].Edge3P() );
                k++;
            }
        }

        // Copy the model faces for this piece
        temp_mfs_2d[id].Copy_Length( temp_mfs_1d );

        id++;

        // Remove this face from the priority queue
        Delete_From_Heap( lengths, bmap, fmap, fmap[i] );
    }
    temp_mfs_2d.Set_Length( id );
    vfs.Set_Length( id );

    // Unmark all the faces and edges
    for( i = 0; i < m->Num_Faces(); i++ ) {
        m->Faces()[i].Unmark();
        m->Faces()[i].Edge1().Unmark();
        m->Faces()[i].Edge2().Unmark();
        m->Faces()[i].Edge3().Unmark();
    }

    // Copy the mfs
    mfs.Copy_Length( temp_mfs_2d );
    for( i = 0; i < temp_mfs_2d.Length(); i++ ) {
        temp_mfs_2d[i].Nullify();
    }

    cerr << "Created " << id << " pieces" << endl;
    cerr << "Original faces = " << m->Num_Faces() << endl;
    cerr << "Created virtual faces = " << created_faces << endl << endl;

    return id;
}



void Jitter( SWIFT_Tri_Mesh* m, SWIFT_Real jampl )
{
    int i;
    for( i = 0; i < m->Num_Vertices(); i++ ) {
        // Perturb the coordinates
        m->Vertices()[i].Set_Coords(
                        m->Vertices()[i].Coords().X() + drand48() * jampl,
                        m->Vertices()[i].Coords().Y() + drand48() * jampl,
                        m->Vertices()[i].Coords().Z() + drand48() * jampl );
    }
}

void Edge_Flip( SWIFT_Tri_Mesh* m, SWIFT_Real etol )
{
    const SWIFT_Real etol_sq = etol*etol;
    int i;
    SWIFT_Tri_Face f;
    SWIFT_Array<SWIFT_Tri_Edge*> fedges;

    // Have all the convex edges marked
    Prepare_Mesh_For_Decomposition( m );

    // Traverse all the faces, marking edges that are not allowed to be flipped
    for( i = 0; i < m->Num_Faces(); i++ ) {
        if( m->Faces()[i].Edge1().Unmarked() ) {
            // May be able to flip this edge

            // Build the proposed edge
            f.Edge1().Set_Origin( m->Faces()[i].Edge3().Origin() );
            f.Edge2().Set_Origin(
                            m->Faces()[i].Edge1().Twin()->Prev()->Origin() );
            f.Edge1().Compute_Direction_Length();
            if( Edge_Flip_Allowed( m->Faces()[i].Edge1P(), f.Edge1P(), etol_sq )
            ) {
                fedges.Add_Grow( m->Faces()[i].Edge1P(), 100 );
                // Have to mark other edges in these two faces as well as
                // their twins
                m->Faces()[i].Edge2().Mark();
                m->Faces()[i].Edge2().Twin()->Mark();
                m->Faces()[i].Edge3().Mark();
                m->Faces()[i].Edge3().Twin()->Mark();

                m->Faces()[i].Edge1().Twin()->Next()->Mark();
                m->Faces()[i].Edge1().Twin()->Next()->Twin()->Mark();
                m->Faces()[i].Edge1().Twin()->Prev()->Mark();
                m->Faces()[i].Edge1().Twin()->Prev()->Twin()->Mark();
            }
            m->Faces()[i].Edge1().Mark();
            m->Faces()[i].Edge1().Twin()->Mark();
        }
        if( m->Faces()[i].Edge2().Unmarked() ) {
            // May be able to flip this edge

            // Build the proposed edge
            f.Edge1().Set_Origin( m->Faces()[i].Edge1().Origin() );
            f.Edge2().Set_Origin(
                            m->Faces()[i].Edge2().Twin()->Prev()->Origin() );
            f.Edge1().Compute_Direction_Length();
            if( Edge_Flip_Allowed( m->Faces()[i].Edge2P(), f.Edge1P(), etol_sq )
            ) {
                fedges.Add_Grow( m->Faces()[i].Edge2P(), 100 );
                m->Faces()[i].Edge1().Mark();
                m->Faces()[i].Edge1().Twin()->Mark();
                m->Faces()[i].Edge3().Mark();
                m->Faces()[i].Edge3().Twin()->Mark();

                m->Faces()[i].Edge2().Twin()->Next()->Mark();
                m->Faces()[i].Edge2().Twin()->Next()->Twin()->Mark();
                m->Faces()[i].Edge2().Twin()->Prev()->Mark();
                m->Faces()[i].Edge2().Twin()->Prev()->Twin()->Mark();
            }
            m->Faces()[i].Edge2().Mark();
            m->Faces()[i].Edge2().Twin()->Mark();
        }
        if( m->Faces()[i].Edge3().Unmarked() ) {
            // May be able to flip this edge

            // Build the proposed edge
            f.Edge1().Set_Origin( m->Faces()[i].Edge2().Origin() );
            f.Edge2().Set_Origin(
                            m->Faces()[i].Edge3().Twin()->Prev()->Origin() );
            f.Edge1().Compute_Direction_Length();
            if( Edge_Flip_Allowed( m->Faces()[i].Edge3P(), f.Edge1P(), etol_sq )
            ) {
                fedges.Add_Grow( m->Faces()[i].Edge3P(), 100 );
                m->Faces()[i].Edge1().Mark();
                m->Faces()[i].Edge1().Twin()->Mark();
                m->Faces()[i].Edge2().Mark();
                m->Faces()[i].Edge2().Twin()->Mark();

                m->Faces()[i].Edge3().Twin()->Next()->Mark();
                m->Faces()[i].Edge3().Twin()->Next()->Twin()->Mark();
                m->Faces()[i].Edge3().Twin()->Prev()->Mark();
                m->Faces()[i].Edge3().Twin()->Prev()->Twin()->Mark();
            }
            m->Faces()[i].Edge3().Mark();
            m->Faces()[i].Edge3().Twin()->Mark();
        }
    }

    cerr << "Flipped " << fedges.Length() << " edges" << endl;

    // Do the actual swaps
    for( i = 0; i < fedges.Length(); i++ ) {
        SWIFT_Tri_Edge* t1 = fedges[i]->Prev()->Twin();
        SWIFT_Tri_Edge* t2 = fedges[i]->Twin()->Prev()->Twin();

        // Set the origins
        fedges[i]->Set_Origin( fedges[i]->Twin()->Prev()->Origin() );
        fedges[i]->Twin()->Set_Origin( fedges[i]->Prev()->Origin() );

        // Set the flipped edge twins
        fedges[i]->Prev()->Set_Twin( fedges[i]->Twin()->Prev() );
        fedges[i]->Twin()->Prev()->Set_Twin( fedges[i]->Prev() );

        // Set the lengths, directions and twins
        fedges[i]->Twin()->Set_Length( t1->Length() );
        fedges[i]->Twin()->Set_Direction_N( -t1->Direction() );
        fedges[i]->Twin()->Set_Twin( t1 );
        t1->Set_Twin( fedges[i]->Twin() );

        fedges[i]->Set_Length( t2->Length() );
        fedges[i]->Set_Direction_N( -t2->Direction() );
        fedges[i]->Set_Twin( t2 );
        t2->Set_Twin( fedges[i] );

        // Compute the lengths and directions of the flipped edge
        fedges[i]->Prev()->Compute_Direction_Length_Twin();

        // Compute the face normals using the two "good" edges
        fedges[i]->Adj_Face()->Compute_Plane_From_Edges( fedges[i] );
        fedges[i]->Prev()->Twin()->Adj_Face()->Compute_Plane_From_Edges(
                                fedges[i]->Prev()->Twin()->Next() );

        // Set the adjacent edges for the vertices
        fedges[i]->Origin()->Set_Adj_Edge( fedges[i] );
        fedges[i]->Next()->Origin()->Set_Adj_Edge( fedges[i]->Next() );
        fedges[i]->Prev()->Origin()->Set_Adj_Edge( fedges[i]->Prev() );
        fedges[i]->Prev()->Twin()->Prev()->Origin()->Set_Adj_Edge(
                                        fedges[i]->Prev()->Twin()->Prev() );
    }
    Prepare_Mesh_For_Decomposition( m );
}


