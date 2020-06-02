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
// cvxutils.C
//
//////////////////////////////////////////////////////////////////////////////


#include <RAPID.h>
#include "../SWIFT/SWIFT.h"
#include "../SWIFT/SWIFT_mesh.h"
#include "../SWIFT/SWIFT_mesh_utils.h"

#include "cvxutils.h"
using std::cerr;

RAPID_model* rm1 = nullptr;
RAPID_model* rm2 = nullptr;
SWIFT_Array<bool> exclude;
SWIFT_Array<int> exclude_idx;


void Convex_Utilities_Initialize( SWIFT_Tri_Mesh* m )
{
    int i;

    // Build the rapid model required for intersection testing during
    // preprocessing
    rm1 = new RAPID_model;
    rm2 = new RAPID_model;

    rm1->BeginModel();
    for( i = 0; i < m->Num_Faces(); i++ ) {
        rm1->AddTri( m->Faces()[i].Edge1().Origin()->Coords().Value(),
                     m->Faces()[i].Edge2().Origin()->Coords().Value(),
                     m->Faces()[i].Edge3().Origin()->Coords().Value(), i );
    }
    rm1->EndModel();

    // Set up the exclusion array
    exclude.Create( m->Num_Faces() );
    for( i = 0; i < m->Num_Faces(); i++ ) {
        exclude[i] = false;
    }
    exclude_idx.Create( 1000 );
    exclude_idx.Set_Length( 0 );
}

void Prepare_Mesh_For_Decomposition( SWIFT_Tri_Mesh* m )
{
    int i;

    // Unmark all the edges
    for( i = 0; i < m->Num_Faces(); i++ ) {
        m->Faces()[i].Edge1().Unmark();
        m->Faces()[i].Edge2().Unmark();
        m->Faces()[i].Edge3().Unmark();
    }
    // Compute the edge convexities (local convexity)
    for( i = 0; i < m->Num_Faces(); i++ ) {

        if( m->Faces()[i].Edge1().Twin() != nullptr && m->Faces()[i].Inside(
                            m->Faces()[i].Edge1().Twin()->Prev()->Origin() )
        ) {
            m->Faces()[i].Edge1().Mark();
            m->Faces()[i].Edge1().Twin()->Mark();
        }
        if( m->Faces()[i].Edge2().Twin() != nullptr && m->Faces()[i].Inside(
                    m->Faces()[i].Edge2().Twin()->Prev()->Origin() )
        ) {
            m->Faces()[i].Edge2().Mark();
            m->Faces()[i].Edge2().Twin()->Mark();
        }
        if( m->Faces()[i].Edge3().Twin() != nullptr && m->Faces()[i].Inside(
                    m->Faces()[i].Edge3().Twin()->Prev()->Origin() )
        ) {
            m->Faces()[i].Edge3().Mark();
            m->Faces()[i].Edge3().Twin()->Mark();
        }
    }
}

bool Intersect_Edge( SWIFT_Tri_Face& f,
                     SWIFT_Tri_Vertex* v1, SWIFT_Tri_Vertex* v2 )
{
    const SWIFT_Real dtf = f.Distance( v1 );
    const SWIFT_Real dhf = f.Distance( v2 );

    if( (dtf >= 0.0 || dhf >= 0.0 ) && (dtf < 0.0 || dhf < 0.0 ) ) {
        SWIFT_Real dt, dh;
        SWIFT_Real lambda;
        SWIFT_Real lambda_t = 0.0;
        SWIFT_Real lambda_h = 1.0;
        dt = f.Edge1().Face_Distance( v1 );
        dh = f.Edge1().Face_Distance( v2 );
        if( dt < 0.0 && dh < 0.0 ) {
            // Edge is fully clipped out by this edge
            return false;
        } else if( dt < 0.0 ) {
            lambda_t = dt / (dt - dh);
        } else if( dh < 0.0 ) {
            lambda_h = dt / (dt - dh);
        }
        dt = f.Edge2().Face_Distance( v1 );
        dh = f.Edge2().Face_Distance( v2 );
        if( dt < 0.0 && dh < 0.0 ) {
            // Edge is fully clipped out by this edge
            return false;
        } else if( dt < 0.0 ) {
            lambda = dt / (dt - dh);
            lambda_t = SWIFT_max( lambda, lambda_t );
        } else if( dh < 0.0 ) {
            lambda = dt / (dt - dh);
            lambda_h = SWIFT_min( lambda, lambda_h );
        }
        if( lambda_t > lambda_h ) {
            // Edge is fully clipped out
            return false;
        }
        dt = f.Edge3().Face_Distance( v1 );
        dh = f.Edge3().Face_Distance( v2 );
        if( dt < 0.0 && dh < 0.0 ) {
            // Edge is fully clipped out by this edge
            return false;
        } else if( dt < 0.0 ) {
            lambda = dt / (dt - dh);
            lambda_t = SWIFT_max( lambda, lambda_t );
        } else if( dh < 0.0 ) {
            lambda = dt / (dt - dh);
            lambda_h = SWIFT_min( lambda, lambda_h );
        }
        if( lambda_t > lambda_h ) {
            // Edge is fully clipped out
            return false;
        }
        lambda = dtf / (dtf - dhf);
        if( lambda > lambda_t && lambda < lambda_h ) {
            return true;
        }
    }

    return false;
}

bool Test_For_Intersection( SWIFT_Tri_Mesh* m, int f, int v,
                            SWIFT_Array<int>& ex )
{
    int i;

    rm2->BeginModel();
    rm2->AddTri( m->Faces()[f].Edge2().Origin()->Coords().Value(),
                 m->Faces()[f].Edge1().Origin()->Coords().Value(),
                 m->Vertices()[v].Coords().Value(), 1 );
    rm2->AddTri( m->Faces()[f].Edge1().Origin()->Coords().Value(),
                 m->Faces()[f].Edge3().Origin()->Coords().Value(),
                 m->Vertices()[v].Coords().Value(), 2 );
    rm2->AddTri( m->Faces()[f].Edge3().Origin()->Coords().Value(),
                 m->Faces()[f].Edge2().Origin()->Coords().Value(),
                 m->Vertices()[v].Coords().Value(), 3 );
    rm2->EndModel();

    for( i = 0; i < ex.Length(); i++ ) {
        exclude[ex[i]] = true;
    }

    RAPID_Collide( rm1, rm2, exclude.Data(), RAPID_FIRST_CONTACT );

    for( i = 0; i < ex.Length(); i++ ) {
        exclude[ex[i]] = false;
    }

    return RAPID_num_contacts > 0;
}


// Quicksort for the convex hull sanity checking
void Quicksort( int* es, int p, int r )
{
    if( p < r ) {
        // Compute a random element to use as the pivot
        int rn = (int) ((SWIFT_Real)(r-p+1) * drand48()) + p;
        int i = p-1;
        int j = r+1;
        int x = es[rn];
        int tes;

        // Swap the random element into the first position
        tes = es[rn];
        es[rn] = es[p];
        es[p] = tes;

        while( true ) {
            for( j--; es[j] > x; j-- );
            for( i++; es[i] < x; i++ );
            if( i < j ) {
                tes = es[i];
                es[i] = es[j];
                es[j] = tes;
            } else {
                break;
            }
        }

        Quicksort( es, p, j );
        Quicksort( es, j+1, r );
    }
}

// Checks whether the mesh given by the triangular faces fs is closed and that
// it includes all vertices numbered 0 through vn-1.
bool Is_Convex_Hull_Sane( int vn, int* fs, int fn )
{
    int i, j;
    bool* occ_map = new bool[vn];
    int* edge_map = new int[6*fn];

    // Initialize the occurence map to nothing occurring
    for( i = 0; i < vn; i++ ) {
        occ_map[i] = false;
    }

    // Insert the edges and their inverses into the edge list.  Also check that
    // none of the edges is null (v1-->v1).
    for( i = 0, j = 0; i < fn*3; ) {
        occ_map[fs[i]] = true;
        if( fs[i] == fs[i+1] || fs[i] == fs[i+2] || fs[i+1] == fs[i+2] ) {
            delete [] occ_map; delete [] edge_map;
            return false;
        }
        edge_map[j] = (fs[i] << 16) | (0xffff & fs[i+1]);
        j++;
        edge_map[j] = (fs[i+1] << 16) | (0xffff & fs[i]);
        i++; j++;
        occ_map[fs[i]] = true;
        edge_map[j] = (fs[i] << 16) | (0xffff & fs[i+1]);
        j++;
        edge_map[j] = (fs[i+1] << 16) | (0xffff & fs[i]);
        i++; j++;
        occ_map[fs[i]] = true;
        edge_map[j] = (fs[i] << 16) | (0xffff & fs[i-2]);
        j++;
        edge_map[j] = (fs[i-2] << 16) | (0xffff & fs[i]);
        i++; j++;
    }

    // Check the vertex occurence
    for( i = 0; i < vn; i++ ) {
        if( !occ_map[i] ) {
            delete [] occ_map; delete [] edge_map;
            return false;
        }
    }

    // Sort the edge map and the reverse edge map
    Quicksort( edge_map, 0, fn*6-1 );

    // Make sure that there are pairs of edges in the list
    for( i = 0; i < fn*6; i += 2 ) {
        if( edge_map[i] != edge_map[i+1] ) {
            delete [] occ_map; delete [] edge_map;
            return false;
        }
    }

    // Now ensure that there are consecutive edges in the list
    for( i = 0, j = 1; i < vn; i++ ) {
        edge_map[0] = (i << 16) | (0xffff & (i==vn-1 ? 0 : i+1));
        for( ; edge_map[0] != edge_map[j] && j < fn*6; j += 2 );
//{ cerr << "j = " << j << endl;
        if( j >= fn*6 ) {
            // Did not find this edge
            delete [] occ_map; delete [] edge_map;
            return false;
        }
    }

    delete [] occ_map; delete [] edge_map;
    return true;
}

// The piece that this function creates is contained (trivially) within the
// original model provided that the original model does not intersect itself.
void Create_First_Face( SWIFT_Tri_Face* f,
                        SWIFT_Array<SWIFT_Tri_Face>& chull,
                        SWIFT_Array<SWIFT_Tri_Face*>& cfs )
{
    chull.Set_Length( 2 );
    cfs.Set_Length( 2 );
    cfs[0] = f;
    cfs[1] = nullptr;
    chull[0] = *f;
    chull[0].Reset_Internal_Edge_Pointers();

    // Unmark the first 2 faces to indicate that they are valid
    chull[0].Unmark();
    chull[1].Unmark();

    // Unmark the edges to indicate that they are not on visible faces
    chull[0].Edge1().Unmark();
    chull[0].Edge2().Unmark();
    chull[0].Edge3().Unmark();
    chull[1].Edge1().Unmark();
    chull[1].Edge2().Unmark();
    chull[1].Edge3().Unmark();

    // Point to the vertices
    chull[1].Edge1().Set_Origin( chull[0].Edge2().Origin() );
    chull[1].Edge2().Set_Origin( chull[0].Edge1().Origin() );
    chull[1].Edge3().Set_Origin( chull[0].Edge3().Origin() );

    // Set the twins
    chull[0].Edge1().Set_Twin( chull[1].Edge1P() );
    chull[0].Edge2().Set_Twin( chull[1].Edge3P() );
    chull[0].Edge3().Set_Twin( chull[1].Edge2P() );
    chull[1].Edge1().Set_Twin( chull[0].Edge1P() );
    chull[1].Edge2().Set_Twin( chull[0].Edge3P() );
    chull[1].Edge3().Set_Twin( chull[0].Edge2P() );

    // Compute the edge geometries and the face planes
    chull[1].Set_Normal_N( -chull[0].Normal() );
    chull[1].Set_Distance( -chull[0].Distance() );
}

// Add a face to the current piece.  f, e, and v are the added face, edge that
// is crossed to add the new face, and the vertex at the far side of the face.
//
// The newly added face cannot cause any of the already added faces to be
// deleted from the convex hull to maintain the convexity of this piece.
//
// If the containment constraint is enforced, then the newly added points
// must not cause the new convex hull to go outside of the original model.
//
// The second parameter is the full convex hull that is currently
// being maintained.  The third is a list of face pointers which are nullptr if
// the corresponding face is a virtual face otherwise the pointer points to the
// corresponding model face.
//
// Face marking is used to indicate if the face actually exists.
// Edge marking is used to indicate that the edge is on the horizon.
bool Add_To_Convex_Hull( SWIFT_Tri_Mesh* m,
                         SWIFT_Array<SWIFT_Tri_Face>& chull,
                         SWIFT_Array<SWIFT_Tri_Face*>& cfs,
                         SWIFT_Array<bool>& fallowed,
                         SWIFT_Array<bool>& cvs,
                         SWIFT_Array<int>& addedfs,
                         SWIFT_Tri_Face* f,
                         SWIFT_Tri_Edge* e,
                         SWIFT_Tri_Vertex* v )
{

    if( !fallowed[ m->Face_Id( f ) ] || cvs[ m->Vertex_Id( v ) ] ) {
        return false;
    }
    int i, j, k, l;
    int old_afs_len;
    bool vis;
    SWIFT_Tri_Edge* e1;
    SWIFT_Tri_Edge* e2;
    SWIFT_Tri_Edge* e3;
    SWIFT_Array<int> vis_faces;
    SWIFT_Array<SWIFT_Tri_Edge*> hor_edges;
    SWIFT_Array<SWIFT_Tri_Face*> orig_faces;
    SWIFT_Array<SWIFT_Tri_Face> new_faces;

    vis_faces.Create( chull.Length() );
    vis_faces.Set_Length( 0 );


    // Mark faces visible from v and reject if one of the faces on the model
    // is visible because that would mean that the global convexity constraint
    // has been violated.
    vis = false;
    j = 0;
    for( i = 0; i < chull.Length(); i++ ) {
        // Only look at those faces which exist (unmarked).
        if( chull[i].Unmarked() ) {

            // The vertex is not already on the convex hull

            // Check to see if this face is visible
            if( chull[i].Outside( v ) ) {
                // If the face is on the model then reject since we cannot
                // destroy a model face (violates global convexity).
                if( cfs[i] != nullptr ) {
                    // Undo the marking on the edges
                    for( j = 0; j < vis_faces.Length(); j++ ) {
                        chull[vis_faces[j]].Edge1().Unmark();
                        chull[vis_faces[j]].Edge2().Unmark();
                        chull[vis_faces[j]].Edge3().Unmark();
                    }
                    return false;
                }

                // Global convexity is not violated by this face
                chull[i].Edge1().Mark();
                chull[i].Edge2().Mark();
                chull[i].Edge3().Mark();
                vis_faces.Add( i );
                vis = true;
            }
        }
    }

    // If no faces are visible from v, then v is inside the hull: reject
    if( !vis ) {
        // Undo the marking on the edges
        for( j = 0; j < vis_faces.Length(); j++ ) {
            chull[vis_faces[j]].Edge1().Unmark();
            chull[vis_faces[j]].Edge2().Unmark();
            chull[vis_faces[j]].Edge3().Unmark();
        }
        return false;
    }

    hor_edges.Create( chull.Length()*3 );

    // Scan the edges of the visible faces and determine if they are on the
    // horizon.  Place their twins in the list.
    k = 0;
    vis = false;
    for( i = 0; i < vis_faces.Length(); i++ ) {
        if( chull[vis_faces[i]].Edge1().Twin()->Unmarked() ) {
            if( chull[vis_faces[i]].Edge2().Origin() == e->Origin() &&
                chull[vis_faces[i]].Edge1().Origin() == e->Next()->Origin()
            ) {
                vis = true;
            }
            hor_edges[k++] = chull[vis_faces[i]].Edge1().Twin();
        }
        if( chull[vis_faces[i]].Edge2().Twin()->Unmarked() ) {
            if( chull[vis_faces[i]].Edge3().Origin() == e->Origin() &&
                chull[vis_faces[i]].Edge2().Origin() == e->Next()->Origin()
            ) {
                vis = true;
            }
            hor_edges[k++] = chull[vis_faces[i]].Edge2().Twin();
        }
        if( chull[vis_faces[i]].Edge3().Twin()->Unmarked() ) {
            if( chull[vis_faces[i]].Edge1().Origin() == e->Origin() &&
                chull[vis_faces[i]].Edge3().Origin() == e->Next()->Origin()
            ) {
                vis = true;
            }
            hor_edges[k++] = chull[vis_faces[i]].Edge3().Twin();
        }
    }

    // Check to make sure that we do indeed have a ring of edges
    if( vis_faces.Length() != k-2 ) {
        // Undo the marking on the edges
        for( j = 0; j < vis_faces.Length(); j++ ) {
            chull[vis_faces[j]].Edge1().Unmark();
            chull[vis_faces[j]].Edge2().Unmark();
            chull[vis_faces[j]].Edge3().Unmark();
        }
        return false;
    }

    orig_faces.Create( k );
    new_faces.Create( k );

    // If none of the horizon edges correspond to e then the face cannot be
    // added since it would be visible from a previously added vertex.
    // Technically, this vertex should not be inside the convex hull since that
    // would mean that the convex hull leaked outside of the model.  This is ok
    // however because this kind of inconsistency can happen with FP computation
    if( !vis ) {
        // Undo the marking on the edges
        for( j = 0; j < vis_faces.Length(); j++ ) {
            chull[vis_faces[j]].Edge1().Unmark();
            chull[vis_faces[j]].Edge2().Unmark();
            chull[vis_faces[j]].Edge3().Unmark();
        }
        return false;
    }

    // Sort the horizon edges in clockwise order around the ring using
    // insertion sort

    // Stick the first edge in the first spot
    for( i = 0; i < k; i++ ) {
        if( hor_edges[i]->Origin() == e->Origin() ) {
            e1 = hor_edges[0];
            hor_edges[0] = hor_edges[i];
            hor_edges[i] = e1;
        }
        orig_faces[i] = nullptr;
    }

    // Sort the horizon edges
    for( i = 0; i < k-1; i++ ) {
        for( j = i+1; j < k; j++ ) {
            if( hor_edges[i]->Twin()->Origin() !=
                hor_edges[i]->Next()->Origin()
            ) {
                cerr << "*** Error: hor_edges[i]->Twin()->Origin() != "
                     << "hor_edges[i]->Next()->Origin()" << endl;
                cerr << "Exiting..." << endl;
                exit( 0 );
            }
            if( hor_edges[i]->Twin()->Origin() == hor_edges[j]->Origin() ) {
                // Swap the edge into position
                e1 = hor_edges[i+1];
                hor_edges[i+1] = hor_edges[j];
                hor_edges[j] = e1;
                break;
            }
        }
    }

    // Assign the new faces.  Also reassigns the hor_edges to be the "new"
    // inner horizon edges.
    e1 = f->EdgeP( hor_edges[0]->Origin() );
    new_faces[0].Edge1().Set_Twin( new_faces[k-1].Edge2P() );
    new_faces[k-1].Edge2().Set_Twin( new_faces[0].Edge1P() );
    new_faces[0].Edge1().Set_Length( e1->Length() );
    new_faces[0].Edge1().Set_Direction_N( e1->Direction() );

    for( i = 0; i < k; i++ ) {
        // Set the edge origins
        new_faces[i].Edge1().Set_Origin( hor_edges[i]->Origin() );
        new_faces[i].Edge2().Set_Origin( v );
        new_faces[i].Edge3().Set_Origin( hor_edges[i]->Next()->Origin() );

        // Compute or set the edge directions and lengths and compute or set
        // face planes
        if( i > 0 ) {
            new_faces[i].Edge1().Set_Twin( new_faces[i-1].Edge2P() );
            new_faces[i-1].Edge2().Set_Twin( new_faces[i].Edge1P() );
            new_faces[i].Edge1().Set_Length( new_faces[i-1].Edge2().Length() );
            new_faces[i].Edge1().Set_Direction_N(
                                        -new_faces[i-1].Edge2().Direction() );
        }
        new_faces[i].Edge3().Set_Twin( hor_edges[i] );
        new_faces[i].Edge3().Set_Length( hor_edges[i]->Length() );
        new_faces[i].Edge3().Set_Direction_N( -(hor_edges[i]->Direction()) );
        hor_edges[i] = new_faces[i].Edge3P();
        if( i < k-1 ) {
            if( orig_faces[i] == nullptr ) {
                new_faces[i].Edge2().Compute_Direction_Length();
                new_faces[i].Compute_Plane_From_Edges();
            } else {
                e1 = orig_faces[i]->EdgeP( v );
                new_faces[i].Edge2().Set_Length( e1->Length() );
                new_faces[i].Edge2().Set_Direction_N( e1->Direction() );
                new_faces[i].Set_Normal_N( orig_faces[i]->Normal() );
                new_faces[i].Set_Distance( orig_faces[i]->Distance() );
            }
        } else {
            new_faces[i].Edge2().Set_Length( new_faces[0].Edge1().Length() );
            new_faces[i].Edge2().Set_Direction_N(
                                        -new_faces[0].Edge1().Direction() );
            if( orig_faces[i] == nullptr ) {
                new_faces[i].Compute_Plane_From_Edges();
            } else {
                new_faces[i].Set_Normal_N( orig_faces[i]->Normal() );
                new_faces[i].Set_Distance( orig_faces[i]->Distance() );
            }
        }
        new_faces[i].Edge1().Compute_Face_Plane();
        new_faces[i].Edge2().Compute_Face_Plane();
        new_faces[i].Edge3().Compute_Face_Plane();
    }

    // Create the lists of faces that are kept or not kept
    old_afs_len = addedfs.Length();
    // This face is always allowed
    addedfs.Add( m->Face_Id( f ) );
    orig_faces[0] = f;
    e2 = e->Twin()->Prev()->Twin();
    for( i = 1; i < k; i++ ) {
        bool hopped = false;
        // At this point, we have the invariant that e1->Origin() ==
        // e2->Origin().  ie. we have lined up the first pair of vertices
        if( e2 == nullptr ) {
            hopped = true;

            // Search for the starting edge on the other side
            for( e2 = e->Twin()->Next(); e2->Twin() != nullptr;
                                         e2 = e2->Twin()->Prev() );
        }
        e2 = e2->Next();
        // Save the edge that we have gotten to on the original model
        // since there cannot be any matches before it from now on
        if( hopped ) {
            e3 = e2->Prev();
        } else {
            e3 = e2->Twin();
            if( e3 == nullptr ) {
                // Search for the starting edge on the other side
                for( e3 = e->Twin()->Next(); e3->Twin() != nullptr;
                                             e3 = e3->Twin()->Prev() );
            }
        }
        if( !hopped && hor_edges[i]->Origin() == e2->Next()->Origin() ) {
        //if( hor_edges[i]->Origin() == e2->Next()->Origin() ) {
            // Want to keep this face
            if( fallowed[j = m->Face_Id( e2->Adj_Face() )] ) {
                addedfs.Add( j );
                orig_faces[i] = e2->Adj_Face();
            }
            e2 = e2->Twin();
            // The invariant holds when we advance since the second pair
            // of vertices is lined up for this face, then the first pair
            // will be lined up for the next face.
        } else {
            // Search the edges of the 1-ring of the convex hull in order to
            // find an edge on the original model that matches
            for( ; i < k; i++ ) {
                // Move around the 1-ring and find out if there is a match with
                // the current vertex on the convex hull
                for( e2 = e3; e2 != e->Twin()->Next() && e2->Origin() !=
                              hor_edges[i]->Origin();
                ) {
                    if( e2->Next()->Twin() == nullptr ) {
                        for( ; e2->Twin() != nullptr; e2 = e2->Twin()->Prev() );
                    } else {
                        e2 = e2->Next()->Twin();
                    }
                }
                if( e2 != e->Twin()->Next() ) {
                    // Found a match.  From now on, we should start at this
                    // edge on the original model (e3).
                    e3 = e2;
                    break;
                }
            }
        }
    }

    bool failed;

    // Test the faces around the newly added vertex in the original model and
    // mark them for exclusion.  Simply exclude them since the opposite edge
    // should be tested for intersection and it is a part of the opposite
    // triangle which will be tested in the neighbor test or in the RAPID test.
    e1 = e->Twin()->Prev();
    do {
        j = m->Face_Id( e1->Adj_Face() );
        exclude_idx.Add( j );
        exclude[j] = true;
        if( e1->Twin() == nullptr ) {
            break;
        }
        e1 = e1->Twin()->Next();
    } while( e1 != e->Twin()->Prev() );
    if( e1->Twin() == nullptr ) {
        e1 = e->Twin()->Next()->Twin();
        while( e1 != nullptr ) {
            j = m->Face_Id( e1->Adj_Face() );
            exclude_idx.Add( j );
            exclude[j] = true;
            e1 = e1->Prev()->Twin();
        }
    }

    // Test the faces around each of the neighbors of the newly added vertex
    // for intersection and then mark them for exclusion.
    cvs[ m->Vertex_Id( v ) ] = true;
    failed = false;
    for( i = 0; i < k; i++ ) {
        l = (i == k-1) ? 0 : i+1;

        // Walk around each horizon vertex and test each of the original faces
        // attached to it to determine if they are intersecting the cone
        // created by the new vertex.
        e1 = hor_edges[i]->Origin()->Adj_Edge();
        do {
            if( !cvs[ m->Vertex_Id( e1->Next()->Origin() ) ] &&
                !cvs[ m->Vertex_Id( e1->Prev()->Origin() ) ]
            ) {
                // Only this vertex is on the convex hull.  Test the new
                // vertex to make sure that it is below this face plane
                bool underneath1 = true;
                bool underneath2 = true;

                // Check the two vertices against the additional volume
                for( j = 0; j < vis_faces.Length(); j++ ) {
                    if( chull[vis_faces[j]].Outside( e1->Next()->Origin() ) ) {
                        underneath1 = false;
                        break;
                    }
                }
                if( !underneath1 ) {
                    // Vertex was outside the old top of the hull.  See if it
                    // is inside the new top of the hull.
                    for( j = 0; j < k; j++ ) {
                        if( new_faces[j].Outside( e1->Next()->Origin() ) ) {
                            break;
                        }
                    }
                    if( j == k ) {
                        failed = true;
                        break;
                    }
                }
                for( j = 0; j < vis_faces.Length(); j++ ) {
                    if( chull[vis_faces[j]].Outside( e1->Prev()->Origin() ) ) {
                        underneath2 = false;
                        break;
                    }
                }
                if( !underneath2 ) {
                    // Vertex was outside the old top of the hull.  See if it
                    // is inside the new top of the hull.
                    for( j = 0; j < k; j++ ) {
                        if( new_faces[j].Outside( e1->Prev()->Origin() ) ) {
                            break;
                        }
                    }
                    if( j == k ) {
                        failed = true;
                        break;
                    }
                }

                // Both vertices are outside of the hull extension

                if( !underneath1 && !underneath2 ) {
                    // Both vertices are not underneath the old hull
                    const SWIFT_Real dr1 =
                                new_faces[i].Distance( e1->Next()->Origin() );
                    const SWIFT_Real dl1 =
                                new_faces[l].Distance( e1->Next()->Origin() );
                    const SWIFT_Real dr2 =
                                new_faces[i].Distance( e1->Prev()->Origin() );
                    const SWIFT_Real dl2 =
                                new_faces[l].Distance( e1->Prev()->Origin() );
                    const bool r1 = dr1 > 0.0;
                    const bool l1 = dl1 > 0.0;
                    const bool r2 = dr2 > 0.0;
                    const bool l2 = dl2 > 0.0;
                    if( (!r1 && !l1) || (!r2 && !l2) ) {
                        // Part of the edge is within both the face planes.
                        failed = true;
                        break;
                    } else if( r1 && !l1 && !r2 && l2 ) {
                        // Check to see if the edge cuts the faces
                        if( dr1 / (dr1-dr2) < dl1 / (dl1-dl2) ) {
                            failed = true;
                            break;
                        }
                    } else if( r2 && !l2 && !r1 && l1 ) {
                        // Check to see if the edge cuts the faces
                        if( dl1 / (dl1-dl2) < dr1 / (dr1-dr2) ) {
                            failed = true;
                            break;
                        }
                    }
                } else if( !underneath1 || !underneath2 ) {
                    // First test the edge opposite to the neighbor vertex

                    // Cannot intersect the first face since the model is
                    // non self-intersecting
                    for( j = 1; j < k; j++ ) {
                        // Only test against the non-model (virtual) faces
                        if( orig_faces[j] != nullptr &&
                            Intersect_Edge( new_faces[j], e1->Next()->Origin(),
                                            e1->Prev()->Origin() )
                        ) {
                            failed = true;
                            break;
                        }
                    }
                    if( failed ) {
                        break;
                    }

                    SWIFT_Tri_Vertex* adjv = underneath1 ? e1->Prev()->Origin()
                                                         : e1->Next()->Origin();
                    // Test the adjacent edge whose endpoint is above the
                    // top of the old hull.  Once again no need to test against
                    // the first face or any model faces in the new top.
                    for( j = 1; j < k; j++ ) {
                        // Only test against the non-model (virtual) faces and
                        // the new faces that are not adjacent to this neighbor
                        if( orig_faces[j] != nullptr && j != i && j != l &&
                            Intersect_Edge( new_faces[j], adjv,
                                            hor_edges[i]->Origin() )
                        ) {
                            failed = true;
                            break;
                        }
                    }
                    if( failed ) {
                        break;
                    }
                }
            } else if( !cvs[ m->Vertex_Id( e1->Prev()->Origin() ) ] ) {
                // Two of the vertices are on the new convex hull.
                // If the other is the new vertex then check the dihedral
                // angles of the two neighboring faces.  If the other is an
                // adjacent vertex (on the horizon) then check the
                // dihedral angle of the horizon edge.  Otherwise, check to see
                // if it is on the horizon.  If it is then failure.

                if( e1->Next()->Origin() == v ) {
                    // The other vertex is the new vertex
                    if( hor_edges[i]->Adj_Face()->Inside(
                                                    e1->Prev()->Origin() ) && 
                        hor_edges[l]->Adj_Face()->Inside(
                                                    e1->Prev()->Origin() )
                    ) {
                        failed = true;
                        break;
                    }
                }
                if( e1->Next()->Origin() == hor_edges[i]->Next()->Origin() ) {
                    // The other vertex is adjacent
                    if( new_faces[i].Inside( e1->Prev()->Origin() ) &&
                        hor_edges[i]->Twin()->Adj_Face()->Inside(
                            e1->Prev()->Origin() )
                    ) {
                        failed = true;
                        break;
                    }
                } else if( e1->Next()->Origin() == hor_edges[l]->Origin() ) {
                    // The other vertex is adjacent
                    if( new_faces[l].Inside( e1->Prev()->Origin() ) &&
                        hor_edges[l]->Twin()->Adj_Face()->Inside(
                            e1->Prev()->Origin() )
                    ) {
                        failed = true;
                        break;
                    }
                } else {
                    // Check to see if the other vertex is on the horizon
                    int r;
                    for( r = 0; r < k; r++ ) {
                        if( e1->Next()->Origin() == hor_edges[r]->Origin() ) {
                            failed = true;
                            break;
                        }
                    }
                    if( failed ) {
                        break;
                    }
                }
            } else if( !cvs[ m->Vertex_Id( e1->Next()->Origin() ) ] ) {
                // Two of the vertices are on the new convex hull.
                // If the other is the new vertex then check the dihedral
                // angles of the two neighboring faces.  If the other is an
                // adjacent vertex (on the horizon) then check the
                // dihedral angle of the horizon edge.  Otherwise, check to see
                // if it is on the horizon.  If it is then failure.

                if( e1->Prev()->Origin() == v ) {
                    // The other vertex is the new vertex
                    if( hor_edges[i]->Adj_Face()->Inside(
                                                    e1->Next()->Origin() ) && 
                        hor_edges[l]->Adj_Face()->Inside(
                                                    e1->Next()->Origin() )
                    ) {
                        failed = true;
                        break;
                    }
                }
                if( e1->Prev()->Origin() == hor_edges[i]->Next()->Origin() ) {
                    // The other vertex is adjacent
                    if( new_faces[i].Inside( e1->Next()->Origin() ) &&
                        hor_edges[i]->Twin()->Adj_Face()->Inside(
                            e1->Next()->Origin() )
                    ) {
                        failed = true;
                        break;
                    }
                } else if( e1->Prev()->Origin() == hor_edges[l]->Origin() ) {
                    // The other vertex is adjacent
                    if( new_faces[l].Inside( e1->Next()->Origin() ) &&
                        hor_edges[l]->Twin()->Adj_Face()->Inside(
                            e1->Next()->Origin() )
                    ) {
                        failed = true;
                        break;
                    }
                } else {
                    // Check to see if the other vertex is on the horizon
                    int r;
                    for( r = 0; r < k; r++ ) {
                        if( e1->Prev()->Origin() == hor_edges[r]->Origin() ) {
                            failed = true;
                            break;
                        }
                    }
                    if( failed ) {
                        break;
                    }
                }
            }

            // This face succeeded.  Add it to the neighbors
            j = m->Face_Id( e1->Adj_Face() );
            exclude_idx.Add( j );
            exclude[j] = true;
            if( e1->Twin() == nullptr ) {
                // Find the other boundary edge
                while( e1->Prev()->Twin() != nullptr ) {
                    e1 = e1->Prev()->Twin();
                }
            } else {
                e1 = e1->Twin()->Next();
            }
        } while( e1 != hor_edges[i]->Origin()->Adj_Edge() );

        if( failed ) {
            // This vertex is not added
            cvs[ m->Vertex_Id( v ) ] = false;
            // There are no added faces
            addedfs.Set_Length( old_afs_len );
            // Unmark the edges of the visible faces
            for( j = 0; j < vis_faces.Length(); j++ ) {
                chull[vis_faces[j]].Edge1().Unmark();
                chull[vis_faces[j]].Edge2().Unmark();
                chull[vis_faces[j]].Edge3().Unmark();
            }
            // Unexclude the up-to-now excluded faces
            for( j = 0; j < exclude_idx.Length(); j++ ) {
                exclude[ exclude_idx[j] ] = false;
            }
            // Set no excluded faces
            exclude_idx.Set_Length( 0 );

            // return failure
            return false;
        }
    }

    // Perform an OBB-tree overlap test between the original model and the
    // newly added faces.  Build the tree for just the newly added faces that
    // are virtual.  Query for the first contact but disallow the faces already
    // in this piece from being counted/reported.
    rm2->BeginModel();
    for( i = 1; i < k; i++ ) {
        if( orig_faces[i] == nullptr ) {
            // Add this face to the rapid model
            rm2->AddTri( hor_edges[i]->Origin()->Coords().Value(),
                         hor_edges[i]->Next()->Origin()->Coords().Value(),
                         v->Coords().Value(), i );
        }
    }
    rm2->EndModel();

    RAPID_Collide( rm1, rm2, exclude.Data(), RAPID_FIRST_CONTACT );

    if( RAPID_num_contacts > 0 ) {
        // Unset the exclusion flags
        // This vertex is not added
        cvs[ m->Vertex_Id( v ) ] = false;
        // There are no added faces
        addedfs.Set_Length( old_afs_len );
        // Unmark the edges of the visible faces
        for( j = 0; j < vis_faces.Length(); j++ ) {
            chull[vis_faces[j]].Edge1().Unmark();
            chull[vis_faces[j]].Edge2().Unmark();
            chull[vis_faces[j]].Edge3().Unmark();
        }
        // Unexclude the up-to-now excluded faces
        for( j = 0; j < exclude_idx.Length(); j++ ) {
            exclude[ exclude_idx[j] ] = false;
        }
        // Set no excluded faces
        exclude_idx.Set_Length( 0 );

        // return failure
        return false;
    }

    // At this point, the vertex has been ok'ed and the addition proceeds.

    // Remove the j visible faces
    for( i = 0; i < vis_faces.Length(); i++ ) {
        chull[vis_faces[i]].Mark();
    }


    // Now we add the k faces

    // Adding this vertex will add exactly 2 faces since there can be no
    // vertices in the convex hull.
    chull.Increment_Length();
    chull.Increment_Length();
    cfs.Increment_Length();
    cfs.Increment_Length();
    chull[ chull.Length()-2 ].Mark();
    chull[ chull.Length()-1 ].Mark();

    e1 = nullptr;
    for( i = 0; i < chull.Length() && k > 0; i++ ) {
        // We can reuse the spots in the array that have been vacated (marked)
        if( chull[i].Marked() ) {
            // Found a free face record
            k--;


            cfs[i] = orig_faces[k];
            // Copy over some of the face's contents
            chull[i].Edge1().Set_Origin( new_faces[k].Edge1().Origin() );
            chull[i].Edge2().Set_Origin( new_faces[k].Edge2().Origin() );
            chull[i].Edge3().Set_Origin( new_faces[k].Edge3().Origin() );
            chull[i].Edge1().Set_Length( new_faces[k].Edge1().Length() );
            chull[i].Edge2().Set_Length( new_faces[k].Edge2().Length() );
            chull[i].Edge3().Set_Length( new_faces[k].Edge3().Length() );
            chull[i].Edge1().Set_Direction_N(
                                            new_faces[k].Edge1().Direction() );
            chull[i].Edge2().Set_Direction_N(
                                            new_faces[k].Edge2().Direction() );
            chull[i].Edge3().Set_Direction_N(
                                            new_faces[k].Edge3().Direction() );
            chull[i].Set_Normal_N( new_faces[k].Normal() );
            chull[i].Set_Distance( new_faces[k].Distance() );

            // Fix the face markings
            chull[i].Edge1().Unmark();
            chull[i].Edge2().Unmark();
            chull[i].Edge3().Unmark();
            chull[i].Unmark();

            // Set the twins
            hor_edges[k]->Twin()->Set_Twin( chull[i].Edge3P() );
            chull[i].Edge3().Set_Twin( hor_edges[k]->Twin() );

            if( e1 == nullptr ) {
                // This is the first face.  Save the last twin in e2.
                e2 = chull[i].Edge2P();
            } else {
                // This is not the first face.  e1 is the previous twin.
                e1->Set_Twin( chull[i].Edge2P() );
                chull[i].Edge2().Set_Twin( e1 );
            }

            e1 = chull[i].Edge1P();

            if( k == 0 ) {
                // Fix up the last face
                e2->Set_Twin( chull[i].Edge1P() );
                chull[i].Edge1().Set_Twin( e2 );
            }
        }
    }

    // Unexclude the up-to-now excluded faces
    for( j = 0; j < exclude_idx.Length(); j++ ) {
        exclude[ exclude_idx[j] ] = false;
    }
    // Set no excluded faces
    exclude_idx.Set_Length( 0 );

    return true;
}

void Attach_Twins( SWIFT_Array<SWIFT_Tri_Face>& new_faces )
{
    int l;
    if( new_faces.Last().Edge1().Twin() == nullptr ) {
        for( l = 0; l < new_faces.Length()-1; l++ ) { 
            if( new_faces[l].Edge1().Origin() ==
                new_faces.Last().Edge2().Origin() &&
                new_faces[l].Edge2().Origin() ==
                new_faces.Last().Edge1().Origin()
            ) {
                // Matched twin for edge1
                new_faces.Last().Edge1().Set_Twin( new_faces[l].Edge1P() );
                break;
            }
            if( new_faces[l].Edge2().Origin() ==
                new_faces.Last().Edge2().Origin() &&
                new_faces[l].Edge3().Origin() ==
                new_faces.Last().Edge1().Origin()
            ) {
                // Matched twin for edge1
                new_faces.Last().Edge1().Set_Twin( new_faces[l].Edge2P() ); 
                break;
            }
            if( new_faces[l].Edge3().Origin() ==
                new_faces.Last().Edge2().Origin() &&
                new_faces[l].Edge1().Origin() ==
                new_faces.Last().Edge1().Origin()
            ) {
                // Matched twin for edge1
                new_faces.Last().Edge1().Set_Twin( new_faces[l].Edge3P() );
                break;
            }
        }
        if( l != new_faces.Length()-1 ) {
            new_faces.Last().Edge1().Twin()->Set_Twin(
                                                new_faces.Last().Edge1P() );
            new_faces.Last().Edge1().Set_Length( 
                                new_faces.Last().Edge1().Twin()->Length() );
            new_faces.Last().Edge1().Set_Direction_N( 
                                new_faces.Last().Edge1().Twin()->Direction() );
        } else {
            // Compute the direction and length of this edge
            new_faces.Last().Edge1().Compute_Direction_Length();
        }
    }
    if( new_faces.Last().Edge2().Twin() == nullptr ) {
        for( l = 0; l < new_faces.Length()-1; l++ ) {
            if( new_faces[l].Edge1().Origin() ==
                new_faces.Last().Edge3().Origin() &&
                new_faces[l].Edge2().Origin() ==
                new_faces.Last().Edge2().Origin()
            ) {
                // Matched twin for edge2
                new_faces.Last().Edge2().Set_Twin( new_faces[l].Edge1P() );
                break;
            }
            if( new_faces[l].Edge2().Origin() ==
                new_faces.Last().Edge3().Origin() &&
                new_faces[l].Edge3().Origin() ==
                new_faces.Last().Edge2().Origin()
            ) {
                // Matched twin for edge2
                new_faces.Last().Edge2().Set_Twin( new_faces[l].Edge2P() );
                break;
            }
            if( new_faces[l].Edge3().Origin() ==
                new_faces.Last().Edge3().Origin() &&
                new_faces[l].Edge1().Origin() ==
                new_faces.Last().Edge2().Origin()
            ) {
                // Matched twin for edge2
                new_faces.Last().Edge2().Set_Twin( new_faces[l].Edge3P() );
                break;
            }
        }
        if( l != new_faces.Length()-1 ) {
            new_faces.Last().Edge2().Twin()->Set_Twin(
                                                new_faces.Last().Edge2P() );
            new_faces.Last().Edge2().Set_Length( 
                                new_faces.Last().Edge2().Twin()->Length() );
            new_faces.Last().Edge2().Set_Direction_N( 
                                new_faces.Last().Edge2().Twin()->Direction() );
        } else {
            // Compute the direction and length of this edge
            new_faces.Last().Edge2().Compute_Direction_Length();
        }
    }
    if( new_faces.Last().Edge3().Twin() == nullptr ) {
        for( l = 0; l < new_faces.Length()-1; l++ ) {
            if( new_faces[l].Edge1().Origin() ==
                new_faces.Last().Edge1().Origin() &&
                new_faces[l].Edge2().Origin() ==
                new_faces.Last().Edge3().Origin()
            ) {
                // Matched twin for edge3
                new_faces.Last().Edge3().Set_Twin( new_faces[l].Edge1P() );
                break;
            }
            if( new_faces[l].Edge2().Origin() ==
                new_faces.Last().Edge1().Origin() &&
                new_faces[l].Edge3().Origin() ==
                new_faces.Last().Edge3().Origin()
            ) {
                // Matched twin for edge3
                new_faces.Last().Edge3().Set_Twin( new_faces[l].Edge2P() );
                break;
            }
            if( new_faces[l].Edge3().Origin() ==
                new_faces.Last().Edge1().Origin() &&
                new_faces[l].Edge1().Origin() ==
                new_faces.Last().Edge3().Origin()
            ) {
                // Matched twin for edge3
                new_faces.Last().Edge3().Set_Twin( new_faces[l].Edge3P() );
                break;
            }
        }
        if( l != new_faces.Length()-1 ) {
            new_faces.Last().Edge3().Twin()->Set_Twin(
                                                new_faces.Last().Edge3P() );
            new_faces.Last().Edge3().Set_Length( 
                                new_faces.Last().Edge3().Twin()->Length() );
            new_faces.Last().Edge3().Set_Direction_N( 
                                new_faces.Last().Edge3().Twin()->Direction() );
        } else {
            // Compute the direction and length of this edge
            new_faces.Last().Edge3().Compute_Direction_Length();
        }
    }
}

void Quicksort( SWIFT_Array< SWIFT_Array<int> >& mfs,
                SWIFT_Array<int>& mfs_mapping, int p, int r )
{
    if( p < r ) {
        // Compute a random element to use as the pivot
        int rn = (int) ((SWIFT_Real)(r-p+1) * drand48()) + p;
        int i = p-1;
        int j = r+1;
        int x = mfs_mapping[rn];
        int tmap;

        // Swap the random element into the first position
        tmap = mfs_mapping[rn];
        mfs_mapping[rn] = mfs_mapping[p];
        mfs_mapping[p] = tmap;

        while( true ) {
            for( j--; mfs[mfs_mapping[j]].Length() > mfs[x].Length(); j-- );
            for( i++; mfs[mfs_mapping[i]].Length() < mfs[x].Length(); i++ );
            if( i < j ) {
                tmap = mfs_mapping[i];
                mfs_mapping[i] = mfs_mapping[j];
                mfs_mapping[j] = tmap;
            } else {
                break;
            }
        }

        Quicksort( mfs, mfs_mapping, p, j );
        Quicksort( mfs, mfs_mapping, j+1, r );
    }
}



// Priority Queue for face counts (used by cresting BFS)

void Build_Heap( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
                 SWIFT_Array<int>& fmap )
{
    int i;

    for( i = 0; i < bmap.Length(); i++ ) {
        bmap[i] = i; fmap[i] = i;
    }

    for( i = ((lengths.Length())>>1)-1; i >= 0; i-- ) {
        Heapify( lengths, bmap, fmap, i );
    }
}   

void Heapify( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
              SWIFT_Array<int>& fmap, int i )
{
    int l = (i<<1)+1;
    int r = l+1;
    int largest = (l < lengths.Length() && lengths[l] > lengths[i]) ? l : i;

    if( r < lengths.Length() && lengths[r] > lengths[largest] ) {
        largest = r;
    }

    while( largest != i ) { 
        int tempi = lengths[i];
        lengths[i] = lengths[largest];
        lengths[largest] = tempi;
        tempi = bmap[i];
        bmap[i] = bmap[largest];
        bmap[largest] = tempi;
        fmap[bmap[i]] = i;
        fmap[bmap[largest]] = largest;
        
        i = largest;
        l = (i<<1)+1;
        r = l+1;
        largest = (l < lengths.Length() && lengths[l] > lengths[i]) ? l : i;

        if( r < lengths.Length() && lengths[r] > lengths[largest] ) {
            largest = r;
        }
    }
}

void Up_Heap( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
              SWIFT_Array<int>& fmap, int i )
{
    int parent = (i-1)>>1;

    while( i > 0 && lengths[parent] < lengths[i] ) {
        int tempi = lengths[i];
        lengths[i] = lengths[parent];
        lengths[parent] = tempi;
        tempi = bmap[i];
        bmap[i] = bmap[parent];
        bmap[parent] = tempi;
        fmap[bmap[i]] = i;
        fmap[bmap[parent]] = parent;

        i = parent;
        parent = (i-1)>>1;
    }
}

void Heap_New_Value( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
                     SWIFT_Array<int>& fmap, int i, int new_val )
{
    bool heap_down = new_val < lengths[i];
    if( heap_down ) {
        Heapify( lengths, bmap, fmap, i );
    } else {
        Up_Heap( lengths, bmap, fmap, i );
    }
}

void Heap_Dec_Value( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
                     SWIFT_Array<int>& fmap, int i )
{
    lengths[i]--;
    Heapify( lengths, bmap, fmap, i );
}

void Delete_Heap_Top( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
                      SWIFT_Array<int>& fmap )
{
    Delete_From_Heap( lengths, bmap, fmap, 0 );
}

void Delete_From_Heap( SWIFT_Array<int>& lengths, SWIFT_Array<int>& bmap,
                       SWIFT_Array<int>& fmap, int i )
{
    bool heap_down = lengths.Last() < lengths[i];

    // Move the last element into the deleted position and re-heap (up or down)
    lengths[i] = lengths.Last();
    lengths.Decrement_Length();
    bmap[i] = bmap.Last();
    fmap[bmap[i]] = i;
    bmap.Decrement_Length();

    if( i == bmap.Length() ) {
        return;
    }

    if( heap_down ) {
        Heapify( lengths, bmap, fmap, i );
    } else {
        Up_Heap( lengths, bmap, fmap, i );
    }
}

// Is flipping from e1 to e2 allowed?
bool Edge_Flip_Allowed( SWIFT_Tri_Edge* e1, SWIFT_Tri_Edge* e2,
                        SWIFT_Real etol_sq )
{
    SWIFT_Triple tri2 = e2->Origin()->Coords() - e1->Origin()->Coords();
    SWIFT_Real d1_dot_v1 = e1->Direction() * tri2;
    SWIFT_Real d2_dot_v1 = e2->Direction() * tri2;
    SWIFT_Real d = e2->Direction() * e1->Direction();
    // t is distance along e1 and u is distance along e2
    SWIFT_Real t, u = 1.0 - d * d;  // denom

    if( u == 0.0 ) {
        return false;
    } else {
        t = (d1_dot_v1 - d2_dot_v1 * d) / u;
        if( t < EPSILON3 * e1->Length() || t > e1->Length() * (1-EPSILON3) ) {
            return false;
        } else {
            u = t*d - d2_dot_v1;
        }
    }

    if( u < 0.0 || u > e2->Length() ) {
        return false;
    } else {
        tri2 = e1->Origin()->Coords() + t * e1->Direction();
        return tri2.Dist_Sq( e2->Origin()->Coords() + u * e2->Direction() ) <
               etol_sq;
    }
}   


