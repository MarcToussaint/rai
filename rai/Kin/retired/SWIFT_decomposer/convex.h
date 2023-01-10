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
// convex.h
//
// Description:
//      The convex decomposition routines
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _CONVEX_H_
#define _CONVEX_H_

#include "../SWIFT/SWIFT_array.h"
#include "../SWIFT/SWIFT_mesh.h"

void Convex_Initialize( SWIFT_Tri_Mesh* m );

void Create_One_Piece( SWIFT_Tri_Mesh* m,
                       SWIFT_Array<int>& piece_ids,
                       SWIFT_Array< SWIFT_Array<int> >& mfs,
                       SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs );

int Decompose_DFS( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
               SWIFT_Array< SWIFT_Array<int> >& mfs,
               SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs, bool random );

int Decompose_BFS( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
               SWIFT_Array< SWIFT_Array<int> >& mfs,
               SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs, bool random );

int Decompose_Cresting_BFS( SWIFT_Tri_Mesh* m, SWIFT_Array<int>& piece_ids,
               SWIFT_Array< SWIFT_Array<int> >& mfs,
               SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs );



void Jitter( SWIFT_Tri_Mesh* m, SWIFT_Real jampl );
void Edge_Flip( SWIFT_Tri_Mesh* m, SWIFT_Real etol );

#endif

