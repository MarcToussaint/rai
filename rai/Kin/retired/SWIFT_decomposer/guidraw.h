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
// guidraw.h
//
// Description:
//      Draw utility functions for the user interface.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _GUIDRAW_H_
#define _GUIDRAW_H_

#include "../SWIFT/SWIFT_array.h"
#include "../SWIFT/SWIFT_mesh.h"

void Draw_Triangle_Mesh( SWIFT_Tri_Mesh* m, bool color
                         , SWIFT_Array<int>& piece_ids );
void Draw_Vertices( SWIFT_Tri_Mesh* m );
void Draw_Edge_Convexity( SWIFT_Tri_Mesh* m, SWIFT_Array<bool>& ecs );
void Draw_Convex_Pieces( SWIFT_Tri_Mesh* m, bool color, bool vfaces, bool tcol,
                         bool explode, SWIFT_Triple& expl_cen,
                         SWIFT_Array<SWIFT_Triple>& coms,
                         SWIFT_Array<int>& piece_ids,
                         SWIFT_Array< SWIFT_Array<int> >& mfs,
                         SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> >& vfs,
                         SWIFT_Array<int>& wcps );
void Draw_Hierarchy( SWIFT_Tri_Mesh* m, bool color, bool vfaces, bool tcol,
                     bool explode, SWIFT_Triple& expl_cen,
                     SWIFT_Array<int>& piece_ids,
                     SWIFT_Array<SWIFT_BV*>& which_pieces );

#endif

