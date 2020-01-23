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
// viewer.h
//
// Description:
//      Exported viewing functions.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _VIEWER_H_
#define _VIEWER_H_

#include "../SWIFT/SWIFT_common.h"
#include "../SWIFT/SWIFT_linalg.h"

//////////////////////////////////////////////////////////////////////////////
// Camera
//
// Description:
//      Camera class.
//////////////////////////////////////////////////////////////////////////////
class Camera {
  public:
    Camera( ) { }
    ~Camera( ) { }

    SWIFT_Triple Position( ) { return orig; }
    SWIFT_Triple X( ) { return x; }
    SWIFT_Triple Y( ) { return y; }
    SWIFT_Triple Z( ) { return z; }

    SWIFT_Triple XW( ) { return xw; }
    SWIFT_Triple YW( ) { return yw; }
    SWIFT_Triple ZW( ) { return zw; }

    void Pitch( SWIFT_Real angle );
    void Roll( SWIFT_Real angle );
    void Yaw( SWIFT_Real angle );
    void PitchW( SWIFT_Real angle );
    void RollW( SWIFT_Real angle );
    void YawW( SWIFT_Real angle );
    void Move( const SWIFT_Triple& t );
    void Move( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z );
    void Move_To( SWIFT_Real tx, SWIFT_Real ty, SWIFT_Real tz );
    void Reset_Orientation( );
    void Reset_OrientationW( );
    void Reset_Angles( );

  private:
    SWIFT_Triple x, y, z, orig;
    SWIFT_Real pangle, yangle, rangle;
    SWIFT_Triple xw, yw, zw;
};

//////////////////////////////////////////////////////////////////////////////
// Light
//
// Description:
//      Light class.
//////////////////////////////////////////////////////////////////////////////
class Light {
  public:
    Light( ) { created = 0; }
    ~Light( ) { }

    SWIFT_Triple Position( ) { return pos; }
    SWIFT_Real Directional( ) { return dir; }
    int Attached_To_Camera( ) { return attach_cam; }
    int Exists( ) { return created; }

    void Create( SWIFT_Real x, SWIFT_Real y, SWIFT_Real z,
                 int directional, int attach_to_cam )
    {
        pos.Set_Value( x, y, z );
        if( directional ) {
            dir = 0.0;
        } else {
            dir = 1.0;
        }
        attach_cam = attach_to_cam;
        created = 1;
    }
    void Destroy( ) { created = 0; }

  private:
    SWIFT_Triple pos;
    SWIFT_Real dir;
    int attach_cam;
    int created;
};

void VIEWER_Initialize( );
void VIEWER_Shutdown( );
void Set_Model_Dim( SWIFT_Real xmin, SWIFT_Real xmax,
                    SWIFT_Real ymin, SWIFT_Real ymax,
                    SWIFT_Real zmin, SWIFT_Real zmax );
void Set_OpenGL_Camera( );

// Rotate the world
void Start_Plain_Mouse1( SWIFT_Real x, SWIFT_Real y );
// Look xy
void Start_Shift_Mouse1( SWIFT_Real x, SWIFT_Real y );
// Nothing
void Start_Control_Mouse1( SWIFT_Real x, SWIFT_Real y );
// Drive mode
void Start_Alt_Mouse1( SWIFT_Real x, SWIFT_Real y );
// Translate xz
void Start_Plain_Mouse2( SWIFT_Real x, SWIFT_Real y );
// Drive mode
void Start_Shift_Mouse2( SWIFT_Real x, SWIFT_Real y );
// Nothing
void Start_Control_Mouse2( SWIFT_Real x, SWIFT_Real y );
// Nothing
void Start_Alt_Mouse2( SWIFT_Real x, SWIFT_Real y );
// Translate xy
void Start_Plain_Mouse3( SWIFT_Real x, SWIFT_Real y );
// Trackball model center
void Start_Shift_Mouse3( SWIFT_Real x, SWIFT_Real y );
// Nothing
void Start_Control_Mouse3( SWIFT_Real x, SWIFT_Real y );
// Nothing
void Start_Alt_Mouse3( SWIFT_Real x, SWIFT_Real y );

void Adjust( SWIFT_Real x, SWIFT_Real y );

void Reset_Cameras( );
void Jump_To_Camera( int which );
void Save_Camera( int which );
void Reset_Current_Camera( );

void Create_Light( int num, SWIFT_Real x, SWIFT_Real y, SWIFT_Real z,
                   int dir, int attach_cam );
void Destroy_Light( int num );

SWIFT_Triple View_X( );
SWIFT_Triple View_Y( );
SWIFT_Triple View_Z( );
SWIFT_Real View_Distance( );
void Draw_World_Axes( );
void Draw_Model_Axes( );

#endif
