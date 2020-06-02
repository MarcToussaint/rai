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
// viewer.cpp
//
// Description:
//      See viewer.h
//
//////////////////////////////////////////////////////////////////////////////

#ifdef DECOMP_GRAPHICS
#include <iostream>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>

#include "../SWIFT/SWIFT_common.h"
#include "../SWIFT/SWIFT_linalg.h"
#include <viewer.h>

// Types
typedef enum { MOUSE_NONE, MOUSE_PLAIN, MOUSE_SHIFT, MOUSE_CONTROL, MOUSE_ALT
                } MOUSE_MODE_T;

// Constants
static const SWIFT_Real DEGREES_TO_RADIANS = PI / 180.0;
static const SWIFT_Real RADIANS_TO_DEGREES = 180.0 / PI;

static const SWIFT_Real fov = 45.0;

static const int MAX_CAMERAS = 10;
static const int MAX_LIGHTS = 8;

MOUSE_MODE_T mouse_mode;
int mouse_button;
SWIFT_Real prev_x, prev_y;

SWIFT_Triple model_center;
SWIFT_Real model_radius;
SWIFT_Real nearp, farp;
SWIFT_Real wsize;

Camera cams[MAX_CAMERAS];
Camera cur_cam;

Light lights[MAX_LIGHTS];
GLenum glLights[MAX_LIGHTS] = { GL_LIGHT0, GL_LIGHT1, GL_LIGHT2, GL_LIGHT3,
                                GL_LIGHT4, GL_LIGHT5, GL_LIGHT6, GL_LIGHT7 };

///////////////////////////////////////////////////////////////////////////////
// Camera Functions
///////////////////////////////////////////////////////////////////////////////
void Camera::Pitch( SWIFT_Real angle )
{
//cerr << "Pitch : " << angle << endl;
    SWIFT_Real cR = cos( angle );
    SWIFT_Real sR = sin( angle );
    SWIFT_Triple newy = SWIFT_Triple( cR*y.X() - sR*z.X(), cR*y.Y() - sR*z.Y(),
                                               cR*y.Z() - sR*z.Z() );
    SWIFT_Triple newz = SWIFT_Triple( sR*y.X() + cR*z.X(), sR*y.Y() + cR*z.Y(),
                                               sR*y.Z() + cR*z.Z() );
    y = newy;
    z = newz;
}

void Camera::Yaw( SWIFT_Real angle )
{
//cerr << "Yaw : " << angle << endl;
    SWIFT_Real cR = cos( angle );
    SWIFT_Real sR = sin( angle );
    SWIFT_Triple newx = SWIFT_Triple( cR*x.X() - sR*z.X(), cR*x.Y() - sR*z.Y(),
                                               cR*x.Z() - sR*z.Z() );
    SWIFT_Triple newz = SWIFT_Triple( sR*x.X() + cR*z.X(), sR*x.Y() + cR*z.Y(),
                                               sR*x.Z() + cR*z.Z() );
    x = newx;
    z = newz;
}

void Camera::Roll( SWIFT_Real angle )
{
//cerr << "Roll : " << angle << endl;
    SWIFT_Real cR = cos( angle );
    SWIFT_Real sR = sin( angle );
    SWIFT_Triple newx = SWIFT_Triple( cR * x.X() + sR * y.X(),
                                      cR * x.Y() + sR * y.Y(),
                                      cR * x.Z() + sR * y.Z() );
    SWIFT_Triple newy = SWIFT_Triple( cR * y.X() - sR * x.X(),
                                      cR * y.Y() - sR * x.Y(),
                                      cR * y.Z() - sR * x.Z() );
    x = newx;
    y = newy;
}

void Camera::PitchW( SWIFT_Real angle )
{
//cerr << "PitchW : " << angle << endl;
    pangle += angle;
    SWIFT_Real cR = cos( pangle );
    SWIFT_Real sR = sin( pangle );
    SWIFT_Triple newx = SWIFT_Triple( cR*xw.X() - sR*zw.X(),
                                      cR*xw.Y() - sR*zw.Y(),
                                      cR*xw.Z() - sR*zw.Z() );
    SWIFT_Triple newz = SWIFT_Triple( sR*xw.X() + cR*zw.X(),
                                      sR*xw.Y() + cR*zw.Y(),
                                      sR*xw.Z() + cR*zw.Z() );
    xw = newx;
    zw = newz;
}

void Camera::YawW( SWIFT_Real angle )
{
//cerr << "YawW : " << angle << endl;
    yangle += angle;
    SWIFT_Real cR = cos( yangle );
    SWIFT_Real sR = sin( yangle );
    SWIFT_Triple newx = SWIFT_Triple( cR*xw.X() + sR*yw.X(),
                                      cR*xw.Y() + sR*yw.Y(),
                                      cR*xw.Z() + sR*yw.Z() );
    SWIFT_Triple newy = SWIFT_Triple( cR*yw.X() - sR*xw.X(),
                                      cR*yw.Y() - sR*xw.Y(),
                                      cR*yw.Z() - sR*xw.Z() );
    xw = newx;
    yw = newy;
}

void Camera::RollW( SWIFT_Real angle )
{
//cerr << "RollW : " << angle << endl;
    rangle += angle;
    SWIFT_Real cR = cos( rangle );
    SWIFT_Real sR = sin( rangle );
    SWIFT_Triple newy = SWIFT_Triple( cR*yw.X() - sR*zw.X(),
                                      cR*yw.Y() - sR*zw.Y(),
                                      cR*yw.Z() - sR*zw.Z() );
    SWIFT_Triple newz = SWIFT_Triple( sR*yw.X() + cR*zw.X(),
                                      sR*yw.Y() + cR*zw.Y(),
                                      sR*yw.Z() + cR*zw.Z() );
    yw = newy;
    zw = newz;
}

void Camera::Move( const SWIFT_Triple& t )
{
//cerr << "Move " << t.X() << " " << t.Y() << " " << t.Z() << endl;
    orig += x * t.X() + y * t.Y() + z * t.Z();
}

void Camera::Move( SWIFT_Real tx, SWIFT_Real ty, SWIFT_Real tz )
{
//cerr << "Move " << tx << " " << ty << " " << tz << endl;
    orig += x * tx + y * ty + z * tz;
}

void Camera::Move_To( SWIFT_Real tx, SWIFT_Real ty, SWIFT_Real tz )
{
    orig.Set_Value( tx, ty, tz );
}

void Camera::Reset_Orientation( )
{
    x.Set_Value( 0.0, 1.0, 0.0 );
    y.Set_Value( 0.0, 0.0, 1.0 );
    z.Set_Value( 1.0, 0.0, 0.0 );
}

void Camera::Reset_OrientationW( )
{
    xw.Set_Value( 1.0, 0.0, 0.0 );
    yw.Set_Value( 0.0, 1.0, 0.0 );
    zw.Set_Value( 0.0, 0.0, 1.0 );
}

void Camera::Reset_Angles( )
{
    pangle = rangle = yangle = 0.0;
}

///////////////////////////////////////////////////////////////////////////////
// Internal Functions
///////////////////////////////////////////////////////////////////////////////
void Recompute_Near_Far_And_Wsize( )
{
    nearp = cur_cam.Z() * ( cur_cam.Position() - model_center ) - model_radius;
    if( nearp < 0.01 ) {
        nearp = 0.01;
    }
    farp = nearp + 2.0 * model_radius;
    wsize = tan( fov * DEGREES_TO_RADIANS * 0.5 ) * nearp;
}

///////////////////////////////////////////////////////////////////////////////
// External Functions
///////////////////////////////////////////////////////////////////////////////
void VIEWER_Initialize( )
{
    mouse_mode = MOUSE_NONE;
    model_center.Set_Value( 0.0, 0.0, 0.0 );
    model_radius = 1.0;
    Reset_Cameras();
    Recompute_Near_Far_And_Wsize();
}

void VIEWER_Shutdown( )
{
}

void Set_Model_Dim( SWIFT_Real xmin, SWIFT_Real xmax,
                    SWIFT_Real ymin, SWIFT_Real ymax,
                    SWIFT_Real zmin, SWIFT_Real zmax )
{
    model_center = 0.5 * SWIFT_Triple( (xmin+xmax), (ymin+ymax), (zmin+zmax) );
    model_radius = sqrt( (xmax-xmin) * (xmax-xmin) +
                         (ymax-ymin) * (ymax-ymin) +
                         (zmax-zmin) * (zmax-zmin) );

    //Reset_Cameras();
    Recompute_Near_Far_And_Wsize();
}

void Set_OpenGL_Camera( )
{
    int i;
    GLfloat light_position[4];
    SWIFT_Real M[16];

//cerr << "Setting Set_OpenGL_Camera near, far = " << nearp << " " << farp << endl;
    // Set perspective and near and far planes
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( fov, 1.0, nearp, farp );

    // Set the modelview matrix
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    // Set positions of lights that follow the camera
    for( i = 0; i < MAX_LIGHTS; i++ ) {
        if( lights[i].Exists() && lights[i].Attached_To_Camera() ) {
            light_position[0] = lights[i].Position().X();
            light_position[1] = lights[i].Position().Y();
            light_position[2] = lights[i].Position().Z();
            light_position[3] = lights[i].Directional();
            glLightfv( glLights[i], GL_POSITION, light_position );
        }
    }

    // Create the camera transformation
    M[0] = cur_cam.X().X();
    M[4] = cur_cam.X().Y();
    M[8] = cur_cam.X().Z();
    M[12] = -(cur_cam.Position() * cur_cam.X());
    M[1] = cur_cam.Y().X();
    M[5] = cur_cam.Y().Y();
    M[9] = cur_cam.Y().Z();
    M[13] = -(cur_cam.Position() * cur_cam.Y());
    M[2] = cur_cam.Z().X();
    M[6] = cur_cam.Z().Y();
    M[10] = cur_cam.Z().Z();
    M[14] = -(cur_cam.Position() * cur_cam.Z());
    M[3] = 0.0; M[7] = 0.0; M[11] = 0.0; M[15] = 1.0;
#ifdef SWIFT_USE_FLOAT
    glLoadMatrixf( M );
#else
    glLoadMatrixd( M );
#endif

    // Create the world transformation
    M[0] = cur_cam.XW().X();
    M[4] = cur_cam.XW().Y();
    M[8] = cur_cam.XW().Z();
    M[12] = 0.0;
    M[1] = cur_cam.YW().X();
    M[5] = cur_cam.YW().Y();
    M[9] = cur_cam.YW().Z();
    M[13] = 0.0;
    M[2] = cur_cam.ZW().X();
    M[6] = cur_cam.ZW().Y();
    M[10] = cur_cam.ZW().Z();
    M[14] = 0.0;
    M[3] = 0.0; M[7] = 0.0; M[11] = 0.0; M[15] = 1.0;
#ifdef SWIFT_USE_FLOAT
    glMultMatrixf( M );
#else
    glMultMatrixd( M );
#endif

    // Set positions of lights relative to the world
    for( i = 0; i < MAX_LIGHTS; i++ ) {
        if( lights[i].Exists() && !lights[i].Attached_To_Camera() ) {
            light_position[0] = lights[i].Position().X();
            light_position[1] = lights[i].Position().Y();
            light_position[2] = lights[i].Position().Z();
            light_position[3] = lights[i].Directional();
            glLightfv( glLights[i], GL_POSITION, light_position );
        }
    }
}

void Start_Plain_Mouse1( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 1;
    mouse_mode = MOUSE_PLAIN;
    prev_x = x;
    prev_y = y;
}

void Start_Shift_Mouse1( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 1;
    mouse_mode = MOUSE_SHIFT;
    prev_x = x;
    prev_y = y;
}

void Start_Control_Mouse1( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 1;
    mouse_mode = MOUSE_CONTROL;
    prev_x = x;
    prev_y = y;
}

void Start_Alt_Mouse1( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 1;
    mouse_mode = MOUSE_ALT;
    prev_x = x;
    prev_y = y;
}

void Start_Plain_Mouse2( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 2;
    mouse_mode = MOUSE_PLAIN;
    prev_x = x;
    prev_y = y;
}

void Start_Shift_Mouse2( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 2;
    mouse_mode = MOUSE_SHIFT;
    prev_x = x;
    prev_y = y;
}

void Start_Control_Mouse2( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 2;
    mouse_mode = MOUSE_CONTROL;
    prev_x = x;
    prev_y = y;
}

void Start_Alt_Mouse2( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 2;
    mouse_mode = MOUSE_ALT;
    prev_x = x;
    prev_y = y;
}

void Start_Plain_Mouse3( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 3;
    mouse_mode = MOUSE_PLAIN;
    prev_x = x;
    prev_y = y;
}

void Start_Shift_Mouse3( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 3;
    mouse_mode = MOUSE_SHIFT;
    prev_x = x;
    prev_y = y;
}

void Start_Control_Mouse3( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 3;
    mouse_mode = MOUSE_CONTROL;
    prev_x = x;
    prev_y = y;
}

void Start_Alt_Mouse3( SWIFT_Real x, SWIFT_Real y )
{
    mouse_button = 3;
    mouse_mode = MOUSE_ALT;
    prev_x = x;
    prev_y = y;
}

void Adjust( SWIFT_Real x, SWIFT_Real y )
{
    SWIFT_Real dx = x-prev_x;
    SWIFT_Real dy = y-prev_y;
    SWIFT_Real dist = sqrt( cur_cam.Position().Dist_Sq( model_center ) );

    switch( mouse_mode ) {
    case MOUSE_NONE:
        return;
    case MOUSE_PLAIN:
        switch( mouse_button ) {
        case 1:
            // Rotate the world
            cur_cam.Reset_OrientationW();
            if( cur_cam.Y().Z() < 0.0 ) {
                cur_cam.YawW( dx * 1.5 );
            } else {
                cur_cam.YawW( -dx * 1.5 );
            }
            cur_cam.PitchW( dy * 1.5 );
            break;
        case 2:
            // Translate xz
            if( dist < model_radius ) {
                dist = model_radius;
            }

            dx = -dist * dx * wsize / nearp;
            dy = -dist * dy * 2.0;
            cur_cam.Move( dx, 0, dy );
            break;
        case 3:
            // Translate xy
            if( dist < model_radius ) {
                dist = model_radius;
            }

            dx = -dist * dx * wsize / nearp;
            dy = -dist * dy * wsize / nearp;
            cur_cam.Move( dx, dy, 0 );
            break;
        }
        break;
    case MOUSE_SHIFT:
        switch( mouse_button ) {
        case 1:
            // Look xy
            cur_cam.Pitch( dy * 1.5 );
            //cur_cam.Yaw( -dx * 1.5 );
            cur_cam.Yaw( dx * 1.5 );
            break;
        case 2:
            // Drive mode
            if( dist < model_radius ) {
                dist = model_radius;
            }

            cur_cam.Move( 0.0, 0.0, -dist * dy * 2.0 );
            cur_cam.Yaw( -dx * 1.5 );
            break;
        case 3:
            // Trackball model center
            cur_cam.Move( 0.0, 0.0, -dist );
            cur_cam.Yaw( -dx * 1.5 );
            cur_cam.Pitch( -dy * 1.5 );
            cur_cam.Move( 0.0, 0.0, dist );
            break;
        }
        break;
    case MOUSE_CONTROL:
        switch( mouse_button ) {
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        }
        break;
    case MOUSE_ALT:
        switch( mouse_button ) {
        case 1:
            // Drive mode
            if( dist < model_radius ) {
                dist = model_radius;
            }

            cur_cam.Move( 0.0, 0.0, -dist * dy * 2.0 );
            cur_cam.Yaw( -dx * 1.5 );
            break;
        case 2:
            break;
        case 3:
            break;
        }
        break;
    }

    Recompute_Near_Far_And_Wsize();

    prev_x = x;
    prev_y = y;
}

void Reset_Cameras( )
{
    int i;

    for( i = 0; i < MAX_CAMERAS; i++ ) {
        cams[i].Move_To( model_center.X() + model_radius * 2.0,
                         model_center.Y(), model_center.Z() );
        cams[i].Reset_Orientation( );
        cams[i].Reset_OrientationW( );
        cams[i].Reset_Angles( );
    }
    cur_cam.Move_To( model_center.X() + model_radius * 2.0,
                     model_center.Y(), model_center.Z() );
    cur_cam.Reset_Orientation( );
    cur_cam.Reset_OrientationW( );
    cur_cam.Reset_Angles( );
}

void Jump_To_Camera( int which )
{
#ifdef DECOMP_DEBUG
    if( which < 0 || which >= MAX_CAMERAS ) {
        cerr << "Error in Jump_To_Camera: position is out of range" << endl;
        return;
    }
#endif

    cur_cam = cams[which];
}

void Save_Camera( int which )
{
#ifdef DECOMP_DEBUG
    if( which < 0 || which >= MAX_CAMERAS ) {
        cerr << "Error in Save_Camera: position is out of range" << endl;
        return;
    }
#endif

    cams[which] = cur_cam;
}

void Reset_Current_Camera( )
{
    cur_cam.Reset_Orientation( );
    cur_cam.Reset_OrientationW( );
    cur_cam.Reset_Angles( );
}

void Create_Light( int which, SWIFT_Real x, SWIFT_Real y, SWIFT_Real z,
                   int dir, int attach_cam )
{
#ifdef DECOMP_DEBUG
    if( which < 0 || which >= MAX_LIGHTS ) {
        cerr << "Error in Create_Light: position is out of range" << endl;
        return;
    }
    if( lights[which].Exists() ) {
        cerr << "Error in Create_Light: light already exists" << endl;
        return;
    }
#endif

    lights[which].Create( x, y, z, dir, attach_cam );
}

void Destroy_Light( int which )
{
#ifdef DECOMP_DEBUG
    if( which < 0 || which >= MAX_LIGHTS ) {
        cerr << "Error in Destroy_Light: position is out of range" << endl;
        return;
    }
    if( !lights[which].Exists() ) {
        cerr << "Error in Destroy_Light: light does not exist" << endl;
        return;
    }
#endif

    lights[which].Destroy();
}

SWIFT_Triple View_X( )
{
    return cur_cam.X();
}

SWIFT_Triple View_Y( )
{
    return cur_cam.Y();
}

SWIFT_Triple View_Z( )
{
    return cur_cam.Z();
}

SWIFT_Real View_Distance( )
{
    return sqrt( model_center.Dist_Sq( cur_cam.Position() ) );
}

void Draw_World_Axes( )
{
    glBegin( GL_LINES );
#ifdef SWIFT_USE_FLOAT
    glVertex3f( 0.0, 0.0, (fabs(model_center.X())+model_radius) * 2.0 );
    glVertex3f( 0.0, 0.0, 0.0 );

    glVertex3f( 0.0, (fabs(model_center.Y())+model_radius) * 2.0, 0.0 );
    glVertex3f( 0.0, 0.0, 0.0 );

    glVertex3f( (fabs(model_center.Z())+model_radius) * 2.0, 0.0, 0.0 );
    glVertex3f( 0.0, 0.0, 0.0 );
#else
    glVertex3d( 0.0, 0.0, (fabs(model_center.X())+model_radius) * 2.0 );
    glVertex3d( 0.0, 0.0, 0.0 );

    glVertex3d( 0.0, (fabs(model_center.Y())+model_radius) * 2.0, 0.0 );
    glVertex3d( 0.0, 0.0, 0.0 );

    glVertex3d( (fabs(model_center.Z())+model_radius) * 2.0, 0.0, 0.0 );
    glVertex3d( 0.0, 0.0, 0.0 );
#endif
    glEnd();
}

void Draw_Model_Axes( )
{
    glBegin( GL_LINES );
#ifdef SWIFT_USE_FLOAT
    glVertex3f( model_center.X(), model_center.Y(),
                model_center.Z() + model_radius * 2.0 );
    glVertex3f( model_center.X(), model_center.Y(), model_center.Z() );

    glVertex3f( model_center.X(), model_center.Y() + model_radius * 2.0,
                model_center.Z() );
    glVertex3f( model_center.X(), model_center.Y(), model_center.Z() );

    glVertex3f( model_center.X() + model_radius * 2.0, model_center.Y(),
                model_center.Z() );
    glVertex3f( model_center.X(), model_center.Y(), model_center.Z() );
#else
    glVertex3d( model_center.X(), model_center.Y(),
                model_center.Z() + model_radius * 2.0 );
    glVertex3d( model_center.X(), model_center.Y(), model_center.Z() );

    glVertex3d( model_center.X(), model_center.Y() + model_radius * 2.0,
                model_center.Z() );
    glVertex3d( model_center.X(), model_center.Y(), model_center.Z() );

    glVertex3d( model_center.X() + model_radius * 2.0, model_center.Y(),
                model_center.Z() );
    glVertex3d( model_center.X(), model_center.Y(), model_center.Z() );
#endif
    glEnd();
}
#endif

