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
// main.C
//
//////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <iomanip>
using std::cerr;
using std::setprecision;
#include <stdlib.h>
#include <string.h>


#ifdef DECOMP_GRAPHICS
#include <togl.h>
#endif
#include "../SWIFT/SWIFT.h"
#include "../SWIFT/SWIFT_mesh.h"
#include "../SWIFT/SWIFT_fileio.h"

#include "gui.h"

#ifdef DECOMP_GRAPHICS
const char* TCL_SCRIPT = "decomposer.tcl";
#endif

// Command line options
#ifdef DECOMP_GRAPHICS
bool g = true;
#endif

bool jitter = false;
SWIFT_Real jampl = 0.0;

bool ef = false;
char* ef_filename = nullptr;
SWIFT_Real edge_flip_tol = 0.0; 

bool dfs = false;
bool bfs = false;
bool w = false;
bool one_piece = false;
char* decomp_filename = nullptr;

extern bool hierarchy;
char* hier_filename = nullptr;
SPLIT_TYPE split = MIDPOINT;


void Print_Usage( )
{
    cerr << "Usage: " << endl;
    cerr << "  decomposer [options] input_filename" << endl;
    cerr << "Options:" << endl;
    cerr << "  -h -help : This help information" << endl;
#ifdef DECOMP_GRAPHICS
    cerr << "  -g       : Run in command line mode (no graphics)" << endl;
#endif
    cerr << "  -j ampl  : Jitter the input at the given amplitude" << endl;
    cerr << "  -e err   : Edge flip using the given absolute error" << endl;
    cerr << "  -ef file : Save edge flip results to file (.tri)" << endl
         << endl;
    cerr << "  -1       : Decompose the model into only 1 piece" << endl;
    cerr << "  -dfs     : Run plain DFS decomposition" << endl;
    cerr << "  -bfs     : Run plain BFS decomposition" << endl;
    cerr << "  -cbfs    : Run cresting BFS decomposition (default)" << endl;
    cerr << "  -df file : Save decomposition to file (.dcp)"
         << endl << endl;
    cerr << "  -hier    : Compute hierarchy" << endl;
    cerr << "  -s split : Type of splitting when building hierarchy: MED, MID, MEAN, GAP" << endl;
    cerr << "  -hf file : Save hierarchy to file (.chr)" << endl << endl;
    cerr << "Default values: " << endl;
#ifdef DECOMP_GRAPHICS
    cerr << "  Graphics"  << endl;
#endif
    cerr << "  No jittering" << endl;
    cerr << "  No edge flipping" << endl;
    cerr << "  Run cresting BFS decomposition" << endl;
    cerr << "  Do not compute hierarchy" << endl;
    cerr << "  Split type MIDPOINT (if hierarchy computed)" << endl;
    cerr << "  No results are saved" << endl;
    exit( -1 );
}

#ifdef DECOMP_GRAPHICS
int tkAppInit( Tcl_Interp *interp )
{
    if( Tcl_Init( interp ) == TCL_ERROR) {
        return TCL_ERROR;
    }

    if( Tk_Init( interp ) == TCL_ERROR) {
        return TCL_ERROR;
    }

    if( Togl_Init( interp ) == TCL_ERROR) {
        return TCL_ERROR;
    }

    // Register new commands for the Togl widget
    Gui_Init_After_TclTk( interp );

    return TCL_OK;
}
#endif

int main( int argc, char **argv )
{
#ifdef DECOMP_GRAPHICS
    char* args[2];
#endif
    int i;

    cerr << setprecision( 20 );

#ifdef DECOMP_GRAPHICS
    cerr << "SWIFT++ Decomposer V1.0 -- Graphical --" << endl << endl;
#else
    cerr << "SWIFT++ Decomposer V1.0 -- Command Line --" << endl << endl;
#endif

    i = 1;
    while( argc != i ) {
        if( !strcmp( argv[i], "-h" ) || !strcmp( argv[i], "-help" ) ) {
            // Help
            Print_Usage();
#ifdef DECOMP_GRAPHICS
        } else if( !strcmp( argv[i], "-g" ) ) {
            // No graphics
            g = false;
#endif
        } else if( !strcmp( argv[i], "-e" ) ) {
            i++;
            ef = true;
            edge_flip_tol = (SWIFT_Real)atof( argv[i] );
        } else if( !strcmp( argv[i], "-ef" ) ) {
            i++;
            ef = true;
            ef_filename = new char[strlen(argv[i])+1];
            strcpy( ef_filename, argv[i] );
        } else if( !strcmp( argv[i], "-j" ) ) {
            i++;
            jampl = (SWIFT_Real)atof( argv[i] );
            jitter = true;
        } else if( !strcmp( argv[i], "-1" ) ) {
            // One piece
            one_piece = true;
        } else if( !strcmp( argv[i], "-dfs" ) ) {
            // Run plain dfs
            dfs = true;
            bfs = false;
        } else if( !strcmp( argv[i], "-bfs" ) ) {
            // Run plain bfs
            dfs = false;
            bfs = true;
        } else if( !strcmp( argv[i], "-cbfs" ) ) {
            // Run cresting bfs
            dfs = false;
            bfs = false;
        } else if( !strcmp( argv[i], "-df" ) ) {
            i++;
            w = true;
            decomp_filename = new char[strlen(argv[i])+1];
            strcpy( decomp_filename, argv[i] );
        } else if( !strcmp( argv[i], "-hier" ) ) {
            // Hierarchy computation
            hierarchy = true;
        } else if( !strcmp( argv[i], "-s" ) ) {
            i++;
            if( !strcmp( argv[i], "MED" ) ||
                !strcmp( argv[i], "med" ) || !strcmp( argv[i], "Med" )
            ) {
                split = MEDIAN;
            } else if( !strcmp( argv[i], "MID" ) ||
                !strcmp( argv[i], "mid" ) || !strcmp( argv[i], "Mid" )
            ) {
                split = MIDPOINT;
            } else if( !strcmp( argv[i], "MEAN" ) ||
                       !strcmp( argv[i], "mean" ) || !strcmp( argv[i], "Mean" )
            ) {
                split = MEAN;
            } else if( !strcmp( argv[i], "GAP" ) ||
                       !strcmp( argv[i], "gap" ) || !strcmp( argv[i], "Gap" )
            ) {
                split = GAP;
            } else {
                cerr << "Unrecognized split setting" << endl;
            }
        } else if( !strcmp( argv[i], "-hf" ) ) {
            i++;
            hierarchy = true;
            hier_filename = new char[strlen(argv[i])+1];
            strcpy( hier_filename, argv[i] );
        } else {
            // Assume that the rest of the parameters are inputs
            break;
        }
        i++;
    }

    if( argc == i ) {
        // No input file given
        cerr << "No input file given -- Exiting..." << endl;
        return 0;
    }


    // Create a swift scene so that the SWIFT utilities are initialized
    SWIFT_Scene scene;

    // Insert plug-in file reader registration here
    //My_File_Reader my_file_reader;
    //scene.Register_File_Reader( "MYNUM", my_file_reader );

    Gui_Init_Before_TclTk( argv[i] );

#ifdef DECOMP_GRAPHICS
    if( g ) {
        args[0] = argv[0];
            // Assume the script is in the current directory
            args[1] = new char[strlen(TCL_SCRIPT)+3];
            strcpy( args[1], "./" );
            strcat( args[1], TCL_SCRIPT );

        Tk_Main( 2, args, tkAppInit );

        delete args[1];
    }
#endif

    return 0;
}


