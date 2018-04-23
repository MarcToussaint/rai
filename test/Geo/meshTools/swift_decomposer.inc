#include <Kin/SWIFT/SWIFT.h>
#include <Kin/SWIFT/SWIFT_mesh.h>
#include <Kin/SWIFT/SWIFT_mesh_utils.h>
#include <Kin/SWIFT_decomposer/convex.h>
#include <Kin/SWIFT_decomposer/io.h>

uint decompose(mlr::Mesh& mesh, const char* filename, intA& triangleAssignments, mlr::Array<mlr::Array<uint> >& shapes){
  SWIFT_Scene swift;
  SWIFT_Tri_Mesh smesh;
  smesh.Create(mesh.V.p, (int*)mesh.T.p,
               mesh.V.d0, mesh.T.d0,
               DEFAULT_ORIENTATION,
               DEFAULT_TRANSLATION,
               1.);

  SWIFT_Array<bool> edge_convexities;
  SWIFT_Array<int> piece_ids;
  SWIFT_Array< SWIFT_Array<int> > model_faces;
  SWIFT_Array< SWIFT_Array<SWIFT_Tri_Face> > virtual_faces;

  Mesh_Utils_Initialize();
  smesh.Compute_Edge_Convexities( edge_convexities );
  /*cerr << "Jittering with amplitude = " << jampl << endl << endl;
  Jitter( mesh, jampl );
  cerr << "Flipping edges with tolerance = " << edge_flip_tol
       << endl << endl;
  Edge_Flip( mesh, edge_flip_tol );
  */
  Convex_Initialize( &smesh );
  Decompose_Cresting_BFS(&smesh, piece_ids, model_faces, virtual_faces );

  triangleAssignments.resize(piece_ids.Length());
  memmove(triangleAssignments.p, piece_ids.Data(), triangleAssignments.sizeT*triangleAssignments.N);
			 

  Save_Decomposition_File( filename, &smesh, piece_ids,
                           model_faces, virtual_faces );

  return piece_ids.Length();
}
