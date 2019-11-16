/** Header file for the Oxford version of Gilbert, Johnson and
   Keerthi's minimum distance routine.

   Version 2.4, July 1998, (c) Stephen Cameron 1996, 1997, 1998

*/

#pragma once

#include <stdio.h>
#include <stdlib.h>

/** define NBEDUG here to turn off checking by assert()
#define NDEBUG
 */
#include "assert.h"

//#define GATHER_STATISTICS

/** defining TEST_BACKUP_PROCEDURE disables the default simplex
 * distance routine, in order to test the (otherwise extremely
 * rarely used) backup procedure
 *
#define TEST_BACKUP_PROCEDURE
 */

#define DIM		3       /** dimension of space (i.e., x/y/z = 3) */
/** REAL is the type of a coordinate, INDEX of an index into the point arrays */
typedef double	REAL;
/** Arithmetic operators on type REAL: defined here to make it
   easy (say) to use fixed point arithmetic. Note that addition
   and subtraction are assumed to work normally.
   */

#define MULTIPLY( a, b)		((a)*(b))
#define DIVIDE( num, den)	((num)/(den))
	/** note that this definition of DOT_PRODUCT depends on DIM */
#define DOT_PRODUCT( a, b)	(     MULTIPLY( (a)[0],(b)[0]) + \
                                 MULTIPLY( (a)[1],(b)[1]) + \
                                 MULTIPLY( (a)[2],(b)[2]) )
#define ZERO			      ((REAL) 0)
                           /** given the need for addition to work ZERO
                              could hardly be anything else ... */
#define ONE			         ((REAL) 1)

#ifdef GATHER_STATISTICS
/** then we declare (in gjkdemo.c) and use five globally accessible counters,
   giving
     * the number of main loop iterations (equal to the number of times
       that the G-test is called),
     * the number of times that the simplex distance sub-routine is called,
     * the number of times that the backup procedure is called,
     * the total number of dot-product operations, and
     * the number of dot-product operations during the calculation of the
       support function.
   */
extern int gjk_num_g_test, gjk_num_simplices, gjk_num_backups,
  gjk_num_dot_products, gjk_num_support_dp, gjk_num_other_ops;

/** Now define the dot-product operations to increment the appropriate
   counters.
   */
#define OTHER_DOT_PRODUCT( a, b) \
	( gjk_num_dot_products++, DOT_PRODUCT( a, b) )
#define SUPPORT_DOT_PRODUCT( a, b) \
	( gjk_num_support_dp++, OTHER_DOT_PRODUCT( a, b) )
#define DO_MULTIPLY( a, b) \
	( gjk_num_other_ops++, MULTIPLY( a, b) )
#define DO_DIVIDE( a, b) \
	( gjk_num_other_ops++, DIVIDE( a, b) )

#define INCREMENT_G_TEST_COUNTER	gjk_num_g_test++
#define INCREMENT_SIMPLICES_COUNTER	gjk_num_simplices++
#define INCREMENT_BACKUP_COUNTER	gjk_num_backups++

#else /** not GATHER_STATISTICS */
#define OTHER_DOT_PRODUCT( a, b)	DOT_PRODUCT( a, b)
#define SUPPORT_DOT_PRODUCT( a, b)	DOT_PRODUCT( a, b)
#define DO_MULTIPLY( a, b)		MULTIPLY( a, b)
#define DO_DIVIDE( a, b)		DIVIDE( a, b)

#define INCREMENT_G_TEST_COUNTER
#define INCREMENT_SIMPLICES_COUNTER
#define INCREMENT_BACKUP_COUNTER
#endif /** not GATHER_STATISTICS */

/** Object structure: holds basic information about each object */
struct Object_structure {
  int numpoints;
  REAL **vertices;
  int *rings;
};
typedef struct Object_structure * Object;

/** Basic selectors, predicates and iterators for the Object structure;
   the idea is that you should be able to supply your own object structure,
   as long as you can fill in equivalent macros for your structure.
   */

#define InvalidVertexID			-1
#define FirstVertex( obj)		0
#define	NumVertices( obj)		(obj->numpoints)
#define IncrementVertex( obj, vertex)	(vertex++)
#define ValidVertex( obj, vertex)	(vertex>=0)
#define LastVertex( obj, vertex)	(vertex>=obj->numpoints)
#define SupportValue( obj, v, d)	SUPPORT_DOT_PRODUCT(obj->vertices[v],d)

#define VertexOfEdge( obj, edge)	(obj->rings[edge])
#define FirstEdge( obj, vertex)		(obj->rings[vertex])
#define ValidEdge( obj, edge)      	(obj->rings[edge]>=0)
#define IncrementEdge( obj, edge)	(edge++)

#define ValidRings( obj)		(obj->rings!=0)

/** The above set up for vertices to be stored in an array, and for
 * edges to be encoded within a single array of integers as follows.
 * Consider a hull whose vertices are stored in array
 * Pts, and edge topology in integer array Ring.  Then the indices of
 * the neighbouring vertices to the vertex with index i are Ring[j],
 * Ring[j+1], Ring[j+2], etc, where j = Ring[i] and the list of
 * indices are terminated with a negative number.  Thus the contents
 * of Ring for a tetrahedron could be
 *
 *  [ 4, 8, 12, 16,  1, 2, 3, -1,  0, 2, 3, -1,  0, 1, 3, -1,  0, 1, 2, -1 ]
 *
 */

/** We use a DIM x DIM+1 array to store transformations, with the
 * shorthand of a null pointer as an alternative (and easily checked for)
 * identity transformation.  For the `standard' case of
 * three-dimensional space the transformation matrix should be a 3x4
 * or 4x4 matrix, designed so that if M is the matrix, R is the
 * rotation sub-matrix of the first 3 rows and columns, t is the
 * translation vector encoded in the fourth column, and x is a column
 * position vector, then M maps x to (Rx + t).  Eqivalently, M maps
 * x to the vector y, where
 *   y[i] = M[i][0]*x[0] + M[i][1]*x[1] + M[i][2]*x[2] + M[i][3]
 */
typedef double **Transform;
#define IdentityTransform( t)	((t)==0)
#define ExtractTranslation( t, v) { int i; overd(i) v[i] = t[i][DIM]; }
#define ApplyTransform( t, obj, v, tgt) apply_trans( t, obj->vertices[v], tgt)
#define ApplyInverseRotation( t, tgt, src) apply_rot_transpose( t, tgt, src)

void apply_trans( Transform, REAL *, REAL *);
void apply_rot_transpose( Transform, REAL *, REAL *);

/** typedefs for `opaque' pointers to a vertex and to an edge */
typedef	int	VertexID;
typedef int	EdgeID;

#ifdef GATHER_STATISTICS
extern int
      gjk_num_g_test,     /** how many times the G-test is performed -- the
                             same as the number of main-loop iterations */
      gjk_num_simplices,  /** how many times the simplex routine
                             was called */
      gjk_num_backups,    /** how many times (if ever!) the GJK backup
                             procedure was called */
      gjk_num_dot_products, /** how many dot-product operations are called */
      gjk_num_support_dp, /** how many dot-product operations are called
			      whilst executing the support function */
      gjk_num_other_ops; /** how many other mults and divides are called */

#endif

/** The simplex_point structure is really designed to be private to the
   main GJK routines.  However the calling routine may wish to allocate
   some to be used to cache information for the distance routines.
   */
struct simplex_point {
  int    npts;   /** number of points in this simplex */
                 /** simplex1 and simplex2 are two arrays of
		    indices into the point arrays, given by the
		    user. */
  VertexID simplex1[DIM+1];
  VertexID simplex2[DIM+1];
                 /** and lambdas gives the values of lambda
		    associated with those points.
		 */
  REAL   lambdas[DIM+1];

  REAL   coords1[DIM+1][DIM];
  REAL   coords2[DIM+1][DIM];
                 /** calculated coordinates from the last iteration */

  VertexID last_best1, last_best2;
                /** last maximal vertices, used for hill-climbing */
  double error; /** indication of maximum error in the return value */
  /** This value is that returned by the `G-test', and indicates the
     difference between the reported shortest distance vector and the
     vector constructed from supporting hyperplanes in the direction
     of the reported shortest distance vector.  That is, if the reported
     shortest distance vector is x, and the hyperplanes are distance D
     apart, this value is G = x.x - D|x|, and should be exactly zero
     for the true shortest distance vector (as |x| should then equal
     D).

     Alternatively, G/(x.x) is a relative error bound on the result.
  */
};

/** Even this algorithm has an epsilon (fudge) factor.  It basically indicates
   how far apart two points have to be to declared different, expressed 
   loosely as a proportion of the `average distance' between the point sets.
 */
#define EPSILON ((REAL) 1.0e-8)

/** TINY is used in one place, to indicate when a positive number is getting
   so small that we loose confidence in being able to divide a positive
   number smaller than it into it, and still believing the result.
   */
#define TINY	((REAL) 1.0e-20)  /** probably pessimistic! */

/** MAX_RING_SIZE gives an upper bound on the size of the array of rings
 * of edges in terms of the number of vertices.  From the formula
 *   v - e + f = 2
 * and the relationships that there are two half-edges for each edge,
 * and at least 3 half-edges per face, we obtain
 *   h <= 6v - 12
 * Add to this another v entries for the initial pointers to each ring,
 * another v entries to indicate the end of each ring list, and round it
 * up.
 */
#define MAX_RING_SIZE_MULTIPLIER	8

/*
 * A subsidary routine, that given a simplex record,
 * an integer 1 or 2, and a pointer to a vector, sets
 * the coordinates of that vector to the coordinates of either the first
 * or second witness point given by the simplex record.
 */
int gjk_extract_point( struct simplex_point *simp,
		       int whichpoint, REAL vector[]);

/** The main GJK distance routine.  This routine implements the routine
 * of Gilbert, Johnson and Keerthi, as described in the paper (GJK88)
 * listed below.  It also optionally runs my speed-up extension to this
 * algorithm, as described in (Cam97).
 
 *
 * The first 4 parameters are two pairs of parameters, one pair for
 * each hull; each pair is an object data-structure, plus a
 * transformation data-structure.  These data-structures are defined
 * to this code in gjk.h, and are designed to be opaque to this code;
 * the data is accessed through selectors, iterators and predicates,
 * which are discussed below.
 *
 * The 5th and 6th parameters are point arrays, that are set to the
 * coordinates of two witness points (one within each convex hull)
 * that realise the minimum distance between them.
 *
 * The actual return value for the function is the square of the
 * minimum distance between the hulls, which is equal to the (square
 * of the) distance between the witness points.
 *
 * The 7th parameter is a pointer to a simplex_point structure.  If
 * this is non-nullptr then a special form of the witness points is
 * stored in the structure by the routine, suitable for passing to
 * this routine as seed points for any further calls involving these
 * two objects. The 8th parameter is a flag, which when set tells the
 * routine to use the given simplex_point structure instance as as
 * seed, otherwise it just uses any seed.  (If the 7th parameter is
 * nullptr then no harm is done.)
 *
 * Note that with this version one field of the simplex_point structure
 * can be used to pass back the confidence region for the routine: when
 * the routine returns with distance squared equal to D*d, it means that
 * the true distance is known to lie between D-(E/D) and D, where E is
 * the positive value returned in the `error' field of the simplex_point.
 * Equivalently; the true value of the distance squared is less than or equal
 * to the value returned DSQ, with an error bound width of 2E - E*E/DSQ.
 * (In `normal' cases E is very small, and so the error bound width 2E can
 * be sensibly used.)
 *
 * The code will attempt to return with E<=EPSILON, which the user
 * can set, but will in any event return with some value of E.  In particular,
 * the code should return even with EPSILON set to zero.
 *
 * Alternatively, either or both of the pointer values for the witness
 * points can be zero, in which case those witness points are not
 * returned.  The caller can later extract the coordinates of the
 * witness points from the simplex_point structure by using the
 * function gjk_extract_point.
 *
 * Returning to the opaque data-structures used to describe the objects
 * and their transformations.  For an object then the minimum information
 * required is a list of vertices.  The vertices themselves are another
 * opaque type, accessed through the type VertexID.  The following macros
 * are defined for the vertices:
 *
 *  InvalidVertexID	 a VertexID which cannot point to a valid vertex
 *  FirstVertex( obj)	 an arbitrary first vertex for the object
 *  NumVertices( obj)	 the number of vertices in the object
 *  IncrementVertex(o,v) set vertex to the next vertex
 *  ValidVertex( obj, v) says whether the VertexID is valid for obj
 *  LastVertex( obj, v)  is this the last vertex?
 *  SupportValue(o,v,d)  returns support value for v in direction d
 *
 * Optionally, the object data-structures encode the wireframe of the objects;
 * this is used in my extended GJK algorithm to greatly speed up the routine
 * in many cases.  For an object the predicate ValidRing( obj) says whether
 * this information is provided, in which case the edges that surround any
 * can be accessed and traversed with the following:
 *
 *  FirstEdge( obj, vertex)	Returns the first edge (type EdgeID)
 *  IncrementEdge( obj, edge)	Sets edge to the next edge
 *  ValidEdge( obj, edge)      	Indicates whether edge is a real edge
 *  VertexOfEdge( obj, edge)	Returns the (other) vertex of an edge
 *
 * With this information this routine runs in expected constant time
 * for tracking operations and small relative motions.  If the
 * information is not supplied the the routine reverts to using the
 * original GJK technique, which takes time roughly linear in the number
 * of vertices.  (As a rough rule of thumb, this difference becomes
 * measurable at around 10 vertices per hull, and important at about
 * 20 vertices per hull.)
 *
 * Transformations are stored in data-structures given by opaque type
 * Transform, for which the following operations need to be defined:
 *
 *  IdentityTransform( t)	Might t be an identity transformation?
 *  ExtractTranslation( t, v)	Set v to the translation component of t
 *  ApplyTransform( t,o,v,tgt)  Apply transform to vertex v of o, result in tgt
 *  ApplyInverseRotation(t,d,r) Apply inverse of t to direction d, result in r
 *
 * Notes:
 *  + it is OK for IdentityTransform( t) to return false when t is
 *    in fact the identity (with a small time penalty)
 *  + ExtractTranslation equivalently sets v to where the origin is
 *    transformed to by t
 *
 * References:
 * (GJK88) "A Fast Procedure for Computing the Distance between Complex
 * Objects in Three-Dimensional Space" by EG Gilbert, DW Johnson and SS
 * Keerthi, IEEE Trans Robotics and Automation 4(2):193--203, April 1988.
 *
 * (Cam97) "A Comparison of Two Fast Algorithms for Computing the Distance
 * between Convex Polyhedra" by Stephen Cameron, IEEE Trans Robotics and
 * Automation 13(6):915-920, December 1997.
 *
 */
REAL gjk_distance(
   Object obj1, REAL **tr1,
   Object obj2, REAL **tr2,
   REAL *wpt1, REAL *wpt2,
   struct simplex_point * simplex, int use_seed
   );
