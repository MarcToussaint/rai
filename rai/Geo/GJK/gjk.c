#include "gjk.h"

/*
 * Implementation of the Gilbert, Johnson and Keerthi routine to compute
 * the minimum distance between two convex polyhedra.
 *
 * Version 2.4, July 1998, (c) Stephen Cameron 1996, 1997, 1998
 *
 */

/* The code uses some tables that essentially encode the constant
   topology of simplices in DIM-dimensional space.  The original
   version of this program did this by constructing the tables
   each time the code was activated (but not once per run of gjk_distance!).
   In this version of the code we can define the constant USE_CONST_TABLES
   to use pre-defined versions of these tables instead.

   Not defining USE_CONST_TABLES makes the code behave as before.  This
   takes around 5000 integer operations at code initialisation when
   DIM==3.

   Defining USE_CONST_TABLES uses the pre-defined tables.

   Defining DUMP_CONST_TABLES makes the code construct appropriate
   versions of the tables for space of dimension <= DIM on the first
   call of gjk_distance, and to simply dump these to standard ouput and
   immediately call exit(0) to abort the programme.  This may be used if
   either the pre-defined tables have become corrupted, or if you wish
   to use pre-defined tables for a higher dimension.
*/
#define USE_CONST_TABLES
/**/

/* standard definitions, derived from those in gjk.h */

#define TWO_TO_DIM	   (1<<DIM)	/* must have TWO_TO_DIM = 2^DIM */
#define DIM_PLUS_ONE	   (DIM+1)
#define TWICE_TWO_TO_DIM   (TWO_TO_DIM+TWO_TO_DIM)

#define overd( ct)         for ( ct=0 ; ct<DIM ; ct++ )

/* The following #defines are defined to make the code easier to
   read: they are simply standard accesses of the following
   arrays.
   */
#define card( s)	cardinality[s]
#define max_elt( s)	max_element[s]
#define elts( s, i)	elements[s][i]
#define non_elts( s, i)	non_elements[s][i]
#define pred( s, i)	predecessor[s][i]
#define succ( s, i)	successor[s][i]
#define delta( s, i)	delta_values[s][i]
#define prod( i, j)	dot_products[i][j]

/* Decide whether the code the construct the constant tables should be
   linked into the source; this either happens if we have been asked to
   dump the tables to standard output, or if we have not been asked to use
   the pre-defined tables.
*/
#ifdef DUMP_CONST_TABLES
#define CONSTRUCT_TABLES
#else
#ifndef USE_CONST_TABLES
#define CONSTRUCT_TABLES
#endif /* USE_CONST_TABLES */
#endif /* DUMP_CONST_TABLES */

/* The following arrays store the constant subset structure -- see the
   comments in initialise_simplex_distance() for the data-invariants.
   Note that the entries could easily be packed, as say for any subset
   with index s we have only DIM_PLUS_ONE active entries in total for
   both elts( s,) and non_elts( s,), and ditto for prec( s,) and succ( s,).
   We have not bothered here as the tables are small.
   */
#ifdef CONSTRUCT_TABLES
static int cardinality[TWICE_TWO_TO_DIM];
static int max_element[TWICE_TWO_TO_DIM];
static int elements[TWICE_TWO_TO_DIM][DIM_PLUS_ONE];
static int non_elements[TWICE_TWO_TO_DIM][DIM_PLUS_ONE];
static int predecessor[TWICE_TWO_TO_DIM][DIM_PLUS_ONE];
static int successor[TWICE_TWO_TO_DIM][DIM_PLUS_ONE];
#else
/* the output of the routine to dump the six arrays above should be
   interpolated between this line and the following #endif */

#define PRE_DEFINED_TABLE_DIM 3

static const int cardinality[16] = {
	 0,  1,  1,  2,  1,  2,  2,  3,  1,  2,  2,  3,  2,  3,  3,  4};
static const int max_element[16] = {
	-1,  0,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  3};
static const int elements[16][4] = {
	{  0,  0,  0,  0},
	{  0,  0,  0,  0},
	{  1,  0,  0,  0},
	{  0,  1,  0,  0},
	{  2,  0,  0,  0},
	{  0,  2,  0,  0},
	{  1,  2,  0,  0},
	{  0,  1,  2,  0},
	{  3,  0,  0,  0},
	{  0,  3,  0,  0},
	{  1,  3,  0,  0},
	{  0,  1,  3,  0},
	{  2,  3,  0,  0},
	{  0,  2,  3,  0},
	{  1,  2,  3,  0},
	{  0,  1,  2,  3} };
static const int non_elements[16][4] = {
	{  0,  1,  2,  3},
	{  1,  2,  3,  0},
	{  0,  2,  3,  0},
	{  2,  3,  0,  0},
	{  0,  1,  3,  0},
	{  1,  3,  0,  0},
	{  0,  3,  0,  0},
	{  3,  0,  0,  0},
	{  0,  1,  2,  0},
	{  1,  2,  0,  0},
	{  0,  2,  0,  0},
	{  2,  0,  0,  0},
	{  0,  1,  0,  0},
	{  1,  0,  0,  0},
	{  0,  0,  0,  0},
	{  0,  0,  0,  0} };
static const int predecessor[16][4] = {
	{  0,  0,  0,  0},
	{  0,  0,  0,  0},
	{  0,  0,  0,  0},
	{  2,  1,  0,  0},
	{  0,  0,  0,  0},
	{  4,  1,  0,  0},
	{  4,  2,  0,  0},
	{  6,  5,  3,  0},
	{  0,  0,  0,  0},
	{  8,  1,  0,  0},
	{  8,  2,  0,  0},
	{ 10,  9,  3,  0},
	{  8,  4,  0,  0},
	{ 12,  9,  5,  0},
	{ 12, 10,  6,  0},
	{ 14, 13, 11,  7} };
static const int successor[16][4] = {
	{  1,  2,  4,  8},
	{  3,  5,  9,  0},
	{  3,  6, 10,  0},
	{  7, 11,  0,  0},
	{  5,  6, 12,  0},
	{  7, 13,  0,  0},
	{  7, 14,  0,  0},
	{ 15,  0,  0,  0},
	{  9, 10, 12,  0},
	{ 11, 13,  0,  0},
	{ 11, 14,  0,  0},
	{ 15,  0,  0,  0},
	{ 13, 14,  0,  0},
	{ 15,  0,  0,  0},
	{ 15,  0,  0,  0},
	{  0,  0,  0,  0} };

#endif /* CONSTRUCT_TABLES */

static REAL delta_values[TWICE_TWO_TO_DIM][DIM_PLUS_ONE];
static REAL dot_products[DIM_PLUS_ONE][DIM_PLUS_ONE];

#ifdef CONSTRUCT_TABLES
static void initialise_simplex_distance( void);
#endif

static VertexID support_function( Object obj,
			       VertexID, REAL *, REAL *);
static VertexID support_simple( Object obj,
			       VertexID, REAL *, REAL *);
static VertexID support_hill_climbing( Object obj,
			       VertexID, REAL *, REAL *);

static int default_distance( struct simplex_point * simplex);
static void backup_distance( struct simplex_point * simplex);
static void reset_simplex( int subset, struct simplex_point * simplex);

static void compute_subterms( struct simplex_point * s);
static void compute_point( REAL pt[DIM], int len,
		   REAL (* vertices)[DIM], REAL *lambdas);
static void add_simplex_vertex( struct simplex_point * s, int pos,
				Object obj1, VertexID v1, Transform t1,
				Object obj2, VertexID v2, Transform t2);


/* The main GJK distance routine.  This routine implements the routine
 * of Gilbert, Johnson and Keerthi, as described in the paper (GJK88)
 * listed below.  It also optionally runs my speed-up extension to this
 * algorithm, as described in (Cam97).
 
 *
 * The first 4 parameters are two pairs of parameters, one pair for
 * each hull; each pair is an object data-structure, plus a
 * transformation data-structure.  These data-structures are defined
 * to this code in gjk.h, and are designed to be opaque to this code;
 * the data is accessed through selectors, iterators and prediciates,
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
   Object obj1, Transform tr1,
   Object obj2, Transform tr2,
   REAL *wpt1, REAL *wpt2,
   struct simplex_point * simplex, int use_seed
   ) {

   VertexID v, p, maxp, minp;
   REAL minus_minv, maxv, sqrd, g_val;
   REAL displacementv[DIM], reverse_displacementv[DIM];
   REAL local_witness1[DIM], local_witness2[DIM];
   REAL local_fdisp[DIM], local_rdisp[DIM], trv[DIM];
   REAL * fdisp, * rdisp;
   struct simplex_point local_simplex;
   int d, compute_both_witnesses, use_default, first_iteration, max_iterations;
   double oldsqrd;

//   int i;
//   for(i=0; i<obj1->numpoints; i++) printf("obj1 v%i x=%g y=%g z=%g\n", i, obj1->vertices[i][0], obj1->vertices[i][1], obj1->vertices[i][2]);
//   for(i=0; i<obj2->numpoints; i++) printf("obj2 v%i x=%g y=%g z=%g\n", i, obj2->vertices[i][0], obj2->vertices[i][1], obj2->vertices[i][2]);

   assert( NumVertices(obj1)>0 && NumVertices(obj2)>0 );

   use_default = first_iteration = 1;
#ifdef CONSTRUCT_TABLES
   initialise_simplex_distance();
		/* will return immediately if already initialised */
#else
   assert( PRE_DEFINED_TABLE_DIM >= DIM );
#endif /* CONSTRUCT_TABLES */

   compute_both_witnesses = ( wpt1!=0 ) || ( wpt2!=0 ) ||
                            (  tr1!=0 ) || (  tr2!=0 );

   if ( wpt1==0 )
       wpt1 = local_witness1;

   if ( wpt2==0 )
       wpt2 = local_witness2;

   fdisp = IdentityTransform(tr1) ?         displacementv : local_fdisp;
   rdisp = IdentityTransform(tr2) ? reverse_displacementv : local_rdisp;

   if ( simplex==0 ) {
      use_seed = 0;
      simplex = & local_simplex;
   }

   if ( use_seed==0 ) {
      simplex->simplex1[0] = 0;    simplex->simplex2[0] = 0;
      simplex->npts = 1;           simplex->lambdas[0] = ONE;
      simplex->last_best1 = 0;     simplex->last_best2 = 0;
      add_simplex_vertex( simplex, 0,
			  obj1, FirstVertex( obj1), tr1,
			  obj2, FirstVertex( obj2), tr2);
   }
   else {
      /* If we are being told to use this seed point, there
         is a good chance that the near point will be on
         the current simplex.  Besides, if we don't confirm
         that the seed point given satisfies the invariant
         (that the witness points given are the closest points
         on the current simplex) things can and will fall down.
         */
      for ( v=0 ; v<simplex->npts ; v++ )
	add_simplex_vertex( simplex, v,
          obj1, simplex->simplex1[v], tr1,
	  obj2, simplex->simplex2[v], tr2);
   }

   /* Now the main loop.  We first compute the distance between the
      current simplicies, the check whether this gives the globally
      correct answer, and if not construct new simplices and try again.
      */

   max_iterations = NumVertices( obj1)*NumVertices( obj2) ;
      /* in practice we never see more than about 6 iterations. */

   /* Counting the iterations in this way should not be necessary;
      a while( 1) should do just as well. */
   while ( max_iterations-- > 0 ) {

     if ( simplex->npts==1 ) { /* simple case */
       simplex->lambdas[0] = ONE;
     }
     else { /* normal case */
       compute_subterms( simplex);
       if ( use_default ) { 
	 use_default = default_distance( simplex);
       }
       if ( !use_default ) {
	 backup_distance( simplex);
       }
     }

     /* compute at least the displacement vectors given by the
	simplex_point structure.  If we are to provide both witness
	points, it's slightly faster to compute those first.
     */
     if ( compute_both_witnesses ) {
       compute_point( wpt1, simplex->npts, simplex->coords1,
		      simplex->lambdas);
       compute_point( wpt2, simplex->npts, simplex->coords2,
		      simplex->lambdas);
      
       overd( d) {
	 displacementv[ d]         = wpt2[d] - wpt1[d];
	 reverse_displacementv[ d] = - displacementv[d];
       }
     }
     else {
       overd( d) {
	 displacementv[d] = 0;
	 for ( p=0 ; p<simplex->npts ; p++ )
	   displacementv[d] +=
	     DO_MULTIPLY( simplex->lambdas[p],
			  simplex->coords2[p][d] - simplex->coords1[p][d]);
	 reverse_displacementv[ d] = - displacementv[d];
       }
     }
	 
     sqrd = OTHER_DOT_PRODUCT( displacementv, displacementv);

     /* if we are using a c-space simplex with DIM_PLUS_ONE
	points, this is interior to the simplex, and indicates
	that the original hulls overlap, as does the distance 
	between them being too small. */
     if ( sqrd<EPSILON ) {
       simplex->error = EPSILON;
       return sqrd;
     }

     if ( ! IdentityTransform( tr1) )
       ApplyInverseRotation( tr1,         displacementv, fdisp);

     if ( ! IdentityTransform( tr2) )
       ApplyInverseRotation( tr2, reverse_displacementv, rdisp);

     /* find the point in obj1 that is maximal in the
	direction displacement, and the point in obj2 that
	is minimal in direction displacement;
     */
     maxp = support_function(
		  obj1,
		  ( use_seed ? simplex->last_best1 : InvalidVertexID),
		  &maxv, fdisp
		  );

     minp = support_function(
		  obj2,
		  ( use_seed ? simplex->last_best2 : InvalidVertexID),
		  &minus_minv, rdisp
		  );

     /* Now apply the G-test on this pair of points */

     INCREMENT_G_TEST_COUNTER;

     g_val = sqrd + maxv + minus_minv;

     if ( ! IdentityTransform( tr1) ) {
       ExtractTranslation( tr1, trv);
       g_val += OTHER_DOT_PRODUCT(         displacementv, trv);
     }

     if ( ! IdentityTransform( tr2) ) {
       ExtractTranslation( tr2, trv);
       g_val += OTHER_DOT_PRODUCT( reverse_displacementv, trv);
     }

     if ( g_val < 0.0 )  /* not sure how, but it happens! */
       g_val = 0;

     if ( g_val < EPSILON ) {
       /* then no better points - finish */
       simplex->error = g_val;
       return sqrd;
     }

     /* check for good calculation above */
     if ( (first_iteration || (sqrd < oldsqrd))
	  && (simplex->npts <= DIM ) ) {
       /* Normal case: add the new c-space points to the current
	  simplex, and call simplex_distance() */
       simplex->simplex1[ simplex->npts] = simplex->last_best1 = maxp;
       simplex->simplex2[ simplex->npts] = simplex->last_best2 = minp;
       simplex->lambdas[ simplex->npts] = ZERO;
       add_simplex_vertex( simplex, simplex->npts,
			   obj1, maxp, tr1,
			   obj2, minp, tr2);
       simplex->npts++;
       oldsqrd = sqrd;
       first_iteration = 0;
       use_default = 1;
       continue;
     }

     /* Abnormal cases! */ 
     if ( use_default ) {
       use_default = 0;
     }
     else { /* give up trying! */
       simplex->error = g_val;
       return sqrd;
     }
   } /* end of `while ( 1 )' */

   return 0.0; /* we never actually get here, but it keeps some
                  fussy compilers happy.  */
}

/*
 * A subsidary routine, that given a simplex record, the point arrays to
 * which it refers, an integer 1 or 2, and a pointer to a vector, sets
 * the coordinates of that vector to the coordinates of either the first
 * or second witness point given by the simplex record.
 */
int gjk_extract_point( struct simplex_point *simp,
		       int whichpoint, REAL vector[]) {

  REAL (* coords)[DIM];

  assert( whichpoint==1 || whichpoint==2 );

  coords = ( whichpoint==1 ) ? simp->coords1 : simp->coords2;
  compute_point( vector, simp->npts, coords, simp->lambdas);

  return 1;
}

static REAL delta[TWICE_TWO_TO_DIM];

/* The simplex_distance routine requires the computation of a number of
   delta terms.  These are computed here.
 */
static void compute_subterms( struct simplex_point * simp) {

   int i, j, ielt, jelt, s, jsubset, size = simp->npts;
   REAL sum, c_space_points[DIM_PLUS_ONE][DIM];
   
   /* compute the coordinates of the simplex as C-space obstacle points */
   for ( i=0 ; i<size ; i++ )
      for ( j=0 ; j<DIM ; j++ )
         c_space_points[i][j] =
            simp->coords1[i][j] - simp->coords2[i][j];
            
   /* compute the dot product terms */
   for ( i=0 ; i<size ; i++ )
      for ( j=i ; j<size ; j++ )
         prod( i, j) = prod( j, i) =
            OTHER_DOT_PRODUCT( c_space_points[i], c_space_points[j]);

   /* now compute all the delta terms */
   for ( s=1 ; s<TWICE_TWO_TO_DIM && max_elt( s) < size ; s++ ) {
      if ( card( s)<=1 ) {  /* just record delta(s, elts(s, 0)) */
         delta( s, elts( s, 0)) = ONE;
         continue;
      }
         
      if ( card( s)==2 ) {  /* the base case for the recursion */
         delta( s, elts( s, 0)) =
            prod( elts( s, 1), elts( s, 1)) -
            prod( elts( s, 1), elts( s, 0));
         delta( s, elts( s, 1)) =
            prod( elts( s, 0), elts( s, 0)) -
            prod( elts( s, 0), elts( s, 1));
         continue;
      }

      /* otherwise, card( s)>2, so use the general case */

      /* for each element of this subset s, namely elts( s, j) */
      for ( j=0 ; j<card( s) ; j++ ) {
         jelt = elts( s, j);
         jsubset = pred( s, j);
         sum = 0;
         /* for each element of subset jsubset */   
         for ( i=0 ; i < card( jsubset) ; i++ ) {
            ielt = elts( jsubset, i);
            sum += DO_MULTIPLY(
               delta( jsubset, ielt ),
               prod( ielt, elts( jsubset, 0)) - prod( ielt, jelt)
                  );
	 }
            
         delta( s, jelt) = sum;
      }
   }
   return;
}


/*
 * default_distance is our equivalent of GJK's distance subalgorithm.
 * It is given a c-space simplex as indices of size (up to DIM_PLUS_ONE) points
 * in the master point list, and computes a pair of witness points for the
 * minimum distance vector between the simplices.  This vector is indicated
 * by setting the values lambdas[] in the given array, and returning the
 * number of non-zero values of lambda. 
 */
 
static int default_distance( struct simplex_point * simplex) {

   int s, j, k, ok=0, size;

   size = simplex->npts;

   INCREMENT_SIMPLICES_COUNTER;

   assert( size>0 && size<=DIM_PLUS_ONE );


   /* for every subset s of the given set of points ...
      */
   for ( s=1 ; s < TWICE_TWO_TO_DIM && max_elt( s) < size ; s++ ) {
      /* delta[s] will accumulate the sum of the delta expressions for
         this subset, and ok will remain TRUE whilst this subset can
         still be thought to be a candidate simplex for the shortest
         distance.
         */
      delta[s] = ZERO;	ok=1;

      /* Now the first check is whether the simplex formed by this
         subset holds the foot of the perpendicular from the origin
         to the point/line/plane passing through the simplex. This will
         be the case if all the delta terms for each predecessor subset
         are (strictly) positive.
         */
      for ( j=0 ; ok && j<card( s) ; j++ ) {
         if ( delta( s, elts( s, j))>ZERO )
            delta[s] += delta( s, elts( s, j));
         else
            ok = 0;
      }

      /* If the subset survives the previous test, we still need to check
         whether the true minimum distance is to a larger piece of geometry,
         or indeed to another piece of geometry of the same dimensionality.
         A necessary and sufficient condition for it to fail at this stage
         is if the delta term for any successor subset is positive, as this
         indicates a direction on the appropriate higher dimensional simplex
         in which the distance gets shorter.
         */ 

      for ( k=0 ; ok && k < size - card( s) ; k++ ) {
         if ( delta( succ( s, k),  non_elts( s, k))>0 )
            ok = 0;
      }
            
#ifdef TEST_BACKUP_PROCEDURE
      /* define TEST_BACKUP_PROCEDURE in gjk.h to test accuracy
         of the the backup procedure */
      ok = 0;
#endif

      if ( ok && delta[s]>=TINY )   /* then we've found a viable subset */
         break;
   }

   if ( ok ) {
     reset_simplex( s, simplex);
     return 1;
   }
   else
     return 0;
}

/* A version of GJK's `Backup Procedure'.
   Note that it requires that the delta[s] entries have been
   computed for all viable s within simplex_distance.
   */
static void backup_distance( struct simplex_point * simplex) {

   int s, i, j, k, bests;
   int size = simplex->npts;
   REAL distsq_num[TWICE_TWO_TO_DIM], distsq_den[TWICE_TWO_TO_DIM];

   INCREMENT_BACKUP_COUNTER;

   /* for every subset s of the given set of points ...
      */
   bests = 0;
   for ( s=1 ; s < TWICE_TWO_TO_DIM && max_elt( s) < size ; s++ ) {
      if ( delta[s] <= ZERO )
         continue;

      for ( i=0 ; i<card( s) ; i++ )
         if ( delta( s, elts( s, i))<=ZERO )
            break;

      if (  i < card( s) )
         continue;

      /* otherwise we've found a viable subset */
      distsq_num[s] = ZERO;
      for ( j=0 ; j<card( s) ; j++ )
         for ( k=0 ; k<card( s) ; k++ )
            distsq_num[s] += DO_MULTIPLY( 
               DO_MULTIPLY( delta( s, elts( s, j)), delta( s, elts( s, k))),
               prod( elts( s, j), elts( s, k))
	                             );

      distsq_den[s] = DO_MULTIPLY( delta[s], delta[s]);

      if ( (bests < 1) ||
            ( DO_MULTIPLY( distsq_num[s], distsq_den[bests]) <
              DO_MULTIPLY( distsq_num[bests], distsq_den[s]) )
             )
         bests = s;
      }

   reset_simplex( bests, simplex);

   return;
}

static void reset_simplex( int subset, struct simplex_point * simplex) {

  int i, j, oldpos;

   /* compute the lambda values that indicate exactly where the
      witness points lie.  We also fold back the values stored for the
      indices into the original point arrays, and the transformed
      coordinates, so that these are ready for subsequent calls.
    */
   for ( j=0 ; j<card( subset) ; j++ ) {
     /* rely on elts( subset, j)>=j, which is true as they are
	stored in ascending order.   */
      oldpos = elts( subset, j);
      if ( oldpos!=j ) {
	  simplex->simplex1[j] = simplex->simplex1[oldpos];
	  simplex->simplex2[j] = simplex->simplex2[oldpos];
	  overd(i ) {
	    simplex->coords1[j][i] = simplex->coords1[oldpos][i];
	    simplex->coords2[j][i] = simplex->coords2[oldpos][i];
	  }
      }

      simplex->lambdas[j] =
	DO_DIVIDE( delta( subset, elts( subset, j)), delta[subset]);
   }
   simplex->npts = card( subset);

   return;
}

/*
 * The implementation of the support function.  Given a direction and
 * a hull, this function returns a vertex of the hull that is maximal
 * in that direction, and the value (i.e., dot-product of the maximal
 * vertex and the direction) associated.
 *
 * If there is no topological information given for the hull
 * then an exhaustive search of the vertices is used.  Otherwise,
 * hill-climbing is performed.  If EAGER_HILL_CLIMBING is defined
 * then the hill-climbing moves to a new vertex as soon as a better
 * vertex is found, and if it is not defined then every vertex leading
 * from the current vertex is explored before moving to the best one.
 * Initial conclusions are that fewer vertices are explored with
 * EAGER_HILL_CLIMBING defined, but that the code runs slighty slower!
 * This is presumably due to pipeline bubbles and/or cache misses.
 *
 */

static VertexID
support_function(
		 Object obj,
		 VertexID start, REAL * supportval, REAL * direction) {

  if ( !ValidRings( obj) ) {
    /* then no information for hill-climbing.  Use brute-force instead. */
    return support_simple( obj, start, supportval, direction);
  }
  else {
    return support_hill_climbing( obj, start, supportval, direction);
  }
}

static VertexID
support_simple(
		 Object obj,
		 VertexID start, REAL * supportval, REAL * direction) {

  VertexID p, maxp;
  REAL maxv, thisv;

  /* then no information for hill-climbing.  Use brute-force instead. */

  p = maxp = FirstVertex( obj);
  maxv = SupportValue( obj, maxp, direction);
  for ( IncrementVertex( obj, p) ;
	!LastVertex( obj, p) ;
	IncrementVertex( obj, p) ) {
    thisv = SupportValue( obj, p, direction);
    if ( thisv>maxv ) {
      maxv = thisv;
      maxp = p;
    }
  }

  *supportval = maxv;
  return maxp;
}

static VertexID
support_hill_climbing(
		 Object obj,
		 VertexID start, REAL * supportval, REAL * direction) {

  VertexID p, maxp, lastvisited, neighbour;
  EdgeID index;
  REAL maxv, thisv;

  /* Use hill-climbing */
  p = lastvisited = InvalidVertexID;
  maxp = ( !ValidVertex( obj, start) ) ? FirstVertex( obj) : start;
  maxv = SupportValue( obj, maxp, direction);

  while ( p != maxp ) {

    p = maxp;
    /* Check each neighbour of the current point. */
    for ( index = FirstEdge( obj, p) ;
	  ValidEdge( obj, index) ;
	  IncrementEdge( obj, index) ) {

      neighbour = VertexOfEdge( obj, index);
      
      /* Check that we haven't already visited this one in the
	 last outer iteration.  This is to avoid us calculating
	 the dot-product with vertices we've already looked at.
      */
      if ( neighbour==lastvisited )
	continue;
      thisv = SupportValue( obj, neighbour, direction);
      if ( thisv>maxv ) {
	maxv = thisv;
	maxp = neighbour;
#ifdef EAGER_HILL_CLIMBING
	/* The next line gives Gilbert & Ong's eager behaviour. */
	break;
#endif
      }
    }
      lastvisited = p;
  }

  *supportval = maxv;

  return p;
}


/* Computes the coordinates of a simplex point.
   Takes an array into which the stuff the result, the number of vertices
   that make up a simplex point, one of the point arrays, the indices of
   which of the points are used in the for the simplex points, and an
   array of the lambda values.
   */
static void compute_point( REAL pt[DIM], int len, REAL (* vertices)[DIM],
                           REAL *lambdas) {
   int d, i;

   overd( d) {
      pt[d] = 0;
      for ( i=0 ; i<len ; i++ )
         pt[d] += DO_MULTIPLY( vertices[i][d], lambdas[i]);
   }

   return;
}

static void add_simplex_vertex( struct simplex_point * s, int pos,
  Object obj1, VertexID v1, Transform t1, Object obj2, VertexID v2,
				Transform t2) {

  ApplyTransform( t1, obj1, v1, s->coords1[pos]);

  ApplyTransform( t2, obj2, v2, s->coords2[pos]);

  return;
}

#ifdef DUMP_CONST_TABLES
#define dump1d( array) \
  printf( "static const int " #array "[%d] = {\n\t%2d", \
	  TWICE_TWO_TO_DIM, array[0]); \
  for ( s=1 ; s<TWICE_TWO_TO_DIM ; s++ ) printf( ", %2d", array[s]); \
  printf( "};\n")

#define dump2d( array) \
  printf( "static const int " #array "[%d][%d] = {\n", \
	  TWICE_TWO_TO_DIM, DIM_PLUS_ONE); \
  for ( s=0 ; s<TWICE_TWO_TO_DIM ; s++ ) { printf( "\t{ %2d", array[s][0]); \
  for ( d=1 ; d<DIM_PLUS_ONE ; d++ ) printf( ", %2d", array[s][d]); \
  printf( (s<TWICE_TWO_TO_DIM-1) ? "},\n" : "} };\n"); }

#endif /* DUMP_CONST_TABLES */

#ifdef CONSTRUCT_TABLES
/* initialise_simplex_distance is called just once per activation of this
   code, to set up some internal tables.  It takes around 5000 integer
   instructions for DIM==3.
   */
static int simplex_distance_initialised = 0;

static void initialise_simplex_distance( void) {
   int power, d, s, e, two_to_e, next_elt, next_non_elt, pr;
   int num_succ[TWICE_TWO_TO_DIM];

   if ( simplex_distance_initialised )
      return;

   /* check that TWO_TO_DIM is two to the power of DIM */
   power = 1;
   for ( d=0 ; d<DIM ; d++ )
      power *= 2;

   assert( power == TWO_TO_DIM );

   /* initialise the number of successors array */
   for ( s=0 ; s<TWICE_TWO_TO_DIM ; s++ )
        num_succ[s] = 0;

   /* Now the main bit of work.  Simply stated, we wish to encode
      within the matrices listed below information about each possible
      subset of DIM_PLUS_ONE integers e in the range
      0 <= e < DIM_PLUS_ONE.  There are TWICE_TWO_TO_DIM such subsets,
      including the trivial empty subset.  We wish to ensure that the
      subsets are ordered such that all the proper subsets of subset
      indexed s themselves have indexes less than s.  The easiest way
      to do this is to take the natural mapping from integers to
      subsets, namely integer s corresponds to the subset that contains
      element e if and only if there is a one in the e'th position in
      the binary expansion of s.
      
      The arrays set are as follows:
      *  card( s) tells how many elements are in the subset s.
      *  max_elt( s) gives the maximum index of all the elements in
         subset s.
      *  elts( s, i) for 0 <= i < card( s) lists the indices of the
         elements in subset s.
      *  non_elts( s, i) for 0 <= i < DIM_PLUS_ONE-card( s) lists the
         indices of the elements that are not in subset s.
      *  pred( s, i) for 0 <= i < card( s) lists the indices of the
         subsets that are subsets of subset s, but with one fewer
         element, namely the element with index elts( s, i).
      *  succ( s, i) for 0 <= i < DIM_PLUS_ONE-card( s) lists the
         indices of the supersets of subset s that have one extra
         element, namely the element with index non_elts( s, i).

      The elements indexed in each elts( s,) and non_elts( s,) are
      listed in order of increasing index. 
     */  

   /* now for every non-empty subset s (indexed for
      0 < s < TWICE_TWO_TO_DIM ) set the elements of card( s), 
      max_elt( s), elts( s,), pred( s,), and succ( s,).
      */

   for ( s=1 ; s<TWICE_TWO_TO_DIM ; s++ ) {
      /* Now consider every possible element.  Element e is
         in subset s if and only if s DIV 2^e is odd. */
      two_to_e = 1;
      next_elt = next_non_elt = 0;
      
      for ( e=0 ; e<DIM_PLUS_ONE ; e++ ) {
	if ( (s/two_to_e) % 2 == 1 ) {
            /* so e belongs to subset s */
            elts( s, next_elt) = e;
            pr = s - two_to_e;
            pred( s, next_elt) = pr;
            succ( pr, num_succ[pr]) = s;
            num_succ[ pr]++;
            next_elt++;
	}
        else
            non_elts( s, next_non_elt++) = e;
            
         two_to_e *= 2;
      }
      card( s) = next_elt;
      max_elt( s) = elts( s, next_elt-1 );
   }

   /* for completeness, add the entries for s=0 as well */
   card( 0) = 0;   max_elt( 0) = -1;
   for ( e=0 ; e<DIM_PLUS_ONE ; e++ )
      non_elts( 0, e) = e;
   

   simplex_distance_initialised = 1;

   /* defining USE_CONST_TABLES to be the dimension of space
      causes this routine to be called and to dump the contents
      of the constant integer arrays to standard output.
   */
#ifdef DUMP_CONST_TABLES
   /* Code to dump the arrays to standard output */
     printf( "/* Include following code in gjk.c */\n\n");
     printf( "#define PRE_DEFINED_TABLE_DIM %d\n", DIM);
     dump1d( cardinality);
     dump1d( max_element);
     dump2d( elements);
     dump2d( non_elements);
     dump2d( predecessor);
     dump2d( successor);
     printf( "\n/* End of code to include -"
	     "Exiting gjk program immediately */\n");
     exit( 0);
#endif /* DUMP_CONST_TABLES */

   return;
}
#endif /* CONSTRUCT_TABLES */


//MT: from gjkdemo.c:

int
      gjk_num_g_test,     /* how many times the G-test is performed -- the
                             same as the number of main-loop iterations */
      gjk_num_simplices,  /* how many times the simplex routine
                             was called */
      gjk_num_backups,    /* how many times (if ever!) the GJK backup
                             procedure was called */
      gjk_num_dot_products, /* how many dot-product operations are called */
      gjk_num_support_dp, /* how many dot-product operations are called
			      whilst executing the support function */
      gjk_num_other_ops; /* how many other mults and divides are called */

void apply_trans(  Transform t, REAL * src, REAL * tgt)
{
  int i;

  if ( t==0 )
    for ( i=0 ; i<DIM ; i++ )
      tgt[i] = src[i];
  else {
    for ( i=0 ; i<DIM ; i++ )
      tgt[i] = t[i][DIM] + OTHER_DOT_PRODUCT( t[i], src);
  }
  return;
}


void
apply_rot_transpose( Transform t, REAL * src, REAL * tgt)
{
  int i;

  if ( t==0 )
    for ( i=0 ; i<DIM ; i++ )
      tgt[i] = src[i];
  else {
    for ( i=0 ; i<DIM ; i++ )
      tgt[i] = DO_MULTIPLY( t[0][i], src[0]) + DO_MULTIPLY( t[1][i], src[1])
	             + DO_MULTIPLY( t[2][i], src[2]);
  }
  return;
}
