
#include "qhull_a.h"

char qh_version[] = "sac 96/8/19";


#define DIM 3     /* dimension of points, must be 3! */
#define MAXpoints 1000	/* not critical */

static void new_facet( void);
static void add_vertex_to_facet( int id);
static void add_potential_half_edge( int a, int b);
static void process_facet( void);
static int construct_hull( double (*points)[3], int * num_rings, int rings[]);



int sac_qhull( int npts, const double (*input_array)[DIM],
	       double (*points)[DIM], int * size_rings, int rings[],
	       int * num_faces, double (*faces)[DIM+1])
{
  boolT ismalloc;
  int curlong, totlong, exitcode;
  char options [200];
  int v, d, numv, numr, numf;
  coordT array[MAXpoints][DIM];

  facetT *facet;
  vertexT *vertex;
  vertexT **vertexp;

  numf = 0;


  if ( npts > MAXpoints )
    {
      fprintf( stderr, "sac_qhull asked to compute hull of %d points,"
	       " but is only compiled to accept a maximum of %d.\n"
	       " To proceed, re-compile with a larger value of MAXpoints.\n",
	       npts, MAXpoints);
      return 0;
    }

  /* copy the points given */
  for ( v=0 ; v<npts ; v++ )
    for ( d=0 ; d<DIM ; d++ )
      array[v][d] = input_array[v][d];
  
  ismalloc= False; 	/* True if qh_freeqhull should 'free(array)' */

  qh_init_A (stdin, stdout, stderr, 0, nullptr);
  exitcode= setjmp (qh errexit);
  if (exitcode) {
	    return exitcode;
	    }

  strcat (qh rbox_command, "user_eg cube");
  sprintf (options, "qhull s Tcv C-0");
  qh_initflags (options);
  qh_init_B (array[0], npts, DIM, ismalloc);
  qh_qhull();
  qh_check_output();


  FORALLfacets
    {
      if ( faces!=0 )
	{
	  for ( d=0 ; d<3 ; d++ )
	    faces[numf][d] = facet->normal[d];
	  faces[numf++][3] = facet->offset;
	}
	  
	new_facet(); 

	FOREACHvertex_( facet->vertices)
	  {
		add_vertex_to_facet( vertex->id);
	  }
	process_facet();
    }

  numv = construct_hull( points, size_rings, rings);
  if ( num_faces!=0 )
    *num_faces = numf;

  qh NOerrexit= True;
  qh_freeqhull (!qh_ALL);
  qh_memfreeshort (&curlong, &totlong);

  return numv;
} /* sac_qhull */

struct half_edge_entry {
	  int this_id, other_id;
	  int confirmed;
	  struct half_edge_entry * next;
	  };

/* The table structure used to store information about half-edges
 * as they are discovered.  The entries are indexed by the vertex of
 * smallest ID, and held in unsorted linked-lists for each first vertex.
 */
static struct half_edge_entry * vertex_table[MAXpoints];
/* For efficiency we maintain a linked list of free half-edge structure
 * nodes, and refer to this first when allocating nodes.
 */
static struct half_edge_entry * half_edge_free_list = 0;
/* For each facet we need to store the list of its vertices (IDs) */
static int temp_vertex_list[MAXpoints], temp_vertex_list_size = 0;
/* vertex_map[ID] says where to find vertex with given ID in the point
 * list that is output.  That is, we don't assume that qhull ID's form
 * a contiguous set, although they probably do!
 */
static int vertex_map[MAXpoints];
/* static to say whether the tables have been initialised yet */
static int sac_tables_initialised = 0;
/* counters for the number of features found */
static int max_vertex_id = 0, num_half_edges = 0, num_faces = 0;

/* Should be called once for each facet, before calling add_vertex_to_facet */
static void new_facet( void)
{
  int i;

  if ( !sac_tables_initialised )
    {
      for ( i=0 ; i<MAXpoints ; i++ )
	{
	  vertex_map[i] = 0;
	  vertex_table[i] = 0;
	}
      max_vertex_id = 0;
      num_half_edges = num_faces = 0;
      sac_tables_initialised = 1;
    }

  temp_vertex_list_size = 0;

  return;
}

static void add_vertex_to_facet( int id)
{
  if ( id<0 || id>=MAXpoints )
    {
      fprintf( stderr, "Bad point index generated (%d)\n", id);
      exit( 1);
    }

  num_half_edges++;
  temp_vertex_list[temp_vertex_list_size++] = id;
  vertex_map[id] = 1;
  if ( id>max_vertex_id )
    max_vertex_id = id;

  return;
}

static void process_facet( void)
{
  int i, j;

  for ( i=0 ; i<temp_vertex_list_size-1 ; i++ )
    for ( j=i+1 ; j<temp_vertex_list_size ; j++ )
      add_potential_half_edge( temp_vertex_list[i], temp_vertex_list[j]);

  num_faces++;

  return;
}

static void add_potential_half_edge( int a, int b)
{
  struct half_edge_entry * hep, * new_he;

  if ( a>b )
    {
      add_potential_half_edge( b, a);
      return;
    }

  /* look for an entry for this potential half edge */
  hep = vertex_table[a];

  while ( hep!=0 )
    {
      if ( hep->other_id==b )
	/* then this edge has been seen before */
	{
	  /* check whether it's already been seen twice -- error */
	  if ( hep->confirmed )
	    {
	      fprintf( stderr, "Bad edge generated, id's %d and %d\n",
		       hep->this_id, hep->other_id);
	      exit( 1);
	    }
	  /* otherwise, confirm the entry */
	  hep->confirmed = 1;
	  return;
	}
      hep = hep->next;
    }

  /* if we get here, there was no existing entry */
  if ( half_edge_free_list != 0 )
    {
      new_he = half_edge_free_list;
      half_edge_free_list = half_edge_free_list->next;
    }
  else
    {
      new_he = (struct half_edge_entry *)
	malloc( sizeof(struct half_edge_entry));
      if ( new_he==0 )
	{
	  fprintf( stderr, "Malloc failed on half-edge entry\n");
	  exit( 1);
	}
    }

  new_he->this_id = a;
  new_he->other_id = b;
  new_he->confirmed = 0;
  new_he->next = vertex_table[a];
  vertex_table[a] = new_he;

  return;
}

static int construct_hull( double (*points)[3], int * num_rings, int rings[])
{
  int nexte, d, v, num_vertices, num_actual_half_edges, otherv;
  vertexT *vertex;
  struct half_edge_entry * hep, * free_he, * next_he;

  /* reset the 'tables initialised' flag */
  sac_tables_initialised = 0;

  /* count number of vertices, and set up the vertex map */
  num_vertices = 0;
  for ( v=0 ; v<=max_vertex_id ; v++ )
    {
      if ( vertex_map[v] )
	  vertex_map[v] = num_vertices++;
    }

  /* check the Euler characteristic: 2V - H + 2F = 4, H = half-edges */
  if ( 2*num_vertices - num_half_edges + 2*num_faces != 4 )
    {
      fprintf( stderr, "Bad Euler characteristic for hull "
	       "(%d vertices, %d half-edges, %d faces\n",
	       num_vertices, num_half_edges, num_faces);
      exit( 1);
    }

  /* now fill in the vertex coordinates entries */
  if ( points!=0 )
    {
      FORALLvertices
	{
	  for ( d=0 ; d<3 ; d++ )
	    points[vertex_map[vertex->id]][d] = vertex->point[d];
	}
    }
    
  nexte = num_vertices;
  num_actual_half_edges = 0;
  /* now process the half-edges */
  for ( v=0 ; v<=max_vertex_id ; v++ )
    {
      hep = vertex_table[v];
      if ( hep==0 )
	continue;

      if ( rings!= 0 )
	rings[vertex_map[v]] = nexte;
      while ( hep!=0 )
	{
	  next_he = hep->next;

	  /* There are 3 things that can happen to each node.  If it is
	   * unconfirmed, we dispose of it.  If it confirmed, and it's the
	   * first time that it's been seen (v==this_id), we move it to
	   * the list for other_id.  Otherwise, we dispose of it.
	   */
	  if ( hep->confirmed )
	    {
	      num_actual_half_edges++;
	      if ( hep->this_id==v )
		{
		  otherv = hep->other_id;
		  hep->next = vertex_table[otherv];
		  vertex_table[otherv] = hep;
		  free_he = 0;
		}
	      else
		{
		  otherv = hep->this_id;
		  free_he = hep;
		}
	      if ( num_actual_half_edges <= num_half_edges )
		{
		  if ( rings!=0 )
		    rings[nexte] = vertex_map[otherv];
		  nexte++;
		}
	    }
	  else /* ! hep->confirmed */
	    free_he = hep;

	  hep = next_he;
	  /* To dispose, add the node to the free list of nodes */
	  if ( free_he!=0 )
	    {
	      free_he->next = half_edge_free_list;
	      half_edge_free_list = free_he;
	    }
	}
      /* terminate the ring entries for the current vertex */
      if ( rings!=0 )
	rings[nexte] = -1;
      nexte++;
    }

  if ( num_actual_half_edges != num_half_edges )
    {
      fprintf( stderr, "Inconsistent half-edges in hull (%d vertices, "
	       "%d expected half-edges, %d half-edges found, %d faces\n",
	       num_vertices, num_half_edges, num_actual_half_edges,
	       num_faces);
      exit( 1);
    }

  
  if ( num_rings!=0 )
    * num_rings = nexte;
  return num_vertices;
}
