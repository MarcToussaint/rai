
## a trivial graph
x            # a 'vertex': key=x, value=true, parents=none
y            # another 'vertex': key=y, value=true, parents=none
(x y)        # an 'edge': key=none, value=true, parents=x y
x            # key=x, value=true, parents=none
y            # key=y, value=true, parents=none
(x y)        # key=none, value=true, parents=x y
(-1 -2)      # key=none, value=true, parents=the previous and the y-node

## a bit more verbose graph
node A{ color=blue }         # keys=node A, value=<Graph>, parents=none
node B{ color=red, value=5 } # keys=node B, value=<Graph>, parents=none
edge C(A,B){ width=2 }       # keys=edge C, value=<Graph>, parents=A B
hyperedge(A B C) = 5         # keys=hyperedge, value=5, parents=A B C

## standard value types
a=string      # MT::String (except for keywords 'true' and 'false' and 'Mod' and 'Include')
b="STRING"    # MT::String (does not require a '=')
c='file.txt'  # MT::FileToken (does not require a '=')
d=-0.1234     # double
e=[1 2 3 0.5] # MT::arr (does not require a '=')
#f=(c d e)    # DEPRECATED!! MT::Array<*Node> (list of other nodes in the Graph)
g!            # bool (default: true, !means false)
h=true        # bool
i=false       # bool
j={ a=0 }     # sub-Graph (special: does not require a '=')

## parsing: = {..} (..) , and \n are separators for parsing key-value-pairs
b0=false b1 b2, b3() b4   # 4 boolean nodes with keys 'b0', 'b1 b2', 'b3', 'b4'
k={ a, b=0.2 x="hallo"     # sub-Graph with 6 nodes
  y
  z()=filename.org x }

## special Node Keys

## editing: after reading all nodes, the Graph takes all Edit nodes, deletes the Edit tag, and calls a edit()
## this example will modify/append the respective attributes of k
Edit k { y=false, z=otherString, b=7, c=newAttribute }

## including
Include = 'example_include.g'   # first creates a normal FileToken node then opens and includes the file directly

## any types
#trans=<T t(10 0 0)>  # 'T' is the tag for an arbitrary type (here an ors::Transformation) which was registered somewhere in the code using the registry()
                      # (does not require a '=')

## strange notations
a()       # key=a, value=true, parents=none
()        # key=none, value=true, parents=none
[1 2 3 4] # key=none, value=MT::arr, parents=none
