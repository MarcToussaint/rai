EQ
object
table
height
ground

## constants
o1
o2
o3

## initial state
STATE{
  (object o1)
  (object o2)
  (object o3)
  (ground o3)
  (height o1)=2
  (height o2)=2
}

### RULES


Rule {
     X, Y
     { (object X) (object Y) (height X)=2 (height Y)=2 aggregate{ Z, { (object Z) }, count=3 } }
     { (height X)=1 }
}


