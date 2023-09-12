
## Syntactic keywords
QUIT
WAIT
Terminate
#Rule

## activities
pickingup
positioning
screwing
releasing
busy
inhandNil

## basic predicates
table
on
wall
screw
ground
object
material
humR
humL
rob
hand
used
inPosition
inhand
fixed

## constants
67
71
72
76
77
78

## initial state
START_STATE {
(screw 67)
(object 67)
(ground 71)
(material 71)
(object 71)
(wall 72)
(material 72)
(object 72)
(humR 76)
(hand 76)
(humL 77)
(hand 77)
(rob 78)
(hand 78)
(inhandNil 76)
(inhandNil 77)
(inhandNil 78)
}

### terminal state

terminal { (used 67) (inPosition 71) (inPosition 72) }
REWARD {}

### RULES

Rule activate_pickingup {
     X, Y
     { (pickingup X Y)! (hand X) (object Y) (inhandNil X) (busy X)! (busy Y)! }
     { (pickingup X Y)=2.1 (busy X) (busy Y) }
}

Term (Terminate pickingup) {
     X, Y
     { (inhand X Y) (inhandNil X)! (pickingup X Y)! (busy X)! }
}

Rule activate_positioning {
     X, Y
     { (positioning X Y)! (hand X) (object Y) (inhand X Y) (material Y) (inPosition Y)! (busy X)! }
     { (positioning X Y)=3.5 (busy X) }
}

Term (Terminate positioning) {
     X, Y
     { (inPosition Y) (positioning X Y)! (busy X)! }
}

Rule activate_releasing {
     X, Y
     { (releasing X Y)! (hand X) (object Y) (inhand X Y) (material Y) (busy X)! }
     { (releasing X Y)=1.0 (busy X) }
}

Term (Terminate releasing){
     X, Y
     { (releasing X Y)! (inhand X Y)! (inhandNil X) (busy X)! (busy Y)! }
}

Rule activate_screwing {
     X, Y, Z, U, V, W
     { (screwing X Y Z U V W)! (hand X) (screw Y) (inhand X Y) (wall Z) (ground W) (inPosition Z) (inPosition W) (hand U) (inhand U Z) (hand V) (inhand V W) (fixed Z W)! (busy X)! (busy U)! (busy V)!  (used Y)! }
     { (screwing X Y Z U V W)=8.0 (busy X) (busy U) (busy V) (inPosition Z) (inPosition W) }
}

Term (Terminate screwing) {
     X, Y, Z, U, V, W
     { (screwing X Y Z U V W)! (fixed Z W) (used Y) (inPosition Z) (inPosition W) (busy X)! (busy U)! (busy V)! (inhand X Y)! (inhandNil X) }
}
