
obj1 { shape:mesh mesh='meshTools/m494_x.off' swiftfile='meshTools/m494_x.dcp' contact }

#obj1 { shape:ssBox size=[.2 .2 1. .05] contact }

obj2 { X: "t(0 .7 2) d(90 1 0 0)"  shape:mesh mesh='meshTools/m494_x.off' swiftfile='meshTools/m494_x.dcp' contact }
#obj2 { X: "t(0 .7 2) d(90 1 0 0)"  shape:ssBox mesh='meshTools/m494_x.off' size=[.5 .5 .5 .1] }

cyl1  { X: "t(-1 .5 0)"  shape:capsule size=[.1 .1 1. .2] contact }

ball  { X: "t(.2 .2 .7)"  shape:sphere size=[.1 .1 1. .02] color=[0 .5 0] contact }

