
body obj1 { type=ST_mesh mesh='meshTools/m494_x.off' swiftfile='meshTools/m494_x.dcp' contact }

#body obj2 { X=<T t(0 .7 2) d(90 1 0 0)>  type=ST_mesh mesh='meshTools/m494_x.off' swiftfile='meshTools/m494_x.dcp' contact }
body obj2 { X=<T t(0 .7 2) d(90 1 0 0)>  type=ST_ssBox mesh='meshTools/m494_x.off' }

body cyl1  { X=<T t(-1 0 0)>  type=ST_capsule size=[.1 .1 1. .2] contact }

body ball  { X=<T t(.2 .2 .7)>  type=ST_sphere size=[.1 .1 1. .02] color=[0 .5 0] contact }

