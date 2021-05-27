
world {}

table { X:[0 0 .1] shape:ssBox size:[2 2 .1 .02] color:[.3 .3 .3] }

obj { X:[0 0 2] shape:marker size:[.3] }

part1 (obj) { Q:<t(0 -.5 0) d(90 0 1 0)> shape:capsule size:[.5 .05] color:[.9 .9 .9] mass:1 }
part2 (obj) { Q:<t(0 .5 0)> shape:capsule size:[1. .05] color:[.9 .9 .9] }
part3 (obj) { Q:<t(0 .25 0) d(90 1 0 0)> shape:capsule size:[1.5 .05] color:[.9 .9 .9] }
part4 (obj) { Q:<t(.3 1. 0) d(90 1 0 0)> shape:ssBox size:[.5 .5 .5 .05] color:[.9 .9 .9] mass:1 }
