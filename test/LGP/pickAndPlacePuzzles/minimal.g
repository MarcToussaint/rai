world {}

floor (world){ joint:rigid, shape:ssBox, Q:<t(0 -2.25 .05)>, size:[4.5 4.5 .1 .02], color:[.5 .5 .5], logical:{table} }
goal (world){ joint:rigid, shape:ssBox, Q:<t(0 2.25 .05)>, size:[4.5 4.5 .09 .02], color:[1.0 .0 .0], logical:{table} }

obj(world) { type:ssBox size:[.1 .1 .2 .02] Q:<t(0 -1.5 .5)> color:[0. 0. 1.],  logical={ object }, joint:rigid }


ego (world) {joint:trans3, shape:ssBox, Q:<t(-1 -1.5 .7)> size:[0.3 .3 0.3 0.01], color:[1.0 1.0 .5] , logical:{gripper} 
limits: [ -6 6 -6 6  -6 6 -100 100 -100 100 -100 100 -100 100  ]
}


