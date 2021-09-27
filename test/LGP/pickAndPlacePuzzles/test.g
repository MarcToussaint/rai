world {}

floor_left (world){ shape:ssBox, Q:<t(0 1. .05)>, size:[4.1 3 .1 .02], color:[.5 .5 .5], logical:{table} }
floor_right (world){ shape:ssBox, Q:<t(0 -1.75 .05)>, size:[4.1 2.5 .1 .02], color:[.5 .5 .5], logical:{table} }

wall_right (world){ shape:ssBox, Q:<t(0 -3. 0.5)>, size:[4.1 .1 1. .02], color:[.3 .3 .3], contact:1 }
wall_back (world){ shape:ssBox, Q:<t(-2. 0 0.5)>, size:[.1 6.1 1. .02], color:[.3 .3 .3], contact:1 }
wall_left (world){ shape:ssBox, Q:<t(0 3. 0.5)>, size:[4.1 .1 1. .02], color:[.3 .3 .3], contact:1 }
wall_front (world){ shape:ssBox, Q:<t(2. 0 0.5)>, size:[.1 6.1 1. .02], color:[.3 .3 .3], contact:1 }
goal (world){ shape:ssBox, Q:<t(0 2.7 .05)>, size:[4.1 0.6 .1 .02], color:[1. .3 .3],logical:{table} }

wall1 (world){ shape:ssBox, Q:<t(-1.25 1.2 0.5)>, size:[1.4 .1 1 .02], color:[.3 .3 .3],  contact:1 }
wall2 (world){ shape:ssBox, Q:<t(1.25 1.2 0.5)>, size:[1.4 .1 1 .02], color:[.3 .3 .3],  contact:1 }

ego_base_origin (floor_left){ X:[0 0 .3] }
ego (ego_base_origin) {joint:transXY, shape:ssBox, Q:<t(-1.0 0 0)>, size:[0.3 .3 0.3 0.01], color:[1.0 1.0 .5] logical:{gripper:True, object:True},  contact:1 }

obj (floor_left) { joint:rigid, shape:ssBox, Q:<t(0.7 -1 0.2)>, size:[0.3 .3 0.3 0.01], color:[0.0 0.0 1.0],  contact:1, logical:{object:True} }


cube2(world) {
    shape:ssBox, Q:<t(0 1.1 0.21)>, size:[.7 .5 .2 .02],  color:[1.0 1.0 1.0],
    joint:rigid, friction:.1, contact:1, logical:{object:True}
}


