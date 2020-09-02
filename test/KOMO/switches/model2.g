world {}

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[1. 1. .1 .02], color:[.3 .3 .3]
}

box (table){
  shape:ssBox, Q:<t(.3 .1 .08) d(120 0 0 1)>, size:[.2 .06 .06 .02], color:[.4 .8 .4]
  joint:rigid
}


Include: '../../../../rai-robotModels/panda/panda.g'
Edit panda_link0{ X:<t(-.3 0 .65)> }

gripper (panda_joint7){
    shape:marker, size:[.03], color:[.9 .9 .5],
    Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155) t(0 0 -.05)>  
}
