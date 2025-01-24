Include: <../../../../rai-robotModels/ranger/ranger.g>

ranger_coll(ranger_base_link) { Q:[0, 0, -.14], shape: ssBox, size: [.75, .5, .3, .02], color:[1 0 0 .2], contact: 1 }

        
goal: { X:[2.5 0 0 1 0 0 1], shape: ssBox, size: [.75, .5, .1, .02], color:[0 1 0 .5] }
world: {}        
wall1(world): { X:[0  3 .2], shape: ssBox, size: [6.1, .2, .4, .02], color:[.3], contact: 1 }
wall2(world): { X:[0 -3 .2], shape: ssBox, size: [6.1, .2, .4, .02], color:[.3], contact: 1 }
wall3(world): { X:[ 3 0 .2], shape: ssBox, size: [.2, 6.1, .4, .02], color:[.3], contact: 1 }
wall4(world): { X:[-3 0 .2], shape: ssBox, size: [.2, 6.1, .4, .02], color:[.3], contact: 1 }
        
wall5(world): { X:[1 .5 .2], shape: ssBox, size: [.2, 3, .4, .02], color:[.3], contact: 1 }
wall5(world): { X:[0 -1 .2], shape: ssBox, size: [2, .2, .4, .02], color:[.3], contact: 1 }
wall6(world): { X:[-1 1 .2], shape: ssBox, size: [.2, 4., .4, .02], color:[.3], contact: 1 }
wall7(world): { X:[-2 0 .2], shape: ssBox, size: [.2, 4., .4, .02], color:[.3], contact: 1 }
wall8(world): { X:[0 -2 .2], shape: ssBox, size: [4., .2, .4, .02], color:[.3], contact: 1 }
wall9(world): { X:[2 .5 .2], shape: ssBox, size: [.2, 5., .4, .02], color:[.3], contact: 1 }

        
