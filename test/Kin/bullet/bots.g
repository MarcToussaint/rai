Prefix: "a_"
Include: <../../../../rai-robotModels/scenarios/panda_fixRobotiq.g>

Prefix: "b_"
Include: <../../../../rai-robotModels/scenarios/panda_fixGripper.g>

Prefix: "c_"
Include: <../../../../rai-robotModels/scenarios/panda_noGripper.g>

Prefix!

Edit a_panda_base { X:[1 0 1.1] }
Edit b_panda_base { X:[0 0 1.1] }
Edit c_panda_base { X:[-1 0 1.1] }

Include: <../../../../rai-robotModels/scenarios/pandasTable.g>

box1 { shape:ssBox, size:[.3 .3 .3 .05], X:[1.3 0 2.5], mass=3 }

box2 { shape:ssBox, size:[.3 .3 .3 .05], X:[.5 .2 2.8], mass=3 }
