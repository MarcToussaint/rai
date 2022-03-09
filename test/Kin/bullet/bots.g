Prefix: "a_"
Include: '../../../../rai-robotModels/scenarios/panda_fixRobotiq.g'

Prefix: "b_"
Include: '../../../../rai-robotModels/scenarios/panda_fixGripper.g'

Prefix: "c_"
Include: '../../../../rai-robotModels/scenarios/panda_noGripper.g'

Prefix!

Edit a_panda_base { X:[1 0 1.1], motors}
Edit b_panda_base { X:[0 0 1.1], motors }
Edit c_panda_base { X:[-1 0 1.1], }

Include: '../../../../rai-robotModels/scenarios/pandasTable.g'

box { shape:ssBox, size:[.3 .3 .3 .05], X:[1.3 0 2.5], mass=3 }
