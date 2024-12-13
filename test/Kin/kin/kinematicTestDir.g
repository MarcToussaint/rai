base: { X: [0, 0, .5], shape:marker, size=[.5] }

dir(base): { Q:[0,0,.2], joint: direction, shape: sphere, size: [.1], color:[.8 .5] }

arm(base) { Q:[0,.2,0], shape:ssBox, size:[.05,.4, .05, .01] }
joint_origin(arm) { Q:[0,.2,0] }
joint(joint_origin) { joint: hingeX, limits:[-1, 1], sampleUniform: 1. }
hand(joint) { Q:[0,.2,0], shape: ssBox, size:[.05, .4, .05, .01] }

