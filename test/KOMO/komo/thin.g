
wall { X:<t(0 0 1)>, shape:ssBox, size:[.02, 1., 2., .01] }

ball (wall) { X:<t(-2. .5 1)>, shape:sphere, size:[.1], joint:trans3 }

target { X:<t(+2. -.5 1)>, shape:sphere, size:[.1], color:[0 1 0 .4] }
