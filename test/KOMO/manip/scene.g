Include: <../../../../rai-robotModels/scenarios/pandaSingle.g>

box(table): { joint: rigid, Q:[-.0,.25,.08], shape: ssBox, size: [.15,.06,.06,.005], contact: 1, mass: .1 }

obstacle(table): { Q:[-.15,.25,.08], shape:ssBox, size: [.06,.15,.06,.005], color:[.1], contact: 1 }

Edit panda_collCameraWrist: { shape: marker, contact: 0 }
