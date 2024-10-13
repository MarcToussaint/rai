Include: <../../rai-robotModels/scenarios/pandasTable.g>

obj1(table): { joint: rigid, Q:[-.1,.3,.08], shape: ssBox, size: [.15,.06,.06,.005], contact: 1, mass: .1 }

obstacle(origin): { Q:[.1,.2,.15], shape:ssBox, size: [.06, .3, .3, .005], color:[.1], contact: 1 }

target (origin): { Q:[.4, .2, .0], shape:ssBox, size: [.1, .1, .02, .005], color:[.5, 1., .5], contact: 1}
