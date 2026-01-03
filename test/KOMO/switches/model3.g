Include: <../../../../rai-robotModels/scenarios/pandasTable.g>

box1 (table){
  shape:ssBox, Q: "t(.4 .1 .08) d(120 0 0 1)", size:[.2 .06 .06 .02], color:[.4 .8 .4], 
  joint:rigid
}

box2 (table){
  shape:ssBox, Q: "t(.7 .1 .08) d(-90 0 0 1)", size:[.2 .06 .06 .02], color:[.8 .4 .4], 
  joint:rigid
}

tray (table){ shape:ssBox, Q:[-.6 .1 .07], size: [.4 .4 .04 .019], color: [.7] }
