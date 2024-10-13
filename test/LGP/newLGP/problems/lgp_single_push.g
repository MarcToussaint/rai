world {}                         

table_base (world) { Q:[0 0 .6]  shape:marker, size:[.05]  }

table(table_base): {
 shape: ssBox, Q:[0, 0, -.05], size: [2., 1.5, .1, .02], color: [.3, .3, .3],
 contact: 1, logical: { is_box, is_place },
 friction: .1
}

Prefix: "l_"
Include: <rai-robotModels/panda/panda.g>
Prefix: False

Edit l_panda_base (table): { Q: "t(0 -.2 .05) d(90 0 0 1)" }

trayB (table_base) 	{  Q:[.3, .3, -.005], shape:ssBox, size:[0.07, 0.07, 0.01, 0.005], color:[0, 0, .6], logical:{ is_box, is_place } }

#objR (table) 	{  Q:[-.4, .1, .1], joint:rigid, shape:ssBox, size:[0.06, 0.06, .1, .005], color:[.7, 0, 0], contact:1, logical:{ is_object } }

#objG (table) 	{  Q:[-.2, .2, .1], joint:rigid, shape:ssBox, size:[0.06, 0.06, .1, .005], color:[0, .7, 0], contact:1, logical:{ is_object } }

objB (table) 	{  Q:[.3, 0, .1], joint:rigid, shape:ssCylinder, size:[.1, 0.04, .005], color:[0, 0, .7], contact:1, logical:{ is_object } }


