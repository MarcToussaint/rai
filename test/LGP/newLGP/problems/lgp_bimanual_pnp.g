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
Prefix: "r_"
Include: <rai-robotModels/panda/panda.g>
Prefix: False

Edit l_panda_base (table): { Q: "t(-.4 -.2 .05) d(90 0 0 1)" }
Edit r_panda_base (table): { Q: "t(+.4 -.2 .05) d(90 0 0 1)" }

trayR (table_base) 	{  Q:[.1, .4, 0], shape:ssBox, size:[0.15, 0.07, 0.01, 0.005], color:[.6, 0, 0], logical:{ is_box, is_place } }

trayG (table_base) 	{  Q:[.7, .1, 0], shape:ssBox, size:[0.07, 0.07, 0.01, 0.005], color:[0, .6, 0], logical:{ is_box, is_place } }

trayB (table_base) 	{  Q:[-.6, .3, 0], shape:ssBox, size:[0.07, 0.07, 0.01, 0.005], color:[0, 0, .6], logical:{ is_box, is_place } }
        
objR (table) 	{  Q:[-.8, .1, .1], joint:rigid, shape:ssBox, size:[0.06, 0.06, .1, .005], color:[.7, 0, 0], contact:1, logical:{ is_object } }

objG (table) 	{  Q:[-.6, .2, .1], joint:rigid, shape:ssBox, size:[0.06, 0.06, .1, .005], color:[0, .7, 0], contact:1, logical:{ is_object } }

objB (trayG) 	{  Q:[-.0, 0, .055], joint:rigid, shape:ssBox, size:[0.06, 0.06, .1, .005], color:[0, 0, .7], contact:1, logical:{ is_object } }


