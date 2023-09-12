frame table1{ shape=ssBox, X: "t(.0 0 .7)", size=[1. 1. .1 .02], color=[.3 .3 .3 .7] contact }

frame world (table1){ Q = <t(0 0 .1)> }

### ball

frame ball1 (world){
    shape=ssBox, X: "t(.1 0 .8)", size=[.1 .1 .1 .05], color=[.9 .3 .3] contact
    joint=transXY,
    Q: "t(-.2 -.2 0)" }

frame ball2 (world){
    shape=ssBox, X: "t(.1 0 .8)", size=[.1 .1 .1 .05], color=[.3 .3 .9] contact, mass=1
    joint=rigid,
    Q: "t(.2 .2 0)" }

