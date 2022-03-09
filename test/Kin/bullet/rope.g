
box { shape:ssBox X:[0 0 1.] size:[.2 .2 .2 .05] }

rope (box){ joint:rigid, shape:mesh, mesh_rope:[.5 .5 -.5 10], mass=.1, soft }
