Include = '../../../../rai-robotModels/pr2/pr2.g'

Include = '../../../../rai-robotModels/pr2/pr2_modify_fixHead.g'
Include = '../../../../rai-robotModels/pr2/pr2_modify_fixGrippers.g'
Include = '../../../../rai-robotModels/pr2/pr2_modify_fixLeft.g'

Include = '../../../../rai-robotModels/objects/shelf.g'

Edit shelf { X:<d(90 0 0 1) t(1. 1. 0) d(120 0 0 1)> }

shape endeff(r_wrist_roll_link){ rel=<T t(.2 0 0)> type=5 color=[1 0 0] size=[.1 0 0 0]}
