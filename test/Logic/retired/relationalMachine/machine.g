Symbol conv
Symbol timeout
Symbol contact
Symbol init
Symbol quit

Action CoreTasks
Action gazeAtHand
Action alignHand
Action positionHand
Action lowerHand
Action controlForce
Action homing

STATE { }

Rule { { (init) }
       { (init)! (alignHand) (positionHand) } }

Rule { { (positionHand conv) (alignHand conv) }
       { (lowerHand) (positionHand)! (positionHand conv)! } }

Rule { { (lowerHand conv) }
       { (controlForce) (lowerHand)! (lowerHand conv)! } }

Rule { { (controlForce timeout) }
       { (homing) (controlForce)! (controlForce timeout)! (gazeAtHand)! (gazeAtHand conv)! (alignHand)! (alignHand conv)! } }

Rule { { (homing conv) }
       { (quit) (homing)! (homing conv)! } }
