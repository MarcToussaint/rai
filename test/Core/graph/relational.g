Action cruiseto{ args=1 }
Symbol incity{ args=1 }
Symbol gas{ args=0 }
Symbol inhand{ args=1 }
Symbol size{ args=2 }
Symbol on{ args=2 }
Symbol upright{ args=1 }

Variable X
Variable Y
Variable Z

Rule default(){
     p = [.3, .7],
     context = {}
     outcome = {}
     outcome noise
}

Rule cruiseto(X){
     p=[.7, .2, .1]
     context = { gas() incity(Y), not incity(X) }
     outcome = { incity(X), not incity(Y) }
     outcome = {}
     outcome noise
}

Const 22

InitialState {
      incity(22)
      gas()=3
}




