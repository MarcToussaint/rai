
incity
gas
noise

20
21
22

Literals{
 X
 (incity, X)  
 (incity, 22) =false
 (gas) = 3
 (gas) = 0
}


STATE {
 (incity, 22) =false
 (gas) = 3
}

Rule default{
  precond{}
  out1{ } 
  out2{ (noise) }
  probs = [.3 .7]
}

Rule cruiseto{
  X
  Y
  precond{ (gas)=1 (incity Y)=! (incity X) }
  # Outcome 1:  action succeeded: different city
  out1{ (incity Y) (incity X)=! } 
  # Outcome 2:  action failed: still in same city
  out2{ }
  # Outcome 3: noise outcome
  out3{ (noise) }

  probs = [0.7 0.2 0.1]
}

Rules{ (default) (cruiseto) }


