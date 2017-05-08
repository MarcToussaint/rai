is
o1
o2

STATE { (is o1) (is o2) }

func Qfunction {
  tree1{
    leaf{ { aggregate{ X, { (is X) }, count=2 } }, r=1. }
    weight=1.
  }
  tree2{
    leaf{ X, { (is X) }, r=3. }
    weight=1.
  }
}
