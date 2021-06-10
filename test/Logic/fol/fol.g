M1
M2
Nono
America
West

American
Weapon
Sells
Hostile
Criminal
Missile
Owns
Enemy

STATE {
(Owns Nono M1),
(Missile M1),
(Missile M2),
(American West),
(Enemy Nono America)
}

Query { (Criminal West) }

Rule {
x, y, z,
{ (American x) (Weapon y) (Sells x y z) (Hostile z) }
{ (Criminal x) }
}


Rule {
x
{ (Missile x) (Owns Nono x) }
{ (Sells West x Nono) }
}


Rule {
x
{ (Missile x) }
{ (Weapon x) }
}

Rule {
x
{ (Enemy x America) }
{ (Hostile x) }
}


