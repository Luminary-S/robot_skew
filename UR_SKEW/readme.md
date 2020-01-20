# formulation explanation

$
\dot{ _{}^B\bold{X}^e_r }=
\dot{ _{}^B\bold{X}^e_d } -
\bold{K_c}* (  _{}^B\bold{X}^e_d - {}_{}^B\bold{X}^e_c  )
$
<!-- $$ \sideset{^1_2}{^3_4}\bigotimes $$ -->

$\dot{ _{}^B\bold{X}^e_r }={}_{W}^B\bold{Adj(q)} \otimes \dot{ _{}^W\bold{X}^e_r }$

$
_{}^B\bold{X}^e_d ={}_{W}^B\bold{T(x_b)} \otimes  {}_{}^W\bold{X}^e_d
$

$
{}_{}^B\bold{X}^e_c =
$

## note

1. IMU : Xsens MTI-30  

## IMU filter algorithm

1. ZUPT(zero-velocity update) ZAR : foot undergo a repeating sequence of stop and move;
2. complementary filter
3. HDE(Heuristic Drift Elimination) : to remove  bias  and  errors  from MEMS  gyroscopes, dominant direction.
