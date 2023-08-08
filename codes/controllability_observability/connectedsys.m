% controllability_observability/connectedsys.m
A1 = [ 0 1; -2 -3 ]; B1 = [ 0; 1 ]; C1 = [ 2 1 ];
sys1 = ss(A1, B1, C1, 0)
P1 = ctrb(sys1)
rank(P1)
Q1 = obsv(sys1)
rank(Q1)

A2 = [ -1 0; 0 -3 ]; B2 = [ 1; 1 ]; C2 = [ 1 1 ];
sys2 = ss(A2, B2, C2, 0)
P2 = ctrb(sys2)
rank(P2)
Q2 = obsv(sys2)
rank(Q2)

sys_s = series(sys1,sys2)
sys_p = parallel(sys1,sys2)
Ps = ctrb(sys_s)
rank(Ps)
Qs = obsv(sys_s)
rank(Qs)

Pp = ctrb(sys_p)
rank(Pp)
Qp = obsv(sys_p)
rank(Qp)
