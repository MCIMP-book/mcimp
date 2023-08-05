% kalman_decompose/uncontrollablesys.m
b = 1; m = 1;
k1 = 0.5; k2 = 1;
A = [-b/m, -1/m, -1/m; k1, 0, 0; k2, 0, 0];
B = [1/m; 0; 0];
P = [B, A*B, A^2*B]
disp(['The rank of P is: ', num2str(rank(P)), '. Hence the system is not controllable.']);
disp('1. Construct M manually.');
Mc = [1 -1; 0, k1; 0, k2];
Muc = [0;0;1];
M = [Mc Muc]
invM = inv(M)
tildeA = invM*A*M
tildeB = invM*B
disp('2. Construct M based on orthonormal decomposition.');
Mc = orth(P)
Muc = null(P')
M = [Mc Muc]
rank(M)
tildeA = M\A*M % this computes inv(M)*A*M
tildeB = M\B % this computes inv(M)*B
