% probability_review/sum_3rv.m
X1 = rand(1,1e5); X2 = rand(1,1e5); X3 = rand(1,1e5);
Z = X1 + X2 + X3;
[fz,x] = hist(Z,100);
w_fz = x(end)/length(fz);
fz = fz/sum(fz)/w_fz;
figure, bar(x,fz)
xlabel 'x'; ylabel 'p_Z(x)';
