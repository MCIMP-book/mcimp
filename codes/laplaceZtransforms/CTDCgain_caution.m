% laplaceZtransforms/CTDCgain_caution.m
H = tf([0 3],[1 -2])
dcgain(H)
figure, step(H)
