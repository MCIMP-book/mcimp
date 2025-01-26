# laplaceZtransforms/CTDCgain.py
import control as ct
a0, a1, a2 = 80, 81, 82
b0, b1, b2 = 60, 61, 62
A = np.array([[-a2, 1, 0],[-a1, 0, 1],[-a0, 0, 0]])
B = np.array([[b2],[b1],[b0]])
C = np.array([1, 0, 0])
D = np.array([1])
G = ct.ss(A,B,C,D)
print(G)
T = np.array([[0,0,1],[0,1,0],[1,0,0]])
Gt = ct.ct.similarity_transform(G,T)
print(Gt)
