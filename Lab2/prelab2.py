import sympy as sym
import math

a = [0, 0, 0, sym.Symbol('a3'), -sym.Symbol('a4'), 0, sym.Symbol('a6')]
alpha = [0, -math.pi/2, math.pi/2, math.pi/2, -math.pi/2, math.pi/2, math.pi/2]
d = [sym.Symbol('d1'), 0, 0, sym.Symbol('d3'), 0, sym.Symbol('d5'), sym.Symbol('d7')]
theta = [sym.Symbol('theta1'), sym.Symbol('theta2'), sym.Symbol('theta3'), sym.Symbol('theta4'), 0, sym.Symbol('theta6'), sym.Symbol('theta7')- math.pi/4]

T_arr = []
for i in range(7):
    c, s = sym.cos(theta[i]), sym.sin(theta[i])
    T = sym.Matrix([[c, -s*math.cos(alpha[i]), s*math.sin(alpha[i]), a[i]*c], [ s, c*math.cos(alpha[i]), -c*math.sin(alpha[i]), a[i]*s], [0, math.sin(alpha[i]), math.cos(alpha[i]), d[i]], [0, 0, 0, 1]])
    T_arr.append(T)

T_7_0 = T_arr[0]*T_arr[1]*T_arr[2]*T_arr[3]*T_arr[4]*T_arr[5]*T_arr[6]

#index is weirdly from 0 to 15 unline normal matrix indices
#print(T_7_0[15])
#checking if the tolerance value is actually fine or no
#print(sym.simplify(T_7_0[0]))
#print('------------------------------------------')
print(sym.simplify(sym.nsimplify(T_7_0[0],tolerance=1e-10,rational=True)))

