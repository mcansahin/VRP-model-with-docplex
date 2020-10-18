import numpy as np
import matplotlib.pyplot as plt
rnd = np.random
rnd.seed(0)
"""
n = node number
Q= vehicle capacity
q= demand
A = next probably nodes
c = cost

"""

n= 14
Q= 60
k=[1,2,3,4]
a={1:50,2:50,3:50,4:50}
N = [i for i in range (1,n+1)]
print (N)
V = [0] + N
L = [i for i in range (2,n+1)]
print (V)
q= {1:10,2:7,3:13,4:19,5:26,6:3,7:5,8:9,9:16,10:16,
    11:12,12:19,13:23,14:20}
print (q)

loc_x = [35,41,35,55,55,15,25,20,10,55,30,20,50,30,15]
loc_y = [35,49,17,45,20,30,30,50,43,60,60,65,35,25,10]


A= [(i,j) for i in V for j in V if i!=j]
c = {(i,j):np.hypot(loc_x[i]-loc_x[j],loc_y[i]-loc_y[j]) for i,j in A}

Y=[(i,j,f) for i,j in A for f in k]

from docplex.mp.model import Model



md1= Model('CVRP')
x = md1.binary_var_dict(Y,name='x')
u = md1.continuous_var_dict(N,ub=Q,name='u')



md1.minimize(md1.sum(c[i,j]*x[i,j,f] for i,j in A for f in k))


md1.add_constraints(md1.sum(x[i,j,f] for f in k for j in V if i!=j)==1 for i in N)


md1.add_indicator_constraints(md1.indicator_constraint(x[i,j,f],u[i]+q[j]==u[j]) for i,j,f in Y if i!=0 and j!=0)
md1.add_constraints(u[i]>=q[i] for i in N)


md1.add_constraints(md1.sum(x[0,j,f] for j in N)==1 for f in k)


md1.add_constraints(((md1.sum(x[i,p,f] for i in V if i!=p))- (md1.sum(x[p,j,f] for j in V if j!=p )))==0 for p in N for f in k)





solution = md1.solve(log_output=True)

print(solution)

active_arcs = [a for a in Y if x[a].solution_value> 0.9]

plt.scatter(loc_x[1:],loc_y[1:],c='b')
for i in N:
    plt.annotate('$q_%d=%d$' %(i,q[i]),(loc_x[i]+2,loc_y[i]))
for i,j,f in active_arcs:
    if f==1 :
        plt.plot([loc_x[i],loc_x[j]],[loc_y[i],loc_y[j]],c='g',alpha=0.3)
    if f==2 :
        plt.plot([loc_x[i],loc_x[j]],[loc_y[i],loc_y[j]],c='b',alpha=0.3)
    if f==3 :
        plt.plot([loc_x[i],loc_x[j]],[loc_y[i],loc_y[j]],c='r',alpha=0.3)
    if f==4 :
        plt.plot([loc_x[i],loc_x[j]],[loc_y[i],loc_y[j]],c='k',alpha=0.3)
    if f==5 :
        plt.plot([loc_x[i],loc_x[j]],[loc_y[i],loc_y[j]],c='c',alpha=0.3)
  
plt.plot(loc_x[0],loc_y[0],c='r',marker='s')
plt.axis('equal');

