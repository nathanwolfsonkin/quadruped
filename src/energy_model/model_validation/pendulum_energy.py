import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.integrate import cumtrapz

# params
g = 9.81
l = 2
m = 3
I_c = (1/12) * m * l**2
theta0 = np.pi/4
dtheta0 = 0
t0 = 0
tf = 5.9
t_span = (t0, tf)
x0 = np.array([theta0, dtheta0])
n = 1000

def input(t):
    if t >= 1.473 and t <= 4.439:
        # u = 2
        u = 0
    else:
        u = 0
    
    return u

# Simple pendulum example
def f(t, x):
    u = input(t)
    Ax = np.array([x[1], -(m*g*l/(I_c + m*l**2))*np.sin(x[0])])

    Bu = np.array([0, 1/(I_c + m*l**2)]) * np.array([u])
    return Ax+Bu


solution = solve_ivp(f, t_span, x0, method='RK45', t_eval=np.linspace(t0, tf, n), max_step=0.01)

theta = solution.y[0]
dtheta = solution.y[1]


############################## NUMERICAL POWER ################################
total_energy = []
for i, time in enumerate(solution.t):
    u = m * g * (-l*np.cos(theta[i]))
    k_t = .5 * m * (l/2)**2 * dtheta[i]**2


############################### ANALYTICAL POWER ###############################
analytical_power = []
for i, time in enumerate(solution.t):
    state = np.array([theta[i], 
                      dtheta[i]])
    ddtheta = f(time, state)
    inst_power = m*g*l*np.sin(theta[i])*dtheta[i] + (I_c + m*l**2)*dtheta[i]*ddtheta
    analytical_power.append(inst_power)

##############################   PLOTTING   #######################################

fig = plt.figure()
ax = fig.add_subplot(111)

# ax.plot(solution.t, power_a, label='Power', lw=2)
ax.plot(solution.t, analytical_power, label='Analytical', lw=2)

ax.tick_params(axis='both', which='major', width=2.5)
ax.set_xlabel('Time (s)', fontsize=14)
ax.set_ylabel('Power (W)', fontsize=14)
ax.legend(fontsize=10)

plt.show()