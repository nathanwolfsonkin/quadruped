import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.integrate import cumtrapz

# params
g = 9.81
l = 2
m = 3
theta0 = np.pi/4
dtheta0 = 0
t0 = 0
tf = 5.9
t_span = (t0, tf)
x0 = np.array([theta0, dtheta0])
n = 1000

def input(t):
    if t >= 1.473 and t <= 4.439:
        u = 2
        # u = 0
    else:
        u = 0
    
    return u

# Simple pendulum example
def f(t, x):
    u = input(t)
    Ax = np.array([x[1], -(g/l)*np.sin(x[0])])
    Bu = np.array([0, 1/(m*l**2)]) * np.array([u])
    return Ax+Bu


solution = solve_ivp(f, t_span, x0, method='RK45', t_eval=np.linspace(t0, tf, n), max_step=0.01)

theta = solution.y[0]
dtheta = solution.y[1]


# Post process
power_g = []
power_a = []
ke = []
pe = []
total_energy = []
u = []
for index, time in enumerate(solution.t):
    u.append(input(time))
    power_g.append(-m * g * l * np.sin(theta[index]) * dtheta[index])
    power_a.append(u[index] * dtheta[index])
    ke_temp = .5 * (m * (l**2)) * (dtheta[index]**2)
    pe_temp = m * g * (-l * np.cos(theta[index]))
    ke.append(ke_temp)
    pe.append(pe_temp)
    total_energy.append(ke_temp + pe_temp)

energy_power_g = cumtrapz(power_g, solution.t)
energy_power_g = np.insert(energy_power_g, 0, 0)
energy_power_a = cumtrapz(power_a, solution.t)
energy_power_a = np.insert(energy_power_a, 0, 0)

abs_power_a = np.abs(power_a)
energy_abs_power_a = cumtrapz(abs_power_a, solution.t)
energy_abs_power_a = np.insert(energy_abs_power_a, 0, 0)

###################################################################################

# Fontsize
fs=14
# Linewidth
lw=3

fig = plt.figure()

ax = fig.add_subplot(211)
ax.plot(solution.t, power_a, label='Power due to Applied Torque', lw=lw)
ax.set_ylabel('Power (W)', fontsize=fs)
ax.set_xlabel('Time (s)', fontsize=fs)
ax.tick_params(axis='both', which='major', width=2.5)
ax.legend(fontsize=fs)


ax = fig.add_subplot(212)

ax.plot(solution.t, energy_power_a, label='Energy Change from Applied Torque', color='red', lw=lw)
# ax.plot(solution.t, energy_abs_power_a, label='Energy Drawn from Motor', color='red')
ax.set_ylabel('Energy (J)', fontsize=fs)  # Label for the second y-axis
ax.set_xlabel('Time (s)', fontsize=fs)  # Label for the x-axis
ax.tick_params(axis='both', which='major', width=2.5)
ax.legend(fontsize=fs)


#######################################################################################

fig = plt.figure()

ax = fig.add_subplot(211)

ax.plot(solution.t, abs_power_a, label='Absolute Value of Power due to Applied Torque', lw=lw)
ax.set_ylabel('Power (W)', fontsize=fs)
ax.set_xlabel('Time (s)', fontsize=fs)
ax.tick_params(axis='both', which='major', width=2.5)
ax.legend(loc=2, fontsize=fs)


ax = fig.add_subplot(212)

ax.plot(solution.t, energy_abs_power_a, label='Energy Drawn from Motor', color='red', lw=lw)
ax.set_ylabel('Energy (J)', fontsize=fs)  # Label for the second y-axis
ax.set_xlabel('Time (s)', fontsize=fs)  # Label for the x-axis
ax.tick_params(axis='both', which='major', width=2.5)
ax.legend(loc=2, fontsize=fs)

plt.show()