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


#############################################################################
plt.figure()
ax1 = plt.gca()  # Get the current axes

# Create the second
ax1.plot(solution.t, energy_power_g, label='Amount of PE -> KE due to Gravity', color='red')
ax1.plot(solution.t, pe, label='Potential Energy', color='tomato')
ax1.plot(solution.t, ke, label='Kinetic Energy', color='darkviolet')
ax1.plot(solution.t, total_energy, label='Total Energy', color='darkolivegreen')
ax1.set_ylabel('Energy (J)', color='tab:red')  # Label for the second y-axis
ax1.set_xlabel('Time (s)')  # Label for the x-axis
ax1.tick_params(axis='y', labelcolor='tab:red')  # Color the ticks

ax2 = ax1.twinx()  # Create a second y-axis
ax2.plot(solution.t, power_g, label='Power due to Gravity')
ax2.set_ylabel('Power (W)', color='tab:blue')  # Label for the first y-axis
ax2.tick_params(axis='y', labelcolor='tab:blue')  # Color the ticks

ax1.legend(loc='upper left')
ax2.legend(loc='upper right')
# ax2.set_ylim(ax1.set_ylim())

#############################################################################

plt.figure()
ax1 = plt.gca()  # Get the current axes

# Create the second
ax1.plot(solution.t, energy_power_g, label='Amount of PE -> KE due to Gravity', color='red')
ax1.plot(solution.t, pe, label='Potential Energy', color='tomato')
ax1.plot(solution.t, ke, label='Kinetic Energy', color='darkviolet')
ax1.plot(solution.t, total_energy, label='Total Energy', color='darkolivegreen')
ax1.set_ylabel('Energy (J)', color='tab:red')  # Label for the second y-axis
ax1.set_xlabel('Time (s)')  # Label for the x-axis
ax1.tick_params(axis='y', labelcolor='tab:red')  # Color the ticks

ax2 = ax1.twinx()  # Create a second y-axis
ax2.plot(solution.t, power_a, label='Power due to Applied Torque')
ax2.set_ylabel('Power (W)', color='tab:blue')  # Label for the first y-axis
ax2.tick_params(axis='y', labelcolor='tab:blue')  # Color the ticks

ax1.legend(loc='upper left')
ax2.legend(loc='upper right')
# ax2.set_ylim(ax1.set_ylim())

###################################################################################
plt.figure()
plt.subplot(2,1,1)
ax1 = plt.gca()  # Get the current axes

# Create the second
ax1.plot(solution.t, energy_power_a, label='Energy Change from Applied Torque', color='red')
# ax2.plot(solution.t, total_energy, label='Total Energy', color='orange')
ax1.set_ylabel('Energy (J)', color='tab:red')  # Label for the second y-axis
ax1.set_xlabel('Time (s)')  # Label for the x-axis
ax1.tick_params(axis='y', labelcolor='tab:red')  # Color the ticks


ax2 = ax1.twinx()  # Create a second y-axis
ax2.plot(solution.t, power_a, label='Power due to Applied Torque')
ax2.set_xlabel('Time (s)')  # Label for the x-axis
ax2.set_ylabel('Power (W)', color='tab:blue')  # Label for the first y-axis
ax2.tick_params(axis='y', labelcolor='tab:blue')  # Color the ticks

ax1.legend(loc='upper left')
ax2.legend(loc='upper right')
# ax2.set_ylim(ax1.set_ylim())

#########################################################################################
plt.subplot(2,1,2)
ax1 = plt.gca()  # Get the current axes

# Create the second
ax1.plot(solution.t, energy_abs_power_a, label='Energy Drawn from Motor', color='red')
ax1.set_ylabel('Energy (J)', color='tab:red')  # Label for the second y-axis
ax1.set_xlabel('Time (s)')  # Label for the x-axis
ax1.tick_params(axis='y', labelcolor='tab:red')  # Color the ticks


ax2 = ax1.twinx()  # Create a second y-axis
ax2.plot(solution.t, abs_power_a, label='Absolute Value of Power due to Applied Torque')
ax2.set_xlabel('Time (s)')  # Label for the x-axis
ax2.set_ylabel('Power (W)', color='tab:blue')  # Label for the first y-axis
ax2.tick_params(axis='y', labelcolor='tab:blue')  # Color the ticks

ax1.legend(loc='upper left')
ax2.legend(loc='right')
# ax2.set_ylim(ax1.set_ylim())

plt.show()