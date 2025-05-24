import matplotlib.pyplot as plt
from pid import PID

def sim_first_order():
    # Sim Params
    dt = 0.01
    total_time = 5.0
    steps = int(total_time/dt)

    # PID Controller
    pid = PID(kp=2.0, kd=0.1, ki=2.0, dt=dt)
    pid.reset()


    # System initial state
    x = 0.0
    setpoint = 1.0

    # Data Logging
    x_history = []
    u_history = []
    t_history = []

    # Simulate
    for step in range(steps):
        t = step * dt
        u = pid.compute(setpoint, x)
        dx = -x + u
        x += dx * dt
        
        
        t_history.append(t)
        x_history.append(x)
        u_history.append(u)
        

    # Plot
    plt.figure(figsize=(10, 4))
    plt.suptitle("1st Order PID Control", fontsize=16)
    plt.subplot(1, 2, 1)
    plt.plot(t_history, x_history, label='Output u(t)')
    plt.plot(t_history, [setpoint]*steps, 'r--', label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.legend()
    plt.title("Outputs")

    plt.subplot(1, 2, 2)
    plt.plot(t_history, u_history, label="Control u(t)")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input")
    plt.legend()
    plt.title("Control Signal")

    plt.tight_layout()
    plt.show()


def sim_second_order():
    # Sim Params
    dt = 0.01
    total_time = 10.0
    steps = int(total_time/dt)
    
    # Second order system params
    m = 1.0     # mass
    b = 2.0     # daming
    k = 5.0     # stiffness
    

    # PID Controller
    pid = PID(kp=100.0, kd=20.0, ki=1.0, dt=dt)
    pid.reset()


    # System initial state
    x = 0.0
    v = 0.0
    setpoint = 1.0

    # Data Logging
    x_history = []
    v_history = []
    u_history = []
    t_history = []

    # Simulate
    for step in range(steps):
        t = step * dt
        u = pid.compute(setpoint, x)
        
        a = (u-b * v-k * x) / m
        v += a * dt
        x += v * dt
        
        # Log data
        t_history.append(t)
        x_history.append(x)
        u_history.append(u)
        v_history.append(v)
        

    # Plot
    plt.figure(figsize=(12, 4))
    plt.suptitle("2nd Order PID Control", fontsize=16)
    

    plt.subplot(1, 3, 1)
    plt.plot(t_history, x_history, label="x(t)")
    plt.plot(t_history, [setpoint]*steps, 'r--', label="Setpoint")
    plt.xlabel("Time (s)")
    plt.ylabel("Position")
    plt.legend()
    plt.title("Position")

    plt.subplot(1, 3, 2)
    plt.plot(t_history, v_history, label="v(t)", color='orange')
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity")
    plt.legend()
    plt.title("Velocity")

    plt.subplot(1, 3, 3)
    plt.plot(t_history, u_history, label="u(t)", color='green')
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input")
    plt.legend()
    plt.title("Control Signal")

    plt.tight_layout()
    plt.show()
    
    
if __name__ == "__main__":
    sim_first_order()
    sim_second_order()