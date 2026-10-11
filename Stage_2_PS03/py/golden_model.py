#!/usr/bin/env python3
"""
PS-03 Golden Model: First-Order Motor Dynamics & Closed-Loop Control
Compares discrete fixed-point PI motor response against standard floating point dynamics.
Pure Python implementation (no numpy dependency).
"""

def clip(val, min_val, max_val):
    return max(min_val, min(val, max_val))

class GoldenMotorController:
    def __init__(self, period=1000, kp=1.0, ki=0.0625, dead_time=20):
        # PWM Parameters
        self.period = period
        self.kp = kp
        self.ki = ki
        self.dead_time = dead_time
        
        # Motor Dynamics Parameters (First-order electrical + mechanical)
        self.R = 1.0       # Resistance (Ohms)
        self.L = 0.001     # Inductance (H)
        self.Ke = 0.05     # Back-EMF Constant (V/(rad/s))
        self.Kt = 0.05     # Torque Constant (Nm/A)
        self.J = 0.0001    # Inertia (kg*m^2)
        self.B = 0.001     # Friction (Nm*s/rad)
        self.V_bus = 24.0  # Supply Voltage
        
        # State Variables
        self.current = 0.0
        self.speed = 0.0
        self.integrator = 0.0
        self.duty = 0.0

    def step_pi(self, setpoint, measured_speed):
        """Simulates PI calculation executed once per PWM period."""
        error = setpoint - measured_speed
        self.integrator += self.ki * error
        # Anti-windup clamping
        self.integrator = clip(self.integrator, -self.period, self.period)
        
        raw_duty = (self.kp * error) + self.integrator
        self.duty = clip(raw_duty, 0, self.period)
        return self.duty

    def simulate_step(self, dt=1e-6):
        """Advances plant motor dynamics by dt seconds."""
        v_in = (self.duty / self.period) * self.V_bus
        di = (v_in - self.R * self.current - self.Ke * self.speed) / self.L
        dw = (self.Kt * self.current - self.B * self.speed) / self.J
        
        self.current += di * dt
        self.speed += dw * dt
        return self.speed

def run_golden_simulation(steps=10000, setpoint=500.0):
    model = GoldenMotorController()
    speed_history = []
    duty_history = []
    
    print(f"Running Golden Model Simulation for Setpoint={setpoint}...")
    for t in range(steps):
        if t % 100 == 0:  # Calculate PI every 100 dt steps (PWM boundary)
            model.step_pi(setpoint, model.speed * 10.0) # Scaled speed feedback
            
        speed = model.simulate_step(dt=1e-6)
        speed_history.append(speed)
        duty_history.append(model.duty)
        
    print(f"Simulation completed. Final Speed: {speed_history[-1]:.2f} rad/s | Final Duty: {duty_history[-1]:.1f}")
    return speed_history, duty_history

if __name__ == "__main__":
    run_golden_simulation()
