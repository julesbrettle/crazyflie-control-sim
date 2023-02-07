from main import run_sim
import numpy as np

def sweep():
    kd=0
    step = 0.05
    for kp in np.arange(step, 10, step):
        if kp %1==0:
            print(kp)
        for ki in np.arange(step, 10, step):
            _, (rise_time, settling_time, overshoot, steadystate_error) = run_sim(f'main.py kp={kp} ki={ki} kd={kd}'.split(' '))
            if rise_time < 2.5 and settling_time < 8 and overshoot < 3.0 and steadystate_error < .5:
                print(f"Hey- kp is {kp}, and ki is {ki}")


if __name__ == '__main__':
    sweep()
