"""Client code that you write."""

import time
import numpy as np
from ae483.clients import CrazyflieClient


class MyCrazyflieClient(CrazyflieClient):
    def move_smooth(self, p_inW_1, p_inW_2, yaw, v):
        print(f'Move smoothly from {p_inW_1} to {p_inW_2} at {v} meters / second')

        p_inW_1 = np.array(p_inW_1)
        p_inW_2 = np.array(p_inW_2)

        d = 0.               # FIXME (A) — distance from p_inW_1 to p_inW_2
        dt = 0.              # FIXME (B) — time to travel that distance at speed v

        start_time = time.time()
        while True:
            t = time.time()
            s = 0.           # FIXME (C) — fraction of the distance travelled by time t
            p_inW_des = 0.   # FIXME (D) — desired position at time t

            self.cf.commander.send_position_setpoint(
                p_inW_des[0], p_inW_des[1], p_inW_des[2], yaw
            )

            if s >= 1:
                return
            else:
                time.sleep(0.1)