"""
Thymio constant-distance keeper
===============================

Keep the Thymio II at a constant distance from the object in front of it,
using the five front horizontal proximity sensors and a feedback controller
on the wheel speeds.

The Thymio proximity sensors report a LARGER number when an object is CLOSER
(roughly 4500 at contact, dropping to 0 beyond ~12 cm). So we pick a target
reading PROX_TARGET that corresponds to the distance we want to hold, and:

    error = PROX_TARGET - measured
      object too close -> measured > target -> error < 0 -> drive backward
      object too far    -> measured < target -> error > 0 -> drive forward

Both wheels get the same speed, so the robot moves straight in/out until the
reading (= the distance) settles on the target. This is the same idea as the
GoPiGo distance-control project, with the ToF sensor replaced by the Thymio's
built-in proximity sensors.

Run with a real robot (Thymio Suite running, Thymio connected):
    python thymio_distance.py

Try the controller without a robot (built-in simulation):
    python thymio_distance.py --sim
"""

import argparse

# ----------------------------- configuration -----------------------------
PROX_TARGET = 1500     # desired front proximity reading (bigger = closer).
                       # Raise it to sit closer to the object, lower it to
                       # keep further away. ~1500 is roughly a hand's width.
DEADBAND    = 60       # ignore tiny errors so the robot does not jitter
MAX_SPEED   = 300      # wheel-target clamp (Thymio motors accept ~ +/-500)
PERIOD      = 0.05     # control-loop period [s]  (~20 Hz)

# Feedback gains (proximity error -> wheel speed). Proximity is a big number
# (0..4500) while the motor target is small (+/-500), so KP is well below 1.
# KD damps the approach; KI removes any steady offset. Start with plain P and
# add KD/KI on the real robot if it overshoots or stops slightly off-target.
KP = 0.20
KI = 0.0
KD = 0.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class PID:
    """Plain PID acting on the proximity error (target - measured)."""

    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        self.integral += error * dt
        deriv = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * deriv


def front_proximity(prox):
    """Closest object seen by the five front sensors (indices 0..4)."""
    return max(prox[0:5])


def command(pid, prox_front, dt):
    """Turn a front proximity reading into equal (left, right) wheel speeds."""
    error = PROX_TARGET - prox_front
    if abs(error) < DEADBAND:
        error = 0.0
    speed = int(clamp(pid.step(error, dt), -MAX_SPEED, MAX_SPEED))
    return speed, speed                     # equal wheels -> straight line


# ------------------------------- real robot -------------------------------
def run_robot():
    # imported lazily so that --sim needs neither tdmclient nor a robot
    from tdmclient import ClientAsync

    with ClientAsync() as client:
        async def prog():
            with await client.lock() as node:
                await node.wait_for_variables({"prox.horizontal"})
                pid = PID(KP, KI, KD)
                print("Holding distance from the object in front (Ctrl+C to stop) ...")
                try:
                    while True:
                        prox = list(node.v.prox.horizontal)
                        left, right = command(pid, front_proximity(prox), PERIOD)
                        node.v.motor.left.target = left
                        node.v.motor.right.target = right
                        node.flush()
                        await client.sleep(PERIOD)
                finally:                              # always stop the motors
                    node.v.motor.left.target = 0
                    node.v.motor.right.target = 0
                    node.flush()

        client.run_async_program(prog)


# ------------------------------- simulation -------------------------------
REACH_M = 0.12         # the proximity sensors see out to about 12 cm


def sim_prox(gap_m):
    """Fake Thymio front proximity: ~4500 at contact, 0 beyond REACH_M."""
    if gap_m >= REACH_M:
        return 0
    return int(4500 * (1 - gap_m / REACH_M))


def run_sim():
    """Drive a simulated robot toward a wall and watch it hold the setpoint."""
    pid = PID(KP, KI, KD)
    gap = 0.30                         # start 30 cm from the wall
    speed_per_unit = 0.0004            # wheel target 500 -> ~0.2 m/s
    target_cm = REACH_M * (1 - PROX_TARGET / 4500) * 100
    print(f"target: prox {PROX_TARGET}  (~{target_cm:.1f} cm)\n")
    print(f"{'t[s]':>5} {'gap[cm]':>8} {'prox':>6} {'wheel':>6}")
    for k in range(160):
        prox_front = sim_prox(gap)
        left, _ = command(pid, prox_front, PERIOD)
        gap = max(gap - left * speed_per_unit * PERIOD, 0.0)   # +speed approaches
        if k % 10 == 0:
            print(f"{k * PERIOD:5.2f} {gap * 100:8.1f} {prox_front:6d} {left:6d}")
    print(f"\nsettled at gap = {gap * 100:.1f} cm")


def main():
    p = argparse.ArgumentParser(description="Thymio constant-distance keeper")
    p.add_argument("--sim", action="store_true",
                   help="run the offline simulation instead of driving a real robot")
    args = p.parse_args()
    run_sim() if args.sim else run_robot()


if __name__ == "__main__":
    main()
