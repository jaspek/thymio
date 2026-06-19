# thymio — constant-distance keeper

Keep a [Thymio II](https://www.thymio.org/) robot at a **constant distance from
the object in front of it**, using its built-in horizontal proximity sensors and
a feedback controller on the wheel speeds.

This is the distance-control counterpart to the GoPiGo project
([gopigo-sysid](https://github.com/jaspek/gopigo-sysid)): the same control idea
— regulate one sensor reading to a setpoint with feedback — but with the ToF
sensor replaced by the Thymio's proximity sensors.

## How it works

The Thymio proximity sensors report a **larger** number when an object is
**closer** (about `4500` at contact, dropping to `0` beyond ~12 cm). We pick a
target reading that corresponds to the distance we want to hold:

```
error = PROX_TARGET - measured
  too close -> measured > target -> error < 0 -> drive backward
  too far    -> measured < target -> error > 0 -> drive forward
```

Both wheels get the **same** speed, so the robot moves straight in or out until
the reading — and therefore the distance — settles on the target.
`front_proximity()` uses the closest of the five front sensors, so it works
wherever the object sits in front of the robot.

## Run it

### With a real Thymio
1. Install [Thymio Suite](https://www.thymio.org/program/) and launch it, then
   connect the Thymio (USB cable or the wireless dongle). Thymio Suite runs the
   Thymio Device Manager (TDM) that this script talks to.
2. Install the client library:
   ```
   .venv\Scripts\python.exe -m pip install tdmclient
   ```
3. Run:
   ```
   .venv\Scripts\python.exe thymio_distance.py
   ```
   Place an object (a hand, a box, a wall) in front of the robot; it drives to
   the target distance and holds it. Press `Ctrl+C` to stop — the motors are
   always set back to zero on exit.

### Without a robot (offline test)
```
.venv\Scripts\python.exe thymio_distance.py --sim
```
Simulates a robot starting 30 cm from a wall and prints how the gap, the
proximity reading and the wheel command evolve until it settles on the target.

## Tuning

All knobs are constants at the top of [`thymio_distance.py`](thymio_distance.py):

| Constant      | Meaning                                                        |
|---------------|----------------------------------------------------------------|
| `PROX_TARGET` | the distance to hold (bigger = sit closer to the object)       |
| `DEADBAND`    | how close to the target counts as "good enough" (anti-jitter)  |
| `MAX_SPEED`   | wheel-speed clamp                                              |
| `KP/KI/KD`    | feedback gains — start with plain P, add `KD` if it overshoots |

## Files
- `thymio_distance.py` — the controller (and the `--sim` offline test)
- `.venv/` — local virtual environment (not tracked)
