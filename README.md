# thymio — cruise + distance control + avoider

A [Thymio II](https://www.thymio.org/) robot that drives with a distance
controller on its seven horizontal proximity sensors (five front, two back):

- **nothing in front** → cruise forward **fast**;
- **object ahead, far** → approach it, slowing as it nears;
- **object ahead, at the setpoint** → **stay** put;
- **object ahead, too close** → **avoid**: back off briskly;
- **object behind** → drive forward away from it.

It steers diagonally, beeps frantically if it's boxed in front and back, and you
can **pause/resume** it with the round center button.

## Important: the sensors only see *close*

Measured on the real robot, the front sensors read **0 with nothing in front**,
then jump straight to **~1280** when an object enters a small close window, and
climb to **~4557 right at contact**. There is **no detection at 10 cm** — the
useful range is only a few centimetres. So this regulates distance *within that
close window*, not at arm's length. `PROX_TARGET` is the hold reading; bigger =
hold closer.

Re-measure your own robot any time with:
```
.venv\Scripts\python.exe read_sensors.py
```
(or just press Run on `read_sensors.py` in Thonny) and set `PROX_TARGET` to the
`fmax` you see at the distance you want.

## How it works

The wheel behaviour keys off the front reading and the setpoint:

```
front <= DETECT                  open road -> cruise forward FAST
front detected, below setpoint   approach, speed proportional to the gap
front within STAY_BAND of target STAY put
front above setpoint             too close -> AVOID (back off, stronger gain)
```

A rear object adds a forward push. **Diagonal steering**: the front sensors steer
toward a lead object while cruising and diagonally away while backing off; the
rear sensors steer the forward escape away. Wheel speeds scale together when they
saturate, so the diagonal is never clipped. **Panic**: boxed in front and back
for >4 s → frantic beeping. **Pause**: tap the round center button.

> ⚠️ On the open road it drives forward fast on its own — give it room and mind
> table edges the sensors can't see, or tap the center button to pause.

## Run it

### With a real Thymio
1. Install [Thymio Suite](https://www.thymio.org/program/) and **launch it**, then
   connect the Thymio. Thymio Suite runs the Device Manager (TDM) the script talks
   to — it must be open, and nothing else (another script, a VPL window) may hold
   the robot, or you'll get a "busy" lock error.
2. `​.venv\Scripts\python.exe -m pip install tdmclient`
3. `​.venv\Scripts\python.exe thymio_distance.py`
   It cruises fast, then holds a short distance off whatever it meets. **Tap the
   center button to pause/resume;** `Ctrl+C` stops it (motors zeroed on exit).

### Calibrate the sensors
```
.venv\Scripts\python.exe read_sensors.py
```

### Without a robot (offline test)
```
.venv\Scripts\python.exe thymio_distance.py --sim
```

## Tuning

| Constant        | Meaning                                                       |
|-----------------|---------------------------------------------------------------|
| `PROX_TARGET`   | hold reading (bigger = hold closer; lower toward ~1400 = stop as soon as detected, i.e. farthest) |
| `DETECT`        | below this = nothing in front → cruise                        |
| `STAY_BAND`     | half-width of the "stay" zone (wider = steadier, looser hold) |
| `CRUISE_SPEED`  | open-road forward speed                                       |
| `MAX_SPEED`     | wheel-speed clamp                                            |
| `KP`            | approach gain                                                 |
| `KP_AVOID`      | too-close gain — how briskly it backs off                     |
| `KT` / `TURN_DEADBAND` | diagonal steering strength / dead-zone                 |
| `ALARM_AFTER` / `BEEP_EVERY` | panic delay / beep cadence                       |

## Files
- `thymio_distance.py` — the controller (plus `--sim` mode)
- `read_sensors.py` — live sensor monitor for calibration
- `.venv/` — local virtual environment (not tracked)
