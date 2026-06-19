"""
Thymio cruise + distance control + avoider
==========================================

Drive the Thymio II with a distance controller on its seven horizontal proximity
sensors (five front, two back).

Measured on this robot: with nothing in front the sensors read 0; an object only
becomes visible within a small, close window, where the reading jumps to about
1280 and then climbs to ~4557 right at contact. (The Thymio cannot see far -- so
there is no "10 cm" hold; it regulates within that close window.) Bigger reading
= closer object.

Behaviour, by the front reading `front`:
  * front <= DETECT          -> nothing in front (open road): cruise forward FAST;
  * front detected, < target  -> approach it, slowing as it nears;
  * front within STAY_BAND of PROX_TARGET -> STAY put;
  * front > target           -> too close: AVOID, back off briskly;
  * object behind             -> drive forward away from it.

It steers diagonally, beeps frantically if boxed in front AND back, and the round
CENTER button pauses/resumes the motors.

Run with a real robot (Thymio Suite running, Thymio connected):
    python thymio_distance.py

Try the controller without a robot (built-in simulation):
    python thymio_distance.py --sim

Re-measure your sensors any time with:  python read_sensors.py
"""

import argparse

# ----------------------------- configuration -----------------------------
# Calibrated to this robot's readings (0 = clear, ~1280 at the detection edge,
# ~4557 at contact). Re-measure with read_sensors.py if it behaves differently.
PROX_TARGET = 2500     # hold setpoint -- a reading inside the detectable window.
                       # Bigger = hold CLOSER to the object; smaller = farther.
DETECT      = 700      # front below this = nothing in front -> open road (cruise)
STAY_BAND   = 400      # half-width of the "stay" zone around PROX_TARGET
CRUISE_SPEED = 300     # forward speed when nothing is in front (the fast cruise)
MAX_SPEED    = 300     # wheel-target clamp (Thymio motors accept ~ +/-500)
PERIOD       = 0.05    # control-loop period [s]  (~20 Hz)
ALARM_AFTER  = 4.0     # seconds boxed in (object front AND back) before it panics
BEEP_EVERY   = 0.6     # while panicking, repeat the frantic beep this often [s]

# Gains (proximity error -> wheel speed). Readings are big (0..4500), the motor
# target is small (+/-500), so the gains are well below 1.
KP       = 0.20        # approach: how fast it closes a gap to a detected object
KP_AVOID = 0.30        # too-close: a stronger gain for a brisk, decisive back-off
KT            = 0.03   # steering gain on the left-vs-right sensor difference
TURN_DEADBAND = 500    # ignore left/right imbalance below this (readings are big)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def front_proximity(prox):
    """Closest object seen by the five front sensors (indices 0..4)."""
    return max(prox[0:5])


def back_proximity(prox):
    """Closest object seen by the two rear sensors (indices 5..6)."""
    return max(prox[5:7])


def boxed_in(prox):
    """True when something is closer than the setpoint BOTH in front and behind
    -- the robot is hemmed in and cannot drive its way out."""
    return front_proximity(prox) > PROX_TARGET and back_proximity(prox) > PROX_TARGET


def command(prox):
    """Compute (left, right) wheel speeds.

      * nothing in front  -> cruise forward FAST (also flees a rear object);
      * object ahead, far -> approach, speed proportional to the gap;
      * object at setpoint -> stay put;
      * object too close   -> avoid (brisk back-off, stronger gain).
    The front sensors steer toward a lead object while cruising and diagonally
    away while backing off; the rear sensors steer the forward escape away.
    """
    front = front_proximity(prox)
    threat_back = max(0.0, back_proximity(prox) - PROX_TARGET)

    if front <= DETECT:
        v = CRUISE_SPEED                 # open road ahead -> go fast (and flee any rear object)
    else:
        err = (front - PROX_TARGET) - threat_back
        if abs(err) <= STAY_BAND:
            v = 0.0                      # at the setpoint -> stay
        elif err > 0:
            v = -KP_AVOID * err          # too close -> avoid (brisk back-off)
        else:
            v = -KP * err                # approach -> slower as it nears

    turn = 0.0
    front_diff = (prox[0] + prox[1]) - (prox[3] + prox[4])   # +ve: front object on the left
    if front > DETECT and abs(front_diff) > TURN_DEADBAND:
        turn += KT * front_diff      # forward: steer toward lead; reverse: peel away diagonally
    if threat_back > 0:
        back_diff = prox[5] - prox[6]                        # +ve: rear object on the left
        if abs(back_diff) > TURN_DEADBAND:
            turn -= KT * back_diff   # drive forward away from the rear object, diagonally

    left = v - turn
    right = v + turn
    peak = max(abs(left), abs(right))
    if peak > MAX_SPEED:                 # scale both together so the diagonal isn't clipped away
        left *= MAX_SPEED / peak
        right *= MAX_SPEED / peak
    return int(clamp(left, -MAX_SPEED, MAX_SPEED)), int(clamp(right, -MAX_SPEED, MAX_SPEED))


# ------------------------------- real robot -------------------------------
def connect():
    """Connect to the Thymio Device Manager, or print a hint and return None.

    tdmclient is imported here so that --sim needs neither it nor a robot.
    """
    from tdmclient import ClientAsync
    try:
        return ClientAsync()             # connects to the Thymio Device Manager
    except OSError:
        print("Could not reach the Thymio Device Manager (TDM).")
        print("Is Thymio Suite running with the robot connected?")
        print("  1. Launch Thymio Suite -- it starts the TDM the script talks to.")
        print("  2. Connect the Thymio (USB cable or RF dongle) and turn it on.")
        print("  3. Confirm the robot shows up in Thymio Suite, then run again.")
        print("To test without a robot:  python thymio_distance.py --sim")
        return None


async def beep(node, client):
    """Panicky alarm: a fast burst of high, jittering, rising tones. Native sound
    is a function call (not a variable), so each blip compiles+runs a one-line
    Aseba program; this does not disturb the variable-based motor control. (If
    your setup plays no sound, swap each blip for: node.v.sound.system = 2;
    node.flush())"""
    for freq in (1100, 1600, 1200, 1750, 1300, 1850):   # high + rising + jittery = frantic
        await node.compile(f"call sound.freq({freq}, 4)")   # ~0.07 s blip
        await node.run()
        await client.sleep(0.06)


def run_robot():
    client = connect()
    if client is None:
        return
    with client:
        async def prog():
            with await client.lock() as node:
                await node.wait_for_variables({"prox.horizontal", "button.center"})
                stuck_time = 0.0
                beep_timer = 0.0
                paused = False
                prev_button = 0
                print("Running. Tap the round CENTER button to pause/resume. Ctrl+C to stop.")
                try:
                    while True:
                        button = node.v.button.center
                        if button == 1 and prev_button == 0:    # rising edge -> toggle
                            paused = not paused
                            print("PAUSED -- tap center to resume." if paused else "RESUMED.")
                        prev_button = button

                        if paused:
                            node.v.motor.left.target = 0
                            node.v.motor.right.target = 0
                            node.flush()
                            stuck_time = beep_timer = 0.0
                            await client.sleep(PERIOD)
                            continue

                        prox = list(node.v.prox.horizontal)
                        left, right = command(prox)
                        node.v.motor.left.target = left
                        node.v.motor.right.target = right
                        node.flush()

                        if boxed_in(prox):                 # close front AND back
                            stuck_time += PERIOD
                            if stuck_time >= ALARM_AFTER:  # hemmed in too long -> panic
                                beep_timer -= PERIOD
                                if beep_timer <= 0.0:
                                    print("Boxed in -- panicking!")
                                    await beep(node, client)
                                    beep_timer = BEEP_EVERY
                        else:
                            stuck_time = 0.0               # escaped -> calm down
                            beep_timer = 0.0

                        await client.sleep(PERIOD)
                finally:                              # always stop the motors
                    node.v.motor.left.target = 0
                    node.v.motor.right.target = 0
                    node.flush()

        client.run_async_program(prog)


# ------------------------------- simulation -------------------------------
DETECT_GAP = 0.08      # the front sensors only "see" within ~8 cm (small window)


def sim_prox(gap_m):
    """Fake Thymio proximity matching the measured curve: 0 beyond DETECT_GAP,
    then it JUMPS to ~1300 at the edge and climbs to ~4557 at contact."""
    if gap_m >= DETECT_GAP:
        return 0
    return int(1300 + (4557 - 1300) * (1 - gap_m / DETECT_GAP))


def run_sim():
    """Show the behaviour: the robot starts far (out of range) and cruises FAST,
    detects the wall in the close window, approaches and HOLDS at the setpoint;
    then the wall advances and the robot AVOIDS by backing off."""
    gap = 0.25                         # start out of range (nothing detected yet)
    speed_per_unit = 0.0004            # wheel target 500 -> ~0.2 m/s
    wall_closes_at = 5.0               # after this, the wall creeps toward the robot
    print("cruise FAST until the wall is seen, approach + hold, then the wall advances\n")
    print(f"{'t[s]':>5} {'gap[cm]':>8} {'prox':>6} {'wheel':>6}  phase")
    for k in range(220):
        t = k * PERIOD
        front = sim_prox(gap)
        prox = [0, 0, front, 0, 0, 0, 0]   # single wall ahead, nothing behind
        left, _ = command(prox)
        gap = gap - left * speed_per_unit * PERIOD          # robot forward (+) -> gap shrinks
        if t >= wall_closes_at:
            gap -= 0.008 * PERIOD                           # wall creeps in
        gap = max(gap, 0.0)
        if k % 10 == 0:
            phase = "wall advancing" if t >= wall_closes_at else "cruise/approach/hold"
            print(f"{t:5.2f} {gap * 100:8.1f} {front:6d} {left:6d}  {phase}")
    print(f"\nfinal gap = {gap * 100:.1f} cm")


def main():
    p = argparse.ArgumentParser(description="Thymio cruise + distance control + avoider")
    p.add_argument("--sim", action="store_true",
                   help="run the offline simulation instead of driving a real robot")
    args = p.parse_args()
    run_sim() if args.sim else run_robot()


if __name__ == "__main__":
    main()
