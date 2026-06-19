"""
Live Thymio proximity-sensor monitor -- for calibrating thymio_distance.py.

Just press Run in Thonny. Then slowly move a flat object (book, box, hand) from
far away (~20 cm) straight toward the FRONT sensors until it touches, and watch
the 'fmax' column. Note the value at the distance where you want the robot to
stop -- that number is your PROX_TARGET.

Tell me three readings and I'll set the controller to YOUR robot:
  * fmax with nothing in front (clear path)
  * fmax at the distance you want it to hold (measure it, e.g. 10 cm)
  * fmax when something is very close / almost touching

Stop it with the Thymio's round CENTER button, or Thonny's stop / Ctrl+C.

NOTE: only one program can use the robot at a time. If you get a "busy" / lock
error, make sure the main controller isn't running and no Thymio Suite
programming window (VPL/Blockly/Aseba) has the robot open.
"""

from tdmclient import ClientAsync


def main():
    try:
        client = ClientAsync()
    except OSError:
        print("Could not reach the Thymio Device Manager -- is Thymio Suite open"
              " with the robot connected?")
        return

    with client:
        async def prog():
            with await client.lock() as node:
                await node.wait_for_variables({"prox.horizontal", "button.center"})
                print("Move an object toward the FRONT; read 'fmax' at your target distance.")
                print("(press the Thymio CENTER button to stop)\n")
                print(f"  {'front sensors [0..4]':28s}{'fmax':>6}{'bmax':>6}")
                while node.v.button.center == 0:
                    prox = list(node.v.prox.horizontal)
                    print(f"  {str(prox[0:5]):28s}{max(prox[0:5]):6d}{max(prox[5:7]):6d}")
                    await client.sleep(0.2)
                print("\ncenter button pressed -- stopped.")

        client.run_async_program(prog)


if __name__ == "__main__":
    main()
