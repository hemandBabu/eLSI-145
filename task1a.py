"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1a.py --eval
  3. Run this file:      python3 task1a_template.py

WHAT YOU IMPLEMENT
  Only control_loop(). Everything else (connecting, receiving sensors,
  sending motor commands) is handled for you by CoppeliaClient.
  Don't Edit this file except control_loop().
  You can add helper functions if you like.

Team ID: [ 145 ]
"""

import time

from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; a higher value means the line is detected.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']
# Global PID variables

"""Here is  a complete PID implementation for control_loop(), with proper error calculation, integral windup protection, and derivative smoothing:python"""

"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
===================================================
Participant template.

HOW TO RUN
  1. Open the Task 1A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1a.py --eval
  3. Run this file:      python3 task1a_template.py

WHAT YOU IMPLEMENT
  Only control_loop(). Everything else (connecting, receiving sensors,
  sending motor commands) is handled for you by CoppeliaClient.
  Don't Edit this file except control_loop().
  You can add helper functions if you like.

Team ID: [ 145 ]
"""

import time
from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; a higher value means the line is detected.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# ---------------- PID tuning ----------------
KP = 2.5
KI = 0.01
KD = 0.6

BASE_SPEED = 2.0          # nominal forward speed
MAX_SPEED = 4.0           # clamp wheel speeds to this
INTEGRAL_LIMIT = 5.0       # anti-windup clamp on accumulated error
LOST_LINE_THRESHOLD = 0.05  # below this on all sensors -> "no line seen"

# Weights used to convert 5 sensor readings into one position error.
# Negative weights = left side, positive = right side, 0 = center.
WEIGHTS = {
    'left_corner': -2.0,
    'left': -1.0,
    'middle': 0.0,
    'right': 1.0,
    'right_corner': 2.0,
}

# Global PID state (persists between control_loop calls)
_integral = 0.0
_prev_error = 0.0
_prev_time = None
_last_error_sign = 0.0   # remembers which side the line was last seen on


def _compute_error(sensors):
    """
    Weighted-average line position error.
    Returns a float roughly in [-2, 2]; negative = line is to the left,
    positive = line is to the right, 0 = centered.
    Falls back to last known direction if no sensor sees the line.
    """
    global _last_error_sign

    total_weighted = 0.0
    total_signal = 0.0
    for key in SENSOR_ORDER:
        val = sensors.get(key, 0.0)
        total_weighted += WEIGHTS[key] * val
        total_signal += val

    if total_signal < LOST_LINE_THRESHOLD:
        # Line lost: keep turning the direction it was last seen,
        # using a strong error so the robot actively searches for it.
        return 2.0 * (_last_error_sign if _last_error_sign != 0 else 1.0)

    error = total_weighted / total_signal
    _last_error_sign = 1.0 if error > 0 else (-1.0 if error < 0 else _last_error_sign)
    return error


def control_loop(sensors):
    sensor = list(sensors.values())
    weight = list(WEIGHTS.values())
    numo = 0
    deno = 0
    setpoint = 0

    for i in range(5):
        numo += sensor[i] * weight[i]
        deno += sensor[i]

    if deno != 0:
        position = numo / deno
    else:
        position = 0

    error = setpoint - position
    _integral += error
    _prev_error = error
    derivative = error - _prev_error

    PID = KP * error + KI * _integral + KD * derivative

    left = BASE_SPEED - PID
    right = BASE_SPEED + PID
    
    return left, right

"""
def control_loop(sensors):

    Return (left_speed, right_speed) for the current sensor reading.

    `sensors` is a dict, e.g.:
        {'left_corner': 0.02, 'left': 0.41, 'middle': 0.95,
         'right': 0.05, 'right_corner': 0.01}

    ------------------------------------------------------------------
    TODO (participants): implement your PID line-following controller.
    ------------------------------------------------------------------
    A typical approach:
      1. Turn the 5 readings into ONE line-position error
      2. Feed that error through a PID controller:
      3. Drive the wheels differentially:
    
    # ----- placeholder: drive straight slowly. REPLACE THIS. -----
    base_speed = 2.0
    left = base_speed
    right = base_speed
    return left, right
"""

def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_sensors = None
    try:
        while True:
            # Pull the freshest sensor packet; reuse the last one between packets.
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            left, right = control_loop (last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)   # stop the robot
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
