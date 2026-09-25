import requests
import threading
import time
import re
import sys
import signal
import math
import queue

import rtde_control
import rtde_receive


# ---------------- ROBOT CONFIG ----------------

PRINTHEAD_IP = "192.168.12.4"
PRINTBED_IP  = "192.168.0.100"

MM_TO_M = 0.001

SERVO_DT = 0.01       # 100 Hz
LOOKAHEAD = 0.1
GAIN = 300

stop_requested = False


# ---------------- EXTRUSION CONVERSION ----------------

EC_DEG_TO_RAD = math.pi / 180.0


# ---------------- OFFSETS ----------------

X_OFFSET = 0
Y_OFFSET = 0
Z_OFFSET = 0


# ---------------- ROBOTS ----------------

printhead = rtde_control.RTDEControlInterface(
    PRINTHEAD_IP
)

printhead_r = rtde_receive.RTDEReceiveInterface(
    PRINTHEAD_IP
)

printbed = rtde_control.RTDEControlInterface(
    PRINTBED_IP
)

printbed_r = rtde_receive.RTDEReceiveInterface(
    PRINTBED_IP
)


# ---------------- KLIPPER ----------------

URL = "http://192.168.142.9:7125/server/jsonrpc"

DEFAULT_KLIPPER_GCODE = (
    "ContinuesExtrude_Test_Small_E_Vals.gcode"
)

FeedRatio = 4


# ---------------- STATE ----------------

gcode_queue = queue.Queue()


# ---------------- PRINTBED ----------------

BED_SPEED_SCALE = 1.00

ec_command = 0.0
last_ec_target = 0.0


# =========================================================
# LOG
# =========================================================

def log(x):
    print(x)


# =========================================================
# KLIPPER CALL
# =========================================================

def call(method, params=None):

    return requests.post(
        URL,
        json={
            "jsonrpc": "2.0",
            "method": method,
            "params": params or {},
            "id": 1
        },
        timeout=2
    ).json()


def gcode(cmd):
    return call(
        "printer.gcode.script",
        {"script": cmd}
    )


def start_print(file):
    return call(
        "printer.print.start",
        {"filename": file}
    )


def cancel_print():
    return call(
        "printer.print.cancel"
    )


# =========================================================
# GCODE STATE
# =========================================================

def get_state():

    try:

        r = call(
            "printer.objects.query",
            {
                "objects": {
                    "print_stats": None
                }
            }
        )

        return (
            r["result"]
             ["status"]
             ["print_stats"]
             ["state"]
        )

    except:

        return None


def wait_state(target):

    t0 = time.time()

    while time.time() - t0 < 300:

        if get_state() == target:
            return True

        time.sleep(0.2)

    return False


# =========================================================
# PARSE GCODE
#
# IMPORTANT:
#
# Orientation is now stored as:
#
#     QW QX QY QZ
#
# NOT RX RY RZ.
# =========================================================

def get_val(pattern, line):

    m = re.search(
        pattern,
        line
    )

    return float(
        m.group(1)
    ) if m else None


def parse_gcode(file_path):

    x = y = z = 0.0

    qw = 1.0
    qx = 0.0
    qy = 0.0
    qz = 0.0

    f = 1200.0

    ec = 0.0
    pd = 0.0

    path = []

    with open(file_path) as fobj:

        for line in fobj:

            line = line.strip()

            if (
                not line
                or line.startswith(";")
            ):
                continue

            if (
                line.startswith("G0")
                or line.startswith("G1")
            ):

                # -------------------------------------------------
                # Position
                # -------------------------------------------------

                nx = get_val(
                    r"X([-+]?\d*\.?\d+)",
                    line
                )

                ny = get_val(
                    r"Y([-+]?\d*\.?\d+)",
                    line
                )

                nz = get_val(
                    r"Z([-+]?\d*\.?\d+)",
                    line
                )


                # -------------------------------------------------
                # Quaternion
                # -------------------------------------------------

                nqw = get_val(
                    r"QW([-+]?\d*\.?\d+)",
                    line
                )

                nqx = get_val(
                    r"QX([-+]?\d*\.?\d+)",
                    line
                )

                nqy = get_val(
                    r"QY([-+]?\d*\.?\d+)",
                    line
                )

                nqz = get_val(
                    r"QZ([-+]?\d*\.?\d+)",
                    line
                )


                # -------------------------------------------------
                # Other values
                # -------------------------------------------------

                nf = get_val(
                    r"F([-+]?\d*\.?\d+)",
                    line
                )

                nec = get_val(
                    r"EC([-+]?\d*\.?\d+)",
                    line
                )

                npd = get_val(
                    r"PD([-+]?\d*\.?\d+)",
                    line
                )


                # -------------------------------------------------
                # Update state
                # -------------------------------------------------

                if nx is not None:
                    x = nx

                if ny is not None:
                    y = ny

                if nz is not None:
                    z = nz


                if nqw is not None:
                    qw = nqw

                if nqx is not None:
                    qx = nqx

                if nqy is not None:
                    qy = nqy

                if nqz is not None:
                    qz = nqz


                if nf is not None:
                    f = nf

                if nec is not None:
                    ec = (
                        nec
                        * EC_DEG_TO_RAD
                    )

                if npd is not None:
                    pd = npd


                # -------------------------------------------------
                # Normalize quaternion
                # -------------------------------------------------

                q = quat_normalize(
                    [
                        qw,
                        qx,
                        qy,
                        qz
                    ]
                )

                qw = q[0]
                qx = q[1]
                qy = q[2]
                qz = q[3]


                # -------------------------------------------------
                # Store point
                # -------------------------------------------------

                path.append({

                    "x":
                        x * MM_TO_M
                        + X_OFFSET,

                    "y":
                        y * MM_TO_M
                        + Y_OFFSET,

                    "z":
                        z * MM_TO_M
                        + Z_OFFSET,


                    "qw": qw,
                    "qx": qx,
                    "qy": qy,
                    "qz": qz,


                    "f": f,
                    "ec": ec,

                    "pd": pd
                })


    # =========================================================
    # Build timestamps from PD
    # =========================================================

    t = 0.0

    if path:

        path[0]["t"] = 0.0

    for i in range(
        1,
        len(path)
    ):

        speed_mm_s = max(
            0.001,
            path[i]["f"] / 60.0
        )

        dt = (
            path[i]["pd"]
            / speed_mm_s
        )

        t += dt

        path[i]["t"] = t


    return path


# =========================================================
# QUATERNION NORMALIZATION
# =========================================================

def quat_normalize(q):

    n = math.sqrt(
        q[0] * q[0]
        + q[1] * q[1]
        + q[2] * q[2]
        + q[3] * q[3]
    )

    if n < 1e-12:

        return [
            1.0,
            0.0,
            0.0,
            0.0
        ]

    return [
        q[0] / n,
        q[1] / n,
        q[2] / n,
        q[3] / n
    ]


# =========================================================
# QUATERNION SLERP
#
# q format:
#
#     [qw, qx, qy, qz]
# =========================================================

def quat_slerp(
        q0,
        q1,
        t
):

    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)


    # ---------------------------------------------------------
    # Quaternion dot product
    # ---------------------------------------------------------

    dot = (
        q0[0] * q1[0]
        + q0[1] * q1[1]
        + q0[2] * q1[2]
        + q0[3] * q1[3]
    )


    # ---------------------------------------------------------
    # q and -q represent the same orientation.
    #
    # Flip q1 so SLERP takes the shortest path.
    # ---------------------------------------------------------

    if dot < 0.0:

        q1 = [
            -q1[0],
            -q1[1],
            -q1[2],
            -q1[3]
        ]

        dot = -dot


    dot = max(
        -1.0,
        min(
            1.0,
            dot
        )
    )


    # ---------------------------------------------------------
    # Very small angular difference
    #
    # Linear interpolation is numerically more stable here.
    # ---------------------------------------------------------

    if dot > 0.9995:

        result = [

            q0[0]
            + t * (q1[0] - q0[0]),

            q0[1]
            + t * (q1[1] - q0[1]),

            q0[2]
            + t * (q1[2] - q0[2]),

            q0[3]
            + t * (q1[3] - q0[3])
        ]

        return quat_normalize(
            result
        )


    # ---------------------------------------------------------
    # Standard SLERP
    # ---------------------------------------------------------

    theta_0 = math.acos(dot)

    sin_theta_0 = math.sin(
        theta_0
    )

    theta = theta_0 * t

    sin_theta = math.sin(
        theta
    )


    s0 = (
        math.cos(theta)
        - dot
        * sin_theta
        / sin_theta_0
    )

    s1 = (
        sin_theta
        / sin_theta_0
    )


    return quat_normalize(
        [

            s0 * q0[0]
            + s1 * q1[0],

            s0 * q0[1]
            + s1 * q1[1],

            s0 * q0[2]
            + s1 * q1[2],

            s0 * q0[3]
            + s1 * q1[3]

        ]
    )


# =========================================================
# QUATERNION -> UR ROTATION VECTOR
#
# Input:
#
#     [qw, qx, qy, qz]
#
# Output:
#
#     [rx, ry, rz]
#
# radians
# =========================================================

def quaternion_to_rotvec(q):

    q = quat_normalize(q)


    # ---------------------------------------------------------
    # Force shortest representation.
    # ---------------------------------------------------------

    if q[0] < 0.0:

        q = [
            -q[0],
            -q[1],
            -q[2],
            -q[3]
        ]


    w = max(
        -1.0,
        min(
            1.0,
            q[0]
        )
    )

    x = q[1]
    y = q[2]
    z = q[3]


    # ---------------------------------------------------------
    # Angle
    # ---------------------------------------------------------

    angle = (
        2.0
        * math.acos(w)
    )


    sin_half = math.sqrt(
        max(
            0.0,
            1.0 - w * w
        )
    )


    # ---------------------------------------------------------
    # Very small rotation
    # ---------------------------------------------------------

    if sin_half < 1e-12:

        return [
            0.0,
            0.0,
            0.0
        ]


    axis = [

        x / sin_half,
        y / sin_half,
        z / sin_half

    ]


    return [

        axis[0] * angle,
        axis[1] * angle,
        axis[2] * angle

    ]


# =========================================================
# INTERPOLATION
# =========================================================

def interp(
        a,
        b,
        p
):

    return (
        a
        + (b - a) * p
    )


# =========================================================
# SAFE CANCEL
# =========================================================

def safe_cancel_print():

    try:

        cancel_print()

    except:

        pass


# =========================================================
# LIFT ROBOT
# =========================================================

def lift_robot(
        z_offset=0.005
):

    try:

        log(
            "Stopping servo mode before lift"
        )


        try:
            printhead.servoStop()
        except:
            pass


        try:
            printbed.servoStop()
        except:
            pass


        time.sleep(
            0.5
        )


        pose = (
            printhead_r
            .getActualTCPPose()
        )


        log(
            f"Current TCP before lift: "
            f"X={pose[0]:.4f}, "
            f"Y={pose[1]:.4f}, "
            f"Z={pose[2]:.4f}"
        )


        lifted_pose = pose.copy()

        lifted_pose[2] += z_offset


        log(
            f"Lifting to: "
            f"X={lifted_pose[0]:.4f}, "
            f"Y={lifted_pose[1]:.4f}, "
            f"Z={lifted_pose[2]:.4f}"
        )


        printhead.moveL(
            lifted_pose,
            0.1,
            0.1
        )


        time.sleep(
            0.2
        )


    except Exception as e:

        log(
            f"lift error: {e}"
        )


# =========================================================
# MAIN SYNC LOOP
# =========================================================

def run_synchronized(
        path,
        klipper_file
):

    log(
        f"Loaded {len(path)} points"
    )


    if len(path) < 2:

        log(
            "Not enough trajectory points."
        )

        return


    start_print(klipper_file)

    wait_state("paused")
    gcode("RESUME")
    wait_state("printing")


    # ---------------------------------------------------------
    # Start global clock
    # ---------------------------------------------------------

    start_time = (
        time.perf_counter()
    )


    # ---------------------------------------------------------
    # Bed initial position
    # ---------------------------------------------------------

    base_wrist3 = (
        printbed_r
        .getActualQ()[5]
    )


    global ec_command

    ec0 = None

    segment = 0


    # ---------------------------------------------------------
    # Main 100 Hz loop
    # ---------------------------------------------------------

    while not stop_requested:

        loop_start = (
            time.perf_counter()
        )


        t = (
            time.perf_counter()
            - start_time
        )


        # -----------------------------------------------------
        # End of trajectory
        # -----------------------------------------------------

        if t >= path[-1]["t"]:

            break


        # -----------------------------------------------------
        # Find current segment
        # -----------------------------------------------------

        while (
            segment < len(path) - 2
            and t > path[segment + 1]["t"]
        ):

            segment += 1


        p0 = path[segment]
        p1 = path[segment + 1]


        segment_dt = (
            p1["t"]
            - p0["t"]
        )


        if segment_dt <= 0:

            continue


        phase = (
            t - p0["t"]
        ) / segment_dt


        phase = max(
            0.0,
            min(
                1.0,
                phase
            )
        )


        # =====================================================
        # LINEAR POSITION INTERPOLATION
        # =====================================================

        x = interp(
            p0["x"],
            p1["x"],
            phase
        )

        y = interp(
            p0["y"],
            p1["y"],
            phase
        )

        z = interp(
            p0["z"],
            p1["z"],
            phase
        )


        # =====================================================
        # QUATERNION INTERPOLATION
        # =====================================================

        q0 = [

            p0["qw"],
            p0["qx"],
            p0["qy"],
            p0["qz"]

        ]

        q1 = [

            p1["qw"],
            p1["qx"],
            p1["qy"],
            p1["qz"]

        ]


        # -----------------------------------------------------
        # SLERP
        # -----------------------------------------------------

        q_interp = quat_slerp(
            q0,
            q1,
            phase
        )


        # =====================================================
        # QUATERNION -> UR ROTATION VECTOR
        # =====================================================

        rx, ry, rz = (
            quaternion_to_rotvec(
                q_interp
            )
        )


        # =====================================================
        # FINAL UR CARTESIAN POSE
        # =====================================================

        pose = [

            x,
            y,
            z,

            rx,
            ry,
            rz

        ]


        # =====================================================
        # LINEAR VELOCITY
        # =====================================================

        dx = (
            p1["x"]
            - p0["x"]
        )

        dy = (
            p1["y"]
            - p0["y"]
        )

        dz = (
            p1["z"]
            - p0["z"]
        )


        linear_distance = math.sqrt(

            dx * dx
            + dy * dy
            + dz * dz

        )


        linear_velocity = (
            linear_distance
            / segment_dt
        )


        # =====================================================
        # ANGULAR VELOCITY
        #
        # Calculate actual angular distance between q0/q1.
        # =====================================================

        q0n = quat_normalize(q0)
        q1n = quat_normalize(q1)


        dot = (

            q0n[0] * q1n[0]
            + q0n[1] * q1n[1]
            + q0n[2] * q1n[2]
            + q0n[3] * q1n[3]

        )


        dot = abs(dot)

        dot = max(
            -1.0,
            min(
                1.0,
                dot
            )
        )


        angular_distance = (
            2.0
            * math.acos(dot)
        )


        rotation_velocity = (
            angular_distance
            / segment_dt
        )


        # =====================================================
        # PRINTHEAD SERVO
        # =====================================================

        printhead.servoL(

            pose,

            linear_velocity,

            rotation_velocity,

            SERVO_DT,

            LOOKAHEAD,

            GAIN

        )


        # =====================================================
        # BED MOTION
        # =====================================================

        ec_target = interp(

            p0["ec"],
            p1["ec"],
            phase

        )


        if ec0 is None:

            ec0 = ec_target


        ec_command = (
            ec_target
            - ec0
        ) * BED_SPEED_SCALE


        q = (
            printbed_r
            .getActualQ()
            .copy()
        )


        q[5] = (
            base_wrist3
            + ec_command
        )


        # -----------------------------------------------------
        # Bed angular velocity
        # -----------------------------------------------------

        dec = (
            p1["ec"]
            - p0["ec"]
        )


        bed_velocity = (
            abs(dec)
            / segment_dt
        )


        printbed.servoJ(

            q,

            bed_velocity,
            bed_velocity,

            SERVO_DT,

            LOOKAHEAD,

            GAIN

        )


        # =====================================================
        # MAINTAIN 100 Hz GLOBAL LOOP
        # =====================================================

        elapsed = (
            time.perf_counter()
            - loop_start
        )


        sleep_time = (
            SERVO_DT
            - elapsed
        )


        if sleep_time > 0:

            time.sleep(
                sleep_time
            )


    # =========================================================
    # FINISH
    # =========================================================

    safe_cancel_print()

    lift_robot(
        0.001
    )

    printhead.servoStop()
    printbed.servoStop()

    log(
        "SYNC MOTION COMPLETE"
    )


# =========================================================
# STOP HANDLER
# =========================================================

def stop(
        sig,
        frame
):

    global stop_requested

    log(
        "CTRL+C received"
    )

    stop_requested = True


    try:

        safe_cancel_print()

    except:

        pass


    try:

        lift_robot(
            0.005
        )

    except Exception as e:

        log(
            f"Stop lift error: {e}"
        )


    log(
        "Robot stopped safely"
    )

    sys.exit(0)


# =========================================================
# ENTRY
# =========================================================

if __name__ == "__main__":

    signal.signal(
        signal.SIGINT,
        stop
    )


    path = parse_gcode(
        "reevaluated_output_quat.nc"
    )


    run_synchronized(
        path,
        DEFAULT_KLIPPER_GCODE
    )