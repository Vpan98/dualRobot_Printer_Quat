import tkinter as tk
from tkinter import ttk
import threading
import time
import re
import math
import sys
import signal

import rtde_control
import rtde_receive


# ============================================================
# CONFIGURATION
# ============================================================

PRINTHEAD_IP = "192.168.12.4"
PRINTBED_IP  = "192.168.0.100"

GCODE_FILE = "reevaluated_output_quat.nc"

MM_TO_M = 0.001
EC_DEG_TO_RAD = math.pi / 180.0

# MoveL test parameters
MOVE_SPEED = 0.05
MOVE_ACCEL = 0.20

X_OFFSET = 0.0
Y_OFFSET = 0.0
Z_OFFSET = 0.0


# ============================================================
# ROBOT CONNECTION
# ============================================================

print("Connecting to printhead...")

printhead = rtde_control.RTDEControlInterface(
    PRINTHEAD_IP
)

printhead_r = rtde_receive.RTDEReceiveInterface(
    PRINTHEAD_IP
)

print("Printhead connected.")


# ============================================================
# GLOBAL STATE
# ============================================================

stop_requested = False

path = []

current_index = 0

robot_command_running = False

state_lock = threading.Lock()


# ============================================================
# LOG
# ============================================================

def log(msg):

    print(
        time.strftime("%H:%M:%S"),
        msg
    )


# ============================================================
# QUATERNION
# ============================================================

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


def quat_slerp(q0, q1, t):

    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = (

        q0[0] * q1[0]
        + q0[1] * q1[1]
        + q0[2] * q1[2]
        + q0[3] * q1[3]

    )

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
        min(1.0, dot)
    )

    if dot > 0.9995:

        result = [

            q0[0] + t * (q1[0] - q0[0]),
            q0[1] + t * (q1[1] - q0[1]),
            q0[2] + t * (q1[2] - q0[2]),
            q0[3] + t * (q1[3] - q0[3])

        ]

        return quat_normalize(result)

    theta_0 = math.acos(dot)

    sin_theta_0 = math.sin(theta_0)

    theta = theta_0 * t

    sin_theta = math.sin(theta)

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

    return quat_normalize([

        s0 * q0[0] + s1 * q1[0],
        s0 * q0[1] + s1 * q1[1],
        s0 * q0[2] + s1 * q1[2],
        s0 * q0[3] + s1 * q1[3]

    ])


# ============================================================
# QUATERNION -> UR ROTATION VECTOR
# ============================================================

def quaternion_to_rotvec(q):

    q = quat_normalize(q)

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

    angle = 2.0 * math.acos(w)

    sin_half = math.sqrt(
        max(
            0.0,
            1.0 - w * w
        )
    )

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


# ============================================================
# PARSING
# ============================================================

def get_val(pattern, line):

    m = re.search(
        pattern,
        line
    )

    if m:

        return float(
            m.group(1)
        )

    return None


def parse_gcode(filename):

    poses = []

    x = 0.0
    y = 0.0
    z = 0.0

    qw = 1.0
    qx = 0.0
    qy = 0.0
    qz = 0.0

    f = 1200.0

    ec = 0.0
    pd = 0.0

    with open(
        filename,
        "r"
    ) as file:

        for line_number, line in enumerate(
            file,
            start=1
        ):

            line = line.strip()

            if not line:
                continue

            if line.startswith(";"):
                continue

            if not (
                line.startswith("G0")
                or line.startswith("G1")
            ):
                continue

            # ------------------------------------------------
            # Position
            # ------------------------------------------------

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

            # ------------------------------------------------
            # Quaternion
            # ------------------------------------------------

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

            # ------------------------------------------------
            # Other
            # ------------------------------------------------

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

            # ------------------------------------------------
            # Update
            # ------------------------------------------------

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

            q = quat_normalize([
                qw,
                qx,
                qy,
                qz
            ])

            poses.append({

                "x":
                    x * MM_TO_M
                    + X_OFFSET,

                "y":
                    y * MM_TO_M
                    + Y_OFFSET,

                "z":
                    z * MM_TO_M
                    + Z_OFFSET,

                "qw": q[0],
                "qx": q[1],
                "qy": q[2],
                "qz": q[3],

                "f": f,

                "ec": ec,

                "pd": pd,

                "line":
                    line_number

            })

    # --------------------------------------------------------
    # TIMESTAMPS
    # --------------------------------------------------------

    t = 0.0

    if poses:

        poses[0]["t"] = 0.0

    for i in range(
        1,
        len(poses)
    ):

        speed_mm_s = max(
            0.001,
            poses[i]["f"] / 60.0
        )

        dt = (
            poses[i]["pd"]
            / speed_mm_s
        )

        t += dt

        poses[i]["t"] = t

    return poses


# ============================================================
# TARGET POSE
# ============================================================

def get_target_pose(index):

    p = path[index]

    rv = quaternion_to_rotvec([

        p["qw"],
        p["qx"],
        p["qy"],
        p["qz"]

    ])

    return [

        p["x"],
        p["y"],
        p["z"],

        rv[0],
        rv[1],
        rv[2]

    ]


# ============================================================
# ACTUAL ROBOT
# ============================================================

def get_actual_pose():

    return (
        printhead_r
        .getActualTCPPose()
    )


# ============================================================
# FORMAT
# ============================================================

def fmt(value, digits=3):

    return f"{value:.{digits}f}"


# ============================================================
# MAIN GUI
# ============================================================

class TrajectoryViewer:

    def __init__(self, root):

        self.root = root

        self.root.title(
            "Robot Trajectory Monitor"
        )

        self.root.geometry(
            "900x800"
        )

        self.create_display()

        self.update_display()


    # ========================================================
    # DISPLAY
    # ========================================================

    def create_display(self):

        # ----------------------------------------------------
        # Point information
        # ----------------------------------------------------

        frame = ttk.LabelFrame(
            self.root,
            text="Trajectory Point"
        )

        frame.pack(
            fill="x",
            padx=10,
            pady=10
        )

        self.point_label = ttk.Label(
            frame,
            text="Point: ---",
            font=("Arial", 16)
        )

        self.point_label.pack(
            pady=5
        )


        # ----------------------------------------------------
        # Target
        # ----------------------------------------------------

        target_frame = ttk.LabelFrame(
            self.root,
            text="TARGET"
        )

        target_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.target_text = tk.Text(
            target_frame,
            height=7,
            font=("Consolas", 13)
        )

        self.target_text.pack(
            fill="x",
            padx=5,
            pady=5
        )


        # ----------------------------------------------------
        # Actual
        # ----------------------------------------------------

        actual_frame = ttk.LabelFrame(
            self.root,
            text="ACTUAL PRINTHEAD"
        )

        actual_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.actual_text = tk.Text(
            actual_frame,
            height=7,
            font=("Consolas", 13)
        )

        self.actual_text.pack(
            fill="x",
            padx=5,
            pady=5
        )


        # ----------------------------------------------------
        # Error
        # ----------------------------------------------------

        error_frame = ttk.LabelFrame(
            self.root,
            text="ERROR"
        )

        error_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.error_text = tk.Text(
            error_frame,
            height=6,
            font=("Consolas", 13)
        )

        self.error_text.pack(
            fill="x",
            padx=5,
            pady=5
        )


        # ----------------------------------------------------
        # Quaternion
        # ----------------------------------------------------

        quat_frame = ttk.LabelFrame(
            self.root,
            text="TARGET QUATERNION"
        )

        quat_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.quat_text = tk.Text(
            quat_frame,
            height=3,
            font=("Consolas", 12)
        )

        self.quat_text.pack(
            fill="x",
            padx=5,
            pady=5
        )


    # ========================================================
    # TEXT UPDATE
    # ========================================================

    def replace_text(
            self,
            widget,
            text
    ):

        widget.delete(
            "1.0",
            tk.END
        )

        widget.insert(
            tk.END,
            text
        )


    # ========================================================
    # UPDATE
    # ========================================================

    def update_display(self):

        try:

            if not path:

                return


            with state_lock:

                index = current_index


            if index >= len(path):

                index = len(path) - 1


            p = path[index]


            # ------------------------------------------------
            # TARGET
            # ------------------------------------------------

            target = get_target_pose(
                index
            )

            target_text = (

                f"X  = {target[0] * 1000:10.3f} mm\n"
                f"Y  = {target[1] * 1000:10.3f} mm\n"
                f"Z  = {target[2] * 1000:10.3f} mm\n\n"

                f"RX = {target[3]:10.6f} rad\n"
                f"RY = {target[4]:10.6f} rad\n"
                f"RZ = {target[5]:10.6f} rad"

            )

            self.replace_text(
                self.target_text,
                target_text
            )


            # ------------------------------------------------
            # ACTUAL
            # ------------------------------------------------

            actual = get_actual_pose()


            actual_text = (

                f"X  = {actual[0] * 1000:10.3f} mm\n"
                f"Y  = {actual[1] * 1000:10.3f} mm\n"
                f"Z  = {actual[2] * 1000:10.3f} mm\n\n"

                f"RX = {actual[3]:10.6f} rad\n"
                f"RY = {actual[4]:10.6f} rad\n"
                f"RZ = {actual[5]:10.6f} rad"

            )

            self.replace_text(
                self.actual_text,
                actual_text
            )


            # ------------------------------------------------
            # ERROR
            # ------------------------------------------------

            ex = (
                actual[0]
                - target[0]
            )

            ey = (
                actual[1]
                - target[1]
            )

            ez = (
                actual[2]
                - target[2]
            )

            erx = (
                actual[3]
                - target[3]
            )

            ery = (
                actual[4]
                - target[4]
            )

            erz = (
                actual[5]
                - target[5]
            )


            position_error = math.sqrt(

                ex * ex
                + ey * ey
                + ez * ez

            )


            rotation_error = math.sqrt(

                erx * erx
                + ery * ery
                + erz * erz

            )


            error_text = (

                f"X error  = {ex * 1000:+10.3f} mm\n"
                f"Y error  = {ey * 1000:+10.3f} mm\n"
                f"Z error  = {ez * 1000:+10.3f} mm\n\n"

                f"Position magnitude = "
                f"{position_error * 1000:.3f} mm\n\n"

                f"RX error = {erx:+.6f} rad\n"
                f"RY error = {ery:+.6f} rad\n"
                f"RZ error = {erz:+.6f} rad\n\n"

                f"Rotation magnitude = "
                f"{rotation_error:.6f} rad"

            )

            self.replace_text(
                self.error_text,
                error_text
            )


            # ------------------------------------------------
            # QUATERNION
            # ------------------------------------------------

            quat_text = (

                f"QW = {p['qw']:+.8f}\n"
                f"QX = {p['qx']:+.8f}\n"
                f"QY = {p['qy']:+.8f}\n"
                f"QZ = {p['qz']:+.8f}"

            )

            self.replace_text(
                self.quat_text,
                quat_text
            )


            self.point_label.config(
                text=(
                    f"POINT {index + 1} / {len(path)}"
                    f"     |     G-code line {p['line']}"
                )
            )


        except Exception as e:

            log(
                f"GUI update error: {e}"
            )


        self.root.after(
            100,
            self.update_display
        )


# ============================================================
# CONTROL WINDOW
# ============================================================

class ControlWindow:

    def __init__(self):

        self.root = tk.Toplevel()

        self.root.title(
            "Robot Controls"
        )

        self.root.geometry(
            "420x500"
        )

        self.create_controls()


    # ========================================================
    # CONTROLS
    # ========================================================

    def create_controls(self):

        frame = ttk.LabelFrame(
            self.root,
            text="Point Navigation"
        )

        frame.pack(
            fill="both",
            expand=True,
            padx=15,
            pady=15
        )


        # ----------------------------------------------------
        # Point display
        # ----------------------------------------------------

        self.index_label = ttk.Label(
            frame,
            text="Point 1",
            font=("Arial", 18)
        )

        self.index_label.pack(
            pady=15
        )


        # ----------------------------------------------------
        # Buttons
        # ----------------------------------------------------

        ttk.Button(
            frame,
            text="▲   Previous Point",
            command=lambda: self.move_index(-1)
        ).pack(
            fill="x",
            padx=30,
            pady=5
        )


        ttk.Button(
            frame,
            text="▼   Next Point",
            command=lambda: self.move_index(1)
        ).pack(
            fill="x",
            padx=30,
            pady=5
        )


        ttk.Button(
            frame,
            text="◀   Back 5 Points",
            command=lambda: self.move_index(-5)
        ).pack(
            fill="x",
            padx=30,
            pady=5
        )


        ttk.Button(
            frame,
            text="▶   Forward 5 Points",
            command=lambda: self.move_index(5)
        ).pack(
            fill="x",
            padx=30,
            pady=5
        )


        ttk.Separator(
            frame
        ).pack(
            fill="x",
            padx=20,
            pady=20
        )


        # ----------------------------------------------------
        # MoveL
        # ----------------------------------------------------

        ttk.Button(
            frame,
            text="MOVE ROBOT TO CURRENT POINT",
            command=self.move_robot
        ).pack(
            fill="x",
            padx=20,
            pady=10,
            ipady=10
        )


        # ----------------------------------------------------
        # Stop
        # ----------------------------------------------------

        ttk.Button(
            frame,
            text="STOP ROBOT",
            command=self.stop_robot
        ).pack(
            fill="x",
            padx=20,
            pady=10,
            ipady=10
        )


        self.status = ttk.Label(
            frame,
            text="Ready"
        )

        self.status.pack(
            pady=15
        )


        # ----------------------------------------------------
        # Keyboard
        # ----------------------------------------------------

        self.root.bind(
            "<Up>",
            lambda e: self.move_index(-1)
        )

        self.root.bind(
            "<Down>",
            lambda e: self.move_index(1)
        )

        self.root.bind(
            "<Left>",
            lambda e: self.move_index(-5)
        )

        self.root.bind(
            "<Right>",
            lambda e: self.move_index(5)
        )

        self.root.bind(
            "<space>",
            lambda e: self.move_robot()
        )

        self.root.bind(
            "<Escape>",
            lambda e: self.stop_robot()
        )

        self.root.focus_force()

        self.update_label()


    # ========================================================
    # INDEX
    # ========================================================

    def move_index(self, amount):

        global current_index

        with state_lock:

            current_index += amount

            current_index = max(
                0,
                min(
                    len(path) - 1,
                    current_index
                )
            )

        self.update_label()


    def update_label(self):

        with state_lock:

            index = current_index

        self.index_label.config(
            text=f"Point {index + 1} / {len(path)}"
        )


    # ========================================================
    # MOVE ROBOT
    # ========================================================

    def move_robot(self):

        global robot_command_running

        if robot_command_running:

            return

        thread = threading.Thread(
            target=self._move_robot_thread,
            daemon=True
        )

        thread.start()


    def _move_robot_thread(self):

        global robot_command_running

        robot_command_running = True

        try:

            with state_lock:

                index = current_index

            target = get_target_pose(
                index
            )

            p = path[index]

            log(
                f"Moving robot to point "
                f"{index + 1}"
            )

            log(
                f"TARGET XYZ: "
                f"{target[0]:.6f}, "
                f"{target[1]:.6f}, "
                f"{target[2]:.6f}"
            )

            log(
                f"TARGET RV: "
                f"{target[3]:.6f}, "
                f"{target[4]:.6f}, "
                f"{target[5]:.6f}"
            )


            # ------------------------------------------------
            # IMPORTANT:
            #
            # This is a DIRECT MoveL to the exact point.
            #
            # No interpolation.
            # No SLERP.
            # No servoL.
            # ------------------------------------------------

            printhead.moveL(
                target,
                MOVE_SPEED,
                MOVE_ACCEL
            )


            # Give controller a moment to settle

            time.sleep(
                0.1
            )


            actual = get_actual_pose()


            ex = (
                actual[0]
                - target[0]
            )

            ey = (
                actual[1]
                - target[1]
            )

            ez = (
                actual[2]
                - target[2]
            )


            log(
                f"ACTUAL XYZ: "
                f"{actual[0]:.6f}, "
                f"{actual[1]:.6f}, "
                f"{actual[2]:.6f}"
            )


            log(
                f"ERROR XYZ: "
                f"{ex * 1000:.3f} mm, "
                f"{ey * 1000:.3f} mm, "
                f"{ez * 1000:.3f} mm"
            )


            self.root.after(
                0,
                lambda: self.status.config(
                    text=(
                        f"Point {index + 1} reached"
                    )
                )
            )


        except Exception as e:

            log(
                f"MoveL error: {e}"
            )

            self.root.after(
                0,
                lambda: self.status.config(
                    text=f"ERROR: {e}"
                )
            )


        finally:

            robot_command_running = False


    # ========================================================
    # STOP
    # ========================================================

    def stop_robot(self):

        try:

            printhead.stopL(
                2.0
            )

        except Exception as e:

            log(
                f"Stop error: {e}"
            )


# ============================================================
# START GUI
# ============================================================

def start_gui():

    root = tk.Tk()

    viewer = TrajectoryViewer(
        root
    )

    controls = ControlWindow()

    root.mainloop()


# ============================================================
# SHUTDOWN
# ============================================================

def shutdown():

    global stop_requested

    stop_requested = True

    log(
        "Stopping robot..."
    )

    try:

        printhead.stopL(
            2.0
        )

    except:
        pass

    try:

        printhead.servoStop()

    except:
        pass

    log(
        "Robot stopped."
    )


def signal_handler(
        sig,
        frame
):

    shutdown()

    sys.exit(0)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    signal.signal(
        signal.SIGINT,
        signal_handler
    )

    log(
        f"Loading {GCODE_FILE}"
    )

    path = parse_gcode(
        GCODE_FILE
    )

    log(
        f"Loaded {len(path)} points"
    )

    if not path:

        print(
            "No trajectory points found."
        )

        sys.exit(1)

    start_gui()