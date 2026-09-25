import numpy as np
import re


# =========================================================
# Quaternion utilities
# =========================================================

def normalize_quaternion(
        quat_w,
        quat_x,
        quat_y,
        quat_z
):

    q = np.array(
        [
            quat_w,
            quat_x,
            quat_y,
            quat_z
        ],
        dtype=float
    )

    length = np.linalg.norm(q)

    if length < 1e-12:
        return np.array(
            [1.0, 0.0, 0.0, 0.0],
            dtype=float
        )

    return q / length


# =========================================================
# Quaternion -> UR rotation vector
#
# KEPT HERE because the conversion is still useful.
#
# We are NOT using it for the exported trajectory yet.
# The control code will use it after quaternion SLERP.
# =========================================================

def quaternion_to_rotvec(
        quat_w,
        quat_x,
        quat_y,
        quat_z
):

    q = normalize_quaternion(
        quat_w,
        quat_x,
        quat_y,
        quat_z
    )

    # -----------------------------------------------------
    # q and -q represent the same rotation.
    #
    # Force w >= 0 so that the rotation angle is in
    # the range 0 ... pi.
    # -----------------------------------------------------

    if q[0] < 0.0:
        q = -q

    w = np.clip(
        q[0],
        -1.0,
        1.0
    )

    x = q[1]
    y = q[2]
    z = q[3]

    # -----------------------------------------------------
    # Rotation angle
    # -----------------------------------------------------

    angle = 2.0 * np.arccos(w)

    sin_half_angle = np.sqrt(
        max(
            0.0,
            1.0 - w * w
        )
    )

    # -----------------------------------------------------
    # Very small rotation
    # -----------------------------------------------------

    if sin_half_angle < 1e-12:
        return np.zeros(3)

    # -----------------------------------------------------
    # Rotation axis
    # -----------------------------------------------------

    axis = np.array(
        [
            x,
            y,
            z
        ],
        dtype=float
    ) / sin_half_angle

    # -----------------------------------------------------
    # UR rotation vector
    # -----------------------------------------------------

    return axis * angle


# =========================================================
# Extract value from G-code token
# =========================================================

def get_value(
        line,
        name
):

    number = (
        r"[-+]?"
        r"(?:"
        r"\d+(?:\.\d*)?"
        r"|"
        r"\.\d+"
        r")"
        r"(?:[eE][-+]?\d+)?"
    )

    pattern = (
        rf"\b{re.escape(name)}"
        rf"({number})"
    )

    match = re.search(
        pattern,
        line,
        re.IGNORECASE
    )

    if match is None:
        return None

    return float(
        match.group(1)
    )


# =========================================================
# Parse G-code
#
# Orientation is kept as QUATERNION.
#
# No quaternion -> rotation-vector conversion happens here.
# =========================================================

def parse_gcode(
        filename
):

    poses = []

    skipped_no_quaternion = 0
    skipped_invalid = 0

    with open(
        filename,
        "r"
    ) as file:

        for line_number, line in enumerate(
            file,
            start=1
        ):

            # -------------------------------------------------
            # Remove comments
            # -------------------------------------------------

            line = line.split(";")[0].strip()

            if not line:
                continue

            # -------------------------------------------------
            # Only G1 commands
            # -------------------------------------------------

            if not re.search(
                r"\bG1\b",
                line,
                re.IGNORECASE
            ):
                continue

            # -------------------------------------------------
            # Position
            # -------------------------------------------------

            x = get_value(
                line,
                "X"
            )

            y = get_value(
                line,
                "Y"
            )

            z = get_value(
                line,
                "Z"
            )

            # -------------------------------------------------
            # Quaternion
            # -------------------------------------------------

            qw = get_value(
                line,
                "quat_w"
            )

            qx = get_value(
                line,
                "quat_x"
            )

            qy = get_value(
                line,
                "quat_y"
            )

            qz = get_value(
                line,
                "quat_z"
            )

            # -------------------------------------------------
            # Feed
            # -------------------------------------------------

            feed = get_value(
                line,
                "F"
            )

            # -------------------------------------------------
            # Optional values
            # -------------------------------------------------

            ec = get_value(
                line,
                "EC"
            )

            pod = get_value(
                line,
                "PoD"
            )

            if pod is None:

                pod = get_value(
                    line,
                    "POD"
                )

            if pod is None:

                pod = get_value(
                    line,
                    "PD"
                )

            # -------------------------------------------------
            # Quaternion is mandatory
            # -------------------------------------------------

            if None in (
                qw,
                qx,
                qy,
                qz
            ):

                skipped_no_quaternion += 1

                continue

            # -------------------------------------------------
            # Position and feed are mandatory
            # -------------------------------------------------

            if None in (
                x,
                y,
                z,
                feed
            ):

                skipped_invalid += 1

                continue

            # -------------------------------------------------
            # Normalize quaternion now.
            #
            # This means the output file always contains
            # a valid unit quaternion.
            # -------------------------------------------------

            q = normalize_quaternion(
                qw,
                qx,
                qy,
                qz
            )

            qw = q[0]
            qx = q[1]
            qy = q[2]
            qz = q[3]

            # -------------------------------------------------
            # Defaults for optional values
            # -------------------------------------------------

            if ec is None:
                ec = 0.0

            if pod is None:
                pod = 0.0

            # -------------------------------------------------
            # Store pose
            # -------------------------------------------------

            poses.append(
                (
                    x,
                    y,
                    z,

                    qw,
                    qx,
                    qy,
                    qz,

                    ec,
                    feed,
                    pod
                )
            )

    print(
        "Parsed poses:",
        len(poses)
    )

    print(
        "Skipped lines without quaternion:",
        skipped_no_quaternion
    )

    print(
        "Skipped invalid lines:",
        skipped_invalid
    )

    return poses


# =========================================================
# Export reevaluated G-code
#
# IMPORTANT:
#
# The quaternion is exported directly.
#
# We do NOT convert it to RX/RY/RZ here.
# =========================================================

def export_corrected(
        poses,
        filename="reevaluated_output_quat.nc"
):

    with open(
        filename,
        "w"
    ) as file:

        for pose in poses:

            (
                x,
                y,
                z,

                qw,
                qx,
                qy,
                qz,

                ec,
                feed,
                pod

            ) = pose

            # -------------------------------------------------
            # Write G-code
            #
            # Quaternion remains the orientation representation.
            # -------------------------------------------------

            file.write(

                f"G1 "

                f"X{x:.3f} "

                f"Y{y:.3f} "

                f"Z{z:.3f} "

                f"QW{qw:.9f} "

                f"QX{qx:.9f} "

                f"QY{qy:.9f} "

                f"QZ{qz:.9f} "

                f"EC{ec:.6f} "

                f"F{feed:.1f} "

                f"PD{pod:.6f}\n"
            )

    print(
        "Saved:",
        filename
    )


# =========================================================
# MAIN
# =========================================================

if __name__ == "__main__":

    input_file = (
        "UR_Posititoner_Bottle_Cage_7.nc"
    )

    output_file = (
        "reevaluated_output_quat.nc"
    )

    poses = parse_gcode(
        input_file
    )

    export_corrected(
        poses,
        output_file
    )