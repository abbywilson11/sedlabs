import math

# Part 1: Inverse Kinematics Function
def inverse_kinematics(x, y, l1=100, l2=100):
    """
    Calculates the joint angles (alpha and beta) for a 2-link robot arm.
    x: Target x-coordinate of the end effector
    y: Target y-coordinate of the end effector
    l1: Length of the first link (default 100 mm)
    l2: Length of the second link (default 100 mm)
    :return: Tuple (alpha, beta) in degrees
    """
    try:
        # Calculate the distance from the base to the target point
        d = math.sqrt(x**2 + y**2)
        if d > (l1 + l2):
            # Check if the target is out of reach
            raise ValueError("Target point is out of reach.")

        # Compute the cosine of the angle beta using the law of cosines
        cos_beta = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        # Calculate beta using arccos
        beta = math.acos(cos_beta)

        # Intermediate values for alpha calculation
        k1 = l1 + l2 * math.cos(beta)
        k2 = l2 * math.sin(beta)

        # Calculate alpha using trigonometric relationships
        alpha = math.atan2(y, x) - math.atan2(k2, k1)

        # Convert alpha and beta from radians to degrees
        alpha_deg = math.degrees(alpha)
        beta_deg = math.degrees(beta)

        return alpha_deg, beta_deg

    except Exception as e:
        # Handle errors and print the issue
        print(f"Error in inverse kinematics calculation: {e}")
        return None


# Part 2: Calibration Program
def calibrate_angles(jig_id):
    """
    Generates calibration data for servo motors by sending angles and recording errors.
    :param jig_id: Unique ID for the test jig
    """
    # Define the name of the calibration file
    calibration_file = f"calibration_data_{jig_id}.txt"

    # Open the file for writing calibration data
    with open(calibration_file, 'w') as f:
        # Write the header row for the file
        f.write("Desired_Angle,Actual_Angle_Shoulder,Actual_Angle_Elbow\n")

        # Iterate over desired angles from 0 to 180 degrees in 10-degree steps
        for angle in range(0, 181, 10):
            print(f"Send desired angle: {angle}")

            # Prompt user to input the actual angles measured for each servo
            actual_shoulder = float(input(f"Enter measured angle for shoulder (desired {angle}): "))
            actual_elbow = float(input(f"Enter measured angle for elbow (desired {angle}): "))

            # Log the desired and actual angles to the file
            f.write(f"{angle},{actual_shoulder},{actual_elbow}\n")

    # Notify user that calibration data has been saved
    print(f"Calibration data saved to {calibration_file}")


# Part 3: Calibration Functions
def calibration_setup(jig_id):
    """
    Reads the calibration file and returns a table of errors.
    :param jig_id: Unique ID for the test jig
    :return: Dictionary with desired angles as keys and errors as values
    """
    # Define the calibration file name
    calibration_file = f"calibration_data_{jig_id}.txt"
    error_table = {}

    # Open the calibration file and read the data
    with open(calibration_file, 'r') as f:
        next(f)  # Skip the header line
        for line in f:
            # Parse the desired and actual angles from the file
            desired, actual_shoulder, actual_elbow = map(float, line.strip().split(','))
            # Calculate errors for both servos and store in the table
            error_table[desired] = {
                'shoulder_error': actual_shoulder - desired,
                'elbow_error': actual_elbow - desired
            }

    return error_table


def send_compensated_angle(desired_angle, error_table, servo):
# Sends an angle to a servo after compensating for errors.
    # Sort the keys of the error table for interpolation
    keys = sorted(error_table.keys())

    # Find the range of keys surrounding the desired angle
    for i in range(len(keys) - 1):
        if keys[i] <= desired_angle <= keys[i + 1]:
            # Lower and upper bounds for interpolation
            low = keys[i]
            high = keys[i + 1]

            # Errors at the lower and upper bounds
            error_low = error_table[low][f"{servo}_error"]
            error_high = error_table[high][f"{servo}_error"]

            # Interpolate the error for the desired angle
            interpolated_error = error_low + (desired_angle - low) * (error_high - error_low) / (high - low)

            # Calculate the compensated angle by adding the interpolated error
            compensated_angle = desired_angle + interpolated_error
            print(f"Sending compensated angle: {compensated_angle} to {servo} servo")
            return compensated_angle

    # Return the desired angle if no interpolation is needed
    return desired_angle
