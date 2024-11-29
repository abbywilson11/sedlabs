import math # import math library for trig: sqrt(), math.degrees()

"""    
    Parameters:
    x (float): The x-coordinate of the end-effector.
    y (float): The y-coordinate of the end-effector.
    l1 (float): The length of the first arm segment.
    l2 (float): The length of the second arm segment.
    
    Returns:
    tuple: A tuple containing the angles alpha and beta in degrees.
    """

def inverse_kinematics(x, y, l1, l2):
    # Calculate the angle beta using the cosine rule
    cos_beta = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    beta = math.acos(cos_beta) # find beta in radians 

    # Calculate the angle alpha using trig
    k1 = l1 + l2 * math.cos(beta)
    k2 = l2 * math.sin(beta)
    alpha = math.atan2(y, x) - math.atan2(k2, k1) # atan finds alpha in radians

    # Convert radians to degrees fpr noth angles 
    alpha_deg = math.degrees(alpha)
    beta_deg = math.degrees(beta)

    return alpha_deg, beta_deg

def calibration_setup(file_path): # read data file and store in a dictionary
    error_table = {}
    
    with open(file_path, 'r') as file: # open and read file
        for line in file:
            angle, error = map(float, line.split()) # split each line into angles and error values 
            error_table[angle] = error # store in dict
    
    return error_table

def send_compensated_angle(desired_angle, servo_name, error_table):
    # Find the closest lower and upper bounds in the error table
    lower_bound = max([angle for angle in error_table if angle <= desired_angle], default=None)
    upper_bound = min([angle for angle in error_table if angle >= desired_angle], default=None)

    if lower_bound is None or upper_bound is None:
        raise ValueError("Desired angle is out of calibration range.")

    # if exact match is found in the table, use that error directly
    if lower_bound == upper_bound:
        error = error_table[lower_bound]
    else: # interpolate between lower and upper if needed
        lower_error = error_table[lower_bound]
        upper_error = error_table[upper_bound]
        ratio = (desired_angle - lower_bound) / (upper_bound - lower_bound)
        error = lower_error + ratio * (upper_error - lower_error)

    compensated_angle = desired_angle + error # compnesated angle = + interpolated (approx) error

    # Send this compensated angle to the servo (pseudo-code)
    # servo.send(servo_name, compensated_angle)

# Example usage:
# error_table = calibration_setup('calibration_data_X.txt')
# send_compensated_angle(90, 'shoulder', error_table)


# Example usage (Uncomment for testing)
# alpha, beta = inverse_kinematics(50, 50)
# print(f"Alpha: {alpha}, Beta: {beta}")
# calibrate_angles("test_jig_01")
# errors = calibration_setup("test_jig_01")
# send_compensated_angle(45, errors, 'shoulder')