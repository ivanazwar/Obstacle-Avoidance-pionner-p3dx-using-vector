import sys
import time
import sim
import keyboard
import math
import numpy as np

# Function to get disc positions
def connect_to_coppeliasim():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to remote API server')
        return clientID
    else:
        print('Failed to connect to remote API server')
        return None

def get_disc_positions(clientID, disc_name):
    res, disc_handle = sim.simxGetObjectHandle(clientID, "/" + disc_name, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        res, disc_position = sim.simxGetObjectPosition(clientID, disc_handle, -1, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Disc position for', disc_name, ':', disc_position)
            return disc_position
        else:
            print('Failed to get disc position for', disc_name)
            return None
    else:
        print('Failed to get disc handle for', disc_name)
        return None

# Function to get disc orientations
def get_disc_orientations(clientID, disc_name):
    res, disc_handle = sim.simxGetObjectHandle(clientID, "/" + disc_name, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        res, disc_orientation = sim.simxGetObjectOrientation(clientID, disc_handle, -1, sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print('Disc orientation for', disc_name, ':', disc_orientation)
            return disc_orientation
        else:
            print('Failed to get disc orientation for', disc_name)
            return None
    else:
        print('Failed to get disc handle for', disc_name)
        return None

# mencari vektor robot terhadap obstacle
def vektor_sum(sensor_readings, threshold_distances):
    polar_vectors = []
    num_sensors = len(sensor_readings)
    for i, (distance, threshold_distance) in enumerate(zip(sensor_readings, threshold_distances)):
        if distance < threshold_distance:
            angle = -90 + (180 / (num_sensors - 1)) * i
            polar_vectors.append((distance, angle))
    return polar_vectors

def adjust_vector_density(polar_vectors, case):
    if case == '1':
        return polar_vectors[::3]
    elif case == '2':
        return polar_vectors[::2]
    elif case == '3':
        return polar_vectors

def dynamic_obstacle_avoidance(polar_vectors, motors_velocity, threshold_distances, is_dynamic):
    segments = {i: [] for i in range(8)}
    segment_angles = [-90 + i * 22.5 for i in range(8)]

    for vector in polar_vectors:
        angle = vector[1]
        for i, segment_angle in enumerate(segment_angles):
            if segment_angle - 11.25 <= angle < segment_angle + 11.25:
                segments[i].append(vector)
                break

    for i in range(8):
        if segments[i]:
            min_distance = min(segments[i])[0]
            corresponding_threshold = threshold_distances[i % len(threshold_distances)]
            if min_distance < corresponding_threshold:
                if 1 <= i <= 3:  # Left side segments
                    return (-0.6 * motors_velocity[0], 0.6 * motors_velocity[1])
                elif 4 <= i <= 6:  # Right side segments
                    return (0.6 * motors_velocity[0], -0.6 * motors_velocity[1])
                if is_dynamic:
                    if i == 0 or i == 7:  # Front segments
                        return (0.7 * motors_velocity[0], -0.7 * motors_velocity[1])
    return motors_velocity

def robot_model(x, u):
    L = 0.5
    R = 0.5
    theta = x[2]
    v = u[0]
    omega = u[1]
    x_next = np.zeros(3)
    x_next[0] = x[0] + v * np.cos(theta)
    x_next[1] = x[1] + v * np.sin(theta)
    x_next[2] = x[2] + omega
    return x_next



def get_robot_handle(clientID):
    res, robot_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        print('Pioneer P3DX handle:', robot_handle)
        return robot_handle
    else:
        print('Failed to get Pioneer P3DX handle')
        return None

def get_motor_handles(clientID, p3dx_handle):
    res, left_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)
    res, right_handle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        print('Pioneer P3DX motor handles obtained')
        motor_handles = (right_handle, left_handle)
        return motor_handles
    else:
        print('Failed to get Pioneer P3DX motor handles')
        return None

def get_robot_position(clientID, robot_handle):
    res, robot_position = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        return robot_position
    else:
        print('Failed to get robot position')
        return None

def get_robot_orientation(clientID, robot_handle):
    res, robot_orientation = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        return robot_orientation
    else:
        print('Failed to get robot orientation')
        return None

def get_sensor_readings(clientID, sensors):
    readings = []
    for sensor in sensors:
        res, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_streaming)
        if res == sim.simx_return_ok and detectionState:
            distance = np.linalg.norm(detectedPoint)
            readings.append(distance)
        else:
            readings.append(float('inf'))
    return readings

def set_robot_motion(clientID, motor_handles, motors_velocity):
    _ = sim.simxSetJointTargetVelocity(clientID, motor_handles[0], motors_velocity[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motor_handles[1], motors_velocity[1], sim.simx_opmode_oneshot)

def stop_robot(clientID, motor_handles):
    _ = sim.simxSetJointTargetVelocity(clientID, motor_handles[0], 0, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motor_handles[1], 0, sim.simx_opmode_oneshot)

# Function to select local sub-target
def select_local_sub_target(robot_position, target_position, obstacles):
    # Implement logic to select the best local sub-target based on the description in the paper
    sub_target_x = (robot_position[0] + target_position[0]) / 2
    sub_target_y = (robot_position[1] + target_position[1]) / 2
    return (sub_target_x, sub_target_y)


def adjust_motor_speed_based_on_environment(motors_velocity, case):
    if case == '1':
        return (1.0 * motors_velocity[0], 1.0 * motors_velocity[1])
    elif case == '2':
        return (0.75 * motors_velocity[0], 0.75 * motors_velocity[1])
    elif case == '3':
        return (0.5 * motors_velocity[0], 0.5 * motors_velocity[1])
    return motors_velocity


clientID = connect_to_coppeliasim()
if clientID is None:
    sys.exit()

robot_handle = get_robot_handle(clientID)
if robot_handle is None:
    sys.exit()

motor_handles = get_motor_handles(clientID, robot_handle)
if motor_handles is None:
    sys.exit()


sensor_handles = []
for i in range(16):
    res, sensor_handle = sim.simxGetObjectHandle(clientID, f'/PioneerP3DX/ultrasonicSensor[{i}]', sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        sensor_handles.append(sensor_handle)
    else:
        print(f'Failed to get handle for sensor {i}')
        sys.exit()


front_sensors = sensor_handles[0:8]


#for i, sensor in enumerate(front_sensors):
#    print(f"Sensor {i}: Handle ID {sensor}")


disc_name = 'Disc'
velo_init = (0.0, 0.0)
samp_time, n = 0.1, 1.0
time_start = time.time()
position_tol = 0.2
orientation_tol = 0.3
k_1 = 0.3#0.5#0.6
k_2 = 0.5#0.5#0.4
k_3 = 0.6#0.5#0.5
R = 0.5
L = 0.5
vnorm = 8.0  # vMax
counter = 0

# jarak robot terhadap obstacle
threshold_distances = [0.1, 0.3, 0.4, 0.6, 0.6, 0.4, 0.3, 0.1]
#threshold_distances = [0.3, 0.3, 0.6, 0.5, 0.5, 0.6, 0.3, 0.3]
#threshold_distances = [0.7, 0.7, 0.7, 0.9, 0.9, 0.7, 0.7, 0.7]

def main_loop():
    global clientID, robot_handle, motor_handles, front_sensors, disc_name, velo_init, samp_time, n, time_start, position_tol, orientation_tol, k_1, k_2, k_3, R, L, vnorm, counter, threshold_distances

    # Initialize robot position and orientation
    robot_position = get_robot_position(clientID, robot_handle)
    robot_orientation = get_robot_orientation(clientID, robot_handle)
    x_act = robot_position[0]
    y_act = robot_position[1]
    g_act = robot_orientation[2]

    while True:
        t_now = time.time() - time_start
        if t_now >= samp_time * n:
            robot_position = get_robot_position(clientID, robot_handle)
            robot_orientation = get_robot_orientation(clientID, robot_handle)

            if robot_position is None or robot_orientation is None:
                raise Exception("Lost connection to robot")

            disc_position = get_disc_positions(clientID, disc_name)
            disc_orientation = get_disc_orientations(clientID, disc_name)

            if disc_position is None or disc_orientation is None:
                raise Exception("Lost connection to disc")

            x_act = robot_position[0]
            y_act = robot_position[1]
            g_act = robot_orientation[2]

            x_ref = disc_position[0]
            y_ref = disc_position[1]
            g_ref = disc_orientation[2]

            e_x = x_ref - x_act
            e_y = y_ref - y_act
            e_g = g_ref - g_act
            theta = math.atan2(e_y, e_x)

            print(f'ref: x={x_ref}, y={y_ref}, g={g_ref}')
            print(f'Errors: e_x={e_x}, e_y={e_y}, e_g={e_g}')

            if abs(e_x) <= position_tol and abs(e_y) <= position_tol:
                x_c, y_c = 0, 0
                if abs(e_g) >= orientation_tol:
                    g_c = k_3 * e_g
                    if abs(e_g) <= orientation_tol:
                        g_c = 0
                else:
                    stop_robot(clientID, motor_handles)
                    time.sleep(5)
                    break
            else:
                x_c = k_1 * e_x
                y_c = k_2 * e_y
                g_c = theta - g_act

            matrix_A = np.array([
                [(R / 2) * math.cos(theta), (R / 2) * math.cos(theta)],
                [(R / 2) * math.sin(theta), (R / 2) * math.sin(theta)],
                [R / (2 * L), -R / (2 * L)]
            ])

            matrix_B = np.array([
                [x_c],
                [y_c],
                [g_c]
            ])

            inverse_A = np.linalg.pinv(matrix_A)
            velocity_gen = np.dot(inverse_A, matrix_B)
            velocity_max = max(abs(velocity_gen[1]), abs(velocity_gen[0]))
            if velocity_max > vnorm:
                velocity_rn = (vnorm / velocity_max) * velocity_gen[0]
                velocity_ln = (vnorm / velocity_max) * velocity_gen[1]
            else:
                velocity_rn, velocity_ln = velocity_gen[0], velocity_gen[1]

            motors_velocity = (velocity_rn[0], velocity_ln[0])

            # Obstacle avoidance
            sensor_readings = get_sensor_readings(clientID, front_sensors)

           # print("Sensor Readings:")
            #for i, reading in enumerate(sensor_readings):
                #print(f"Sensor {i}: {reading}")

            polar_vectors = vektor_sum(sensor_readings, threshold_distances)
            num_arah = len(polar_vectors)

            for i, vector in enumerate(polar_vectors):
                print(f'Vektor heading {i+10}: deltaS = {vector[0]}, theta = {vector[1]}')

            case = '3' if len(polar_vectors) > 3 else '2' if len(polar_vectors) > 0 else '1'

            print(f'case: {case}')

            adjusted_vectors = adjust_vector_density(polar_vectors, case)
            motors_velocity = dynamic_obstacle_avoidance(adjusted_vectors, motors_velocity, threshold_distances, is_dynamic=True)

            motors_velocity = adjust_motor_speed_based_on_environment(motors_velocity, case)

            set_robot_motion(clientID, motor_handles, motors_velocity)

            n += 1.0
            if keyboard.is_pressed('esc'):
                break

try:
    while True:
        main_loop()
except Exception as e:
    print(f"Exception occurred: {e}")
    print("Attempting to reconnect...")
    while True:
        clientID = connect_to_coppeliasim()
        if clientID is not None:
            robot_handle = get_robot_handle(clientID)
            if robot_handle is not None:
                motor_handles = get_motor_handles(clientID, robot_handle)
                if motor_handles is not None:
                    break
        time.sleep(1)

sim.simxFinish(clientID)
print('Connection closed')
