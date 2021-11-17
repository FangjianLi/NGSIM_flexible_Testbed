import numpy as np
from Utils.car_type import car_generator, test_car_c
from Utils.utils import get_lane_ID, get_relevant_cars
from data_processing import data_processing, process_the_map
from data_processing import INTERESTED_CAR_ID
from get_avail_car_index import TIME_INTERVAL
from Utils.vehicle_dynamics import vehicle_dynamic_model
from Utils.low_level_controller import vehicle_following_controller, speed_tacking_controller, lane_keeping_controller, \
    get_lane_sequence
from Utils.animation_tools import NGSIM_animation_generator



# This is the hyperparameter of the lane-changing sequence
DELTA_1 = -4
DELTA_2 = -5
DELTA_3 = -5

lane_change_delta_1 = [0, DELTA_1, DELTA_2, DELTA_3]

INSTANCE_1 = 42
INSTANCE_2 = 56
INSTANCE_3 = 79

lane_change_instance_1 = [0, INSTANCE_1, INSTANCE_2, INSTANCE_3]

# Get the data
data_1_A, environment_vehicle_ID, total_length, min_total_time, max_total_time = data_processing()

centerline, lanes = process_the_map()

# Generate the cars
car_interested = car_generator(INTERESTED_CAR_ID, data_1_A, total_length, min_total_time, max_total_time, TIME_INTERVAL)
car_interested_pre = car_generator(int(car_interested.preceding[0]), data_1_A, total_length, min_total_time,
                                   max_total_time, TIME_INTERVAL)
car_tested = test_car_c(car_interested, car_interested_pre, total_length, TIME_INTERVAL)
car_env_list = []

for index in environment_vehicle_ID:
    car_env = car_generator(index, data_1_A, total_length, min_total_time, max_total_time, TIME_INTERVAL)
    car_env_list.append(car_env)

# Here we perform the low level controller
lane_intervals = get_lane_sequence(lane_change_delta_1, lane_change_instance_1, 0, total_length, car_interested)

print(lane_intervals)

# same lane searching
front_vehicle = np.zeros(total_length)
X_f = np.zeros(total_length)
Y_f = np.zeros(total_length)
S_f_distance = np.zeros(total_length)

# left lane searching
left_front_vehicle = np.zeros(total_length)
X_l_f = np.zeros(total_length)
Y_l_f = np.zeros(total_length)
L_f_distance = np.zeros(total_length)

left_rear_vehicle = np.zeros(total_length)
X_l_r = np.zeros(total_length)
Y_l_r = np.zeros(total_length)
L_r_distance = np.zeros(total_length)

# right lane searching
right_front_vehicle = np.zeros(total_length)
X_r_f = np.zeros(total_length)
Y_r_f = np.zeros(total_length)
R_f_distance = np.zeros(total_length)

right_rear_vehicle = np.zeros(total_length)
X_r_b = np.zeros(total_length)
Y_r_b = np.zeros(total_length)
R_b_distance = np.zeros(total_length)

# the initial value of the test car


front_vehicle_2 = np.zeros(total_length)
X_f_2 = np.zeros(total_length)
Y_f_2 = np.zeros(total_length)

# check the cose

near_point_X = np.zeros(total_length)
near_point_Y = np.zeros(total_length)

#
lane_ID_list = np.zeros(total_length)

for i in range(total_length):

    print(i)

    subject_car_X = car_interested.X[i]
    subject_car_Y = car_interested.Y[i]
    subject_car_Lane = car_interested.lane[i]

    S_F_car1, L_F_car1, L_B_car1, R_F_car1, R_B_car1 = get_relevant_cars(i, subject_car_X, subject_car_Y,
                                                                         subject_car_Lane, environment_vehicle_ID,
                                                                         car_env_list)

    front_vehicle[i] = S_F_car1.ID
    X_f[i] = S_F_car1.X_r
    Y_f[i] = S_F_car1.Y_r
    S_f_distance[i] = S_F_car1.Distance

    left_front_vehicle[i] = L_F_car1.ID
    X_l_f[i] = L_F_car1.X_r
    Y_l_f[i] = L_F_car1.Y_r
    L_f_distance[i] = L_F_car1.Distance

    left_rear_vehicle[i] = L_B_car1.ID
    X_l_r[i] = L_B_car1.X_r
    Y_l_r[i] = L_B_car1.Y_r
    L_r_distance[i] = L_B_car1.Distance

    right_front_vehicle[i] = R_F_car1.ID
    X_r_f[i] = R_F_car1.X_r
    Y_r_f[i] = R_F_car1.Y_r
    R_f_distance[i] = R_F_car1.Distance

    right_rear_vehicle[i] = R_B_car1.ID
    X_r_b[i] = R_B_car1.X_r
    Y_r_b[i] = R_B_car1.Y_r
    R_b_distance[i] = R_B_car1.Distance

    lane_ID_list[i] = get_lane_ID(car_tested.X[i], car_tested.Y[i], lanes)

    S_F_car2, L_F_car2, L_B_car2, R_F_car2, R_B_car2 = get_relevant_cars(i, car_tested.X[i], car_tested.Y[i],
                                                                         lane_intervals[i], environment_vehicle_ID,
                                                                         car_env_list)

    front_vehicle_2[i] = S_F_car2.ID
    X_f_2[i] = S_F_car2.X_r
    Y_f_2[i] = S_F_car2.Y_r

    interdistance0 = np.sqrt((X_f_2[i] - car_tested.X[i]) ** 2 + (Y_f_2[i] - car_tested.Y[i]) ** 2)

    print(interdistance0)

    delta_a, h_1 = vehicle_following_controller(interdistance0, 1.05, car_tested.vel[i], car_tested.h[i])

    if (interdistance0 > 5 * car_tested.vel[i]) | (front_vehicle_2[i] == 0):
        delta_a = speed_tacking_controller(car_tested.vel[i], car_tested.vel[i - 1], 20)
        h_1 = 5

    car_tested.a[i + 1] = car_tested.a[i] + delta_a
    car_tested.vel[i + 1] = car_tested.vel[i] + car_tested.a[i] * TIME_INTERVAL / 1000
    car_tested.h[i + 1] = h_1

    delta_phi, theta_1, theta_f, X1, Y1 = lane_keeping_controller(car_tested.X[i], car_tested.Y[i], 10,
                                                                  car_tested.PHI[i], car_tested.theta[i],
                                                                  lane_intervals[i],
                                                                  X_f_2[i], Y_f_2[i], car_tested.theta_f[i], centerline)

    car_tested.phi[i + 1] = car_tested.phi[i] + delta_phi
    car_tested.theta[i + 1] = theta_1
    car_tested.theta_f[i + 1] = theta_f
    near_point_X[i] = X1
    near_point_Y[i] = Y1

    delta_X, delta_Y, r_a, beta_a, PHI_a = vehicle_dynamic_model(car_tested.beta[i], car_tested.vel[i],
                                                                 car_tested.phi[i],
                                                                 car_tested.r[i], car_tested.PHI[i])

    car_tested.X[i + 1] = car_tested.X[i] + delta_X
    car_tested.Y[i + 1] = car_tested.Y[i] + delta_Y
    car_tested.r[i + 1] = r_a
    car_tested.beta[i + 1] = beta_a
    car_tested.PHI[i + 1] = PHI_a


dict_present_car = {"right_front_vehicle": right_front_vehicle, "right_rear_vehicle": right_rear_vehicle,
                    "left_front_vehicle": left_front_vehicle,
                    "front_vehicle": front_vehicle, "left_rear_vehicle": left_rear_vehicle}

dict_X_Y_point = {"X_f": X_f, "Y_f": Y_f, "X_l_f": X_l_f, "Y_l_f": Y_l_f, "X_l_r": X_l_r,
                  "Y_l_r": Y_l_r, "X_r_f": X_r_f, "Y_r_f": Y_r_f, "X_r_b": X_r_b, "Y_r_b": Y_r_b,
                  "near_point_X": near_point_X, "near_point_Y": near_point_Y}


animation_generator = NGSIM_animation_generator(car_interested, car_tested, car_env_list, centerline, total_length,  lanes, dict_present_car, dict_X_Y_point)
animation_generator.show_animation()