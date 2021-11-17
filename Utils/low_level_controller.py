import numpy as np
from get_avail_car_index import TIME_INTERVAL

def vehicle_following_controller(interdistance, desired_h, vel, previous_h):
    k_car = 4
    k_follow = 1.5

    current_h = interdistance / vel
    delta_a = k_car * (current_h - previous_h) + k_follow * (current_h - desired_h) * TIME_INTERVAL / 1000
    print("error:", current_h - desired_h)
    return delta_a, current_h


def speed_tacking_controller(vel, previous_vel, desired_vel):
    k_I = 0.4
    k_n = 0.5
    delta_a = k_n * (previous_vel - vel) + k_I * (desired_vel - vel)
    return delta_a


def lane_keeping_controller(Car_X, Car_Y, look_ahead_dis, heading_angle, previous_theta, Car_line, X1, Y1,
                            previous_theta_f, centerline):
    kn = 0.9
    KI = 0.7
    kf = 0.05
    rotational_matrix_X = np.array([np.cos(-heading_angle), -np.sin(-heading_angle)])
    rotational_matrix_Y = np.array([np.sin(-heading_angle), np.cos(-heading_angle)])

    centerline_x = centerline[Car_line][:, 0]
    centerline_y = centerline[Car_line][:, 1]


    X_0 = Car_X + look_ahead_dis * np.cos(heading_angle)
    Y_0 = Car_Y + look_ahead_dis * np.sin(heading_angle)

    distance_0 = 1000
    X_1 = 0
    Y_1 = 0

    index_1 = np.argwhere((centerline_x < X_0 + 130) & (centerline_x > X_0 - 130))
    centerline_x1 = centerline_x[index_1].flatten()
    centerline_y1 = centerline_y[index_1].flatten()
    for ix in np.arange(X_0 - 100, X_0 + 100, 0.1):
        iy = np.interp(ix, centerline_x1, centerline_y1)
        distance_1 = np.sqrt((X_0 - ix) ** 2 + (Y_0 - iy) ** 2)
        if distance_1 < distance_0:
            distance_0 = distance_1
            X_1 = ix
            Y_1 = iy

    X_new = np.dot(rotational_matrix_X, np.array([X_1 - Car_X, Y_1 - Car_Y]))
    Y_new = np.dot(rotational_matrix_Y, np.array([X_1 - Car_X, Y_1 - Car_Y]))

    X_front = np.dot(rotational_matrix_X, np.array([X1 - Car_X, Y1 - Car_Y]))
    Y_front = np.dot(rotational_matrix_Y, np.array([X1 - Car_X, Y1 - Car_Y]))
    theta_front = np.arctan(Y_front / X_front)

    theta = np.arctan(Y_new / X_new)

    print("angle:", theta)
    change_theta = theta - previous_theta
    change_theta_f = theta_front - previous_theta_f
    delta_phi = kf * (np.clip(change_theta_f, -0.007, 0.007)) + kn * (
        np.clip(change_theta, -0.007, 0.007)) + KI * np.clip(theta, -0.06, 0.06) * TIME_INTERVAL / 1000
    print("change_theta_f, change_theta, theta:", change_theta_f, change_theta, theta)

    return delta_phi, theta, theta_front, X_1, Y_1



def get_lane_sequence(lane_change_delta, lane_change_index, choose_1, total_length, car_interested):
    lane_id = [car_interested.lane[0]]
    lane_index = [0]

    for i in range(total_length):
        if car_interested.lane[i] != lane_id[-1]:
            lane_id.append(car_interested.lane[i])
            lane_index.append(i)

    print("The time instances of the interested car are", lane_index)

    lane_index = np.array(lane_index)

    if choose_1 == 1:
        lane_index += np.array(lane_change_delta)[:len(lane_index)]
    else:
        lane_index = np.array(lane_change_index)[:len(lane_index)]

    print("The time instances of the tested car are", lane_index)
    print("The lane ID of the interested car are", lane_index)

    lane_index = lane_index.tolist()

    lane_index.append(total_length)

    lane_change_sequence = np.zeros(total_length)

    for j in range(len(lane_index) - 1):
        lane_change_sequence[lane_index[j]:lane_index[j + 1]] = lane_id[j]

    return lane_change_sequence