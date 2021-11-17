import numpy as np
from Utils.car_type import neighbor_car

def get_lane_ID(car_X, car_Y, lanes):
    x2 = np.flip(lanes[:, 18])
    y2 = np.flip(lanes[:, 19])
    lane_indexes_2 = np.argwhere((x2 < car_X + 50) & (x2 > car_X - 50))
    lane_2_X = x2[lane_indexes_2].flatten()
    lane_2_Y = y2[lane_indexes_2].flatten()
    Y_2 = np.interp(car_X, lane_2_X, lane_2_Y)

    x3 = np.flip(lanes[:, 16])
    y3 = np.flip(lanes[:, 17])

    lane_indexes_3 = np.argwhere((x3 < car_X + 50) & (x3 > car_X - 50))
    lane_3_X = x3[lane_indexes_3].flatten()
    lane_3_Y = y3[lane_indexes_3].flatten()
    Y_3 = np.interp(car_X, lane_3_X, lane_3_Y)

    x4 = np.flip(lanes[:, 14])
    y4 = np.flip(lanes[:, 15])

    lane_indexes_4 = np.argwhere((x4 < car_X + 50) & (x4 > car_X - 50))
    lane_4_X = x4[lane_indexes_4].flatten()
    lane_4_Y = y4[lane_indexes_4].flatten()
    Y_4 = np.interp(car_X, lane_4_X, lane_4_Y)

    if car_Y > Y_2:
        lane_ID = 1
    elif car_Y > Y_3:
        lane_ID = 2
    elif car_Y > Y_4:
        lane_ID = 3
    else:
        lane_ID = 10

    return lane_ID


def get_relevant_cars(i, Car_X, Car_Y, Car_Lane, environment_vehicle_ID, car_env_list):
    # define the interested cars

    S_F_car = neighbor_car()
    L_F_car = neighbor_car()
    L_B_car = neighbor_car()
    R_F_car = neighbor_car()
    R_B_car = neighbor_car()

    # same lane searching
    same_lane_index = []
    same_lane_distance = []
    x_f = []
    y_f = []

    # left lane searching
    left_lane_index = []
    x_l = []
    y_l = []
    left_lane_distance = []
    x_l_f = []
    y_l_f = []

    # right_lane_searching
    right_lane_index = []
    x_r = []
    y_r = []
    right_lane_distance = []
    x_r_f = []
    y_r_f = []

    trans_matrix = np.array([np.cos(np.radians(40)), -np.sin(np.radians(40))])

    for car_index, car_a in zip(environment_vehicle_ID, car_env_list):

        if car_a.lane[i] == Car_Lane:
            same_lane_index.append(car_index)
            dis_vector = np.array([car_a.X[i] - Car_X, car_a.Y[i] - Car_Y])
            dist_x = np.dot(trans_matrix, dis_vector)
            same_lane_distance.append(dist_x)
            x_f.append(car_a.X[i] - car_a.length * np.cos(np.radians(40)))
            y_f.append(car_a.Y[i] + car_a.length * np.sin(np.radians(40)))

        if car_a.lane[i] == Car_Lane - 1:
            left_lane_index.append(car_index)
            dis_vector_l = np.array([car_a.X[i] - Car_X, car_a.Y[i] - Car_Y])
            dist_x_l = np.dot(trans_matrix, dis_vector_l)
            left_lane_distance.append(dist_x_l)
            x_l.append(car_a.X[i] - car_a.length * np.cos(np.radians(40)))
            x_l_f.append(car_a.X[i])
            y_l.append(car_a.Y[i] + car_a.length * np.sin(np.radians(40)))
            y_l_f.append(car_a.Y[i])

        if car_a.lane[i] == Car_Lane + 1:
            right_lane_index.append(car_index)
            dis_vector_r = np.array([car_a.X[i] - Car_X, car_a.Y[i] - Car_Y])
            dist_x_r = np.dot(trans_matrix, dis_vector_r)
            right_lane_distance.append(dist_x_r)
            x_r.append(car_a.X[i] - car_a.length * np.cos(np.radians(40)))
            x_r_f.append(car_a.X[i])
            y_r.append(car_a.Y[i] + car_a.length * np.sin(np.radians(40)))
            y_r_f.append(car_a.Y[i])

    list_sl = list(j for j in same_lane_distance if j > 0)

    if list_sl:
        index_1 = np.argwhere(same_lane_distance == min(list_sl))
        S_F_car.ID = same_lane_index[np.asscalar(index_1[0])]
        S_F_car.X_r = x_f[np.asscalar(index_1[0])]
        S_F_car.Y_r = y_f[np.asscalar(index_1[0])]
        S_F_car.Distance = same_lane_distance[np.asscalar(index_1[0])]

    list_l_f = list(j for j in left_lane_distance if j > 0)

    if list_l_f:
        index_2 = np.argwhere(left_lane_distance == min(list_l_f))
        L_F_car.ID = left_lane_index[np.asscalar(index_2[0])]
        L_F_car.X_r = x_l[np.asscalar(index_2[0])]
        L_F_car.Y_r = y_l[np.asscalar(index_2[0])]
        L_F_car.Distance = left_lane_distance[np.asscalar(index_2[0])]

    list_l_r = list(j for j in left_lane_distance if j <= 0)

    if list_l_r:
        index_3 = np.argwhere(left_lane_distance == max(list_l_r))
        L_B_car.ID = left_lane_index[np.asscalar(index_3[0])]
        L_B_car.X_r = x_l_f[np.asscalar(index_3[0])]
        L_B_car.Y_r = y_l_f[np.asscalar(index_3[0])]
        L_B_car.Distance = left_lane_distance[np.asscalar(index_3[0])]

    list_r_f = list(j for j in right_lane_distance if j > 0)

    if list_r_f:
        index_4 = np.argwhere(right_lane_distance == min(list_r_f))
        R_F_car.ID = right_lane_index[np.asscalar(index_4[0])]
        R_F_car.X_r = x_r[np.asscalar(index_4[0])]
        R_F_car.Y_r = y_r[np.asscalar(index_4[0])]
        R_F_car.Distance = right_lane_distance[np.asscalar(index_4[0])]

    list_r_b = list(j for j in right_lane_distance if j <= 0)

    if list_r_b:
        index_5 = np.argwhere(right_lane_distance == max(list_r_b))
        R_B_car.ID = right_lane_index[np.asscalar(index_5[0])]
        R_B_car.X_r = x_r_f[np.asscalar(index_5[0])]
        R_B_car.Y_r = y_r_f[np.asscalar(index_5[0])]
        R_B_car.Distance = right_lane_distance[np.asscalar(index_5[0])]

    return S_F_car, L_F_car, L_B_car, R_F_car, R_B_car