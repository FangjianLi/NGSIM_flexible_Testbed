import numpy as np


class neighbor_car:
    ID = 0
    X_r = 0
    Y_r = 0
    Distance = 0


class test_car_c:
    def __init__(self, duplicate_car, front_duplicate_car, total_length, TIME_INTERVAL):
        self.X = np.zeros(total_length + 1)
        self.Y = np.zeros(total_length + 1)
        self.PHI = np.zeros(total_length + 1)  # heading angle
        self.beta = np.zeros(total_length + 1)
        self.vel = np.zeros(total_length + 1)
        self.h = np.zeros(total_length + 1)
        self.a = np.zeros(total_length + 1)
        self.theta = np.zeros(total_length + 1)  #
        self.theta_f = np.zeros(total_length + 1)
        self.phi = np.zeros(total_length + 1)
        self.r = np.zeros(total_length + 1)
        self.width = duplicate_car.width
        self.length = duplicate_car.length

        self.X[0] = duplicate_car.X[0]
        self.Y[0] = duplicate_car.Y[0]
        self.PHI[0] = np.radians(-42)
        self.vel[0] = duplicate_car.vel[0]
        car_f_x = front_duplicate_car.X[0] - front_duplicate_car.length * np.cos(np.radians(40))
        car_f_y = front_duplicate_car.Y[0] + front_duplicate_car.length * np.sin(np.radians(40))
        self.h[0] = np.sqrt((car_f_x - duplicate_car.X[0]) ** 2 + (car_f_y - duplicate_car.Y[0]) ** 2) / \
                    duplicate_car.vel[0]  #
        self.a[0] = (duplicate_car.vel[1] - duplicate_car.vel[0]) / TIME_INTERVAL * 1000
        self.vel[-1] = duplicate_car.vel[0]



class car_generator:
    def __init__(self, veh_id, data_1_A, total_length, min_total_time, max_total_time, TIME_INTERVAL):
        self.id = veh_id
        self.X = np.zeros(total_length)
        self.Y = np.zeros(total_length)
        self.vel = np.zeros(total_length)
        self.lane = np.zeros(total_length)
        self.preceding = np.zeros(total_length)

        min_data_time = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Global_Time'].min()
        max_data_time = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Global_Time'].max()

        start_time_diff = int((min_data_time - min_total_time) / TIME_INTERVAL)
        end_time_diff = int((max_data_time - max_total_time) / TIME_INTERVAL)

        data_length = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Lane_ID'].shape[0]
        lane_id = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Lane_ID'].tolist()
        self.lane[0 + max(0, start_time_diff):total_length + min(0, end_time_diff)] = lane_id[0 + max(0,
                                                                                                      -start_time_diff):data_length + min(
            0, -end_time_diff)]
        car_x = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Global_X'].tolist()
        self.X[0 + max(0, start_time_diff):total_length + min(0, end_time_diff)] = car_x[0 + max(0,
                                                                                                 -start_time_diff):data_length + min(
            0, -end_time_diff)]
        self.X *= 0.3048
        car_y = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Global_Y'].tolist()
        self.Y[0 + max(0, start_time_diff):total_length + min(0, end_time_diff)] = car_y[0 + max(0,
                                                                                                 -start_time_diff):data_length + min(
            0, -end_time_diff)]
        self.Y *= 0.3048
        car_vel = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'v_Vel'].tolist()
        self.vel[0 + max(0, start_time_diff):total_length + min(0, end_time_diff)] = car_vel[0 + max(0,
                                                                                                     -start_time_diff):data_length + min(
            0, -end_time_diff)]
        self.vel *= 0.3048
        self.length = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'v_length'].tolist()[0]
        self.length *= 0.3048
        self.width = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'v_Width'].tolist()[0]
        self.width *= 0.3048
        car_preceding = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Preceding'].tolist()
        self.preceding[0 + max(0, start_time_diff):total_length + min(0, end_time_diff)] = car_preceding[0 + max(0,
                                                                                                                 -start_time_diff):data_length + min(
            0, -end_time_diff)]