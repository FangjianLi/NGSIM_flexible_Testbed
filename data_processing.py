from get_avail_car_index import get_ids_and_data
import numpy as np
import pandas as pd

INTERESTED_CAR_ID = 44  # this value is chosen based on the IDs from running get_avail_car_index.py


def data_processing():
    data_1_A, basic_vehicle_ID = get_ids_and_data()
    car_interested_data = data_1_A.loc[data_1_A["Vehicle_ID"] == INTERESTED_CAR_ID, data_1_A.columns]
    car_interested_data = car_interested_data.sort_values(by='Global_Time')
    total_length = car_interested_data['Global_Time'].shape[0]

    environment_vehicle_ID = set()

    basic_vehicle_ID.remove(INTERESTED_CAR_ID)

    for index in basic_vehicle_ID:
        time_min = data_1_A.loc[data_1_A['Vehicle_ID'] == index, 'Global_Time'].min()
        time_max = data_1_A.loc[data_1_A['Vehicle_ID'] == index, 'Global_Time'].max()
        for i in range(time_min, time_max, 100):
            x1 = data_1_A.loc[(data_1_A['Vehicle_ID'] == index) & (data_1_A['Global_Time'] == i), 'Global_X'].tolist()
            y1 = data_1_A.loc[(data_1_A['Vehicle_ID'] == index) & (data_1_A['Global_Time'] == i), 'Global_Y'].tolist()
            xr = data_1_A.loc[
                (data_1_A['Vehicle_ID'] == INTERESTED_CAR_ID) & (data_1_A['Global_Time'] == i), 'Global_X'].tolist()
            yr = data_1_A.loc[
                (data_1_A['Vehicle_ID'] == INTERESTED_CAR_ID) & (data_1_A['Global_Time'] == i), 'Global_Y'].tolist()
            distance1 = np.sqrt((x1[0] - xr[0]) ** 2 + (y1[0] - yr[0]) ** 2) * 0.3048
            if distance1 < 40:
                environment_vehicle_ID.add(index)

    min_total_time = car_interested_data['Global_Time'].min()
    max_total_time = car_interested_data['Global_Time'].max()

    return data_1_A, environment_vehicle_ID, total_length, min_total_time, max_total_time



def process_the_map():
    boundary = pd.read_excel('us101_road.xls')
    centerline_1 = pd.read_csv('./us_101_traj/us_101_lane1', sep='\s+', names=['X', 'Y'])
    centerline_1a = centerline_1.to_numpy() * 0.3048
    centerline_2 = pd.read_csv('./us_101_traj/us_101_lane2', sep='\s+', names=['X', 'Y'])
    centerline_2a = centerline_2.to_numpy() * 0.3048
    centerline_3 = pd.read_csv('./us_101_traj/us_101_lane3', sep='\s+', names=['X', 'Y'])
    centerline_3a = centerline_3.to_numpy() * 0.3048

    centerline = {
        1: centerline_1a,
        2: centerline_2a,
        3: centerline_3a
    }

    boundary[['lane1_x', 'lane1_y']] = boundary.B_1.str.split(expand=True)
    boundary[['lane2_x', 'lane2_y']] = boundary.B_2.str.split(expand=True)
    boundary[['lane3_x', 'lane3_y']] = boundary.B_3.str.split(expand=True)
    boundary[['lane4_x', 'lane4_y']] = boundary.B_4.str.split(expand=True)
    boundary[['lane5_x', 'lane5_y']] = boundary.B_5.str.split(expand=True)
    boundary[['lane6_x', 'lane6_y']] = boundary.B_6.str.split(expand=True)
    boundary[['lane7_x', 'lane7_y']] = boundary.B_7.str.split(expand=True)
    boundary[['lane8_x', 'lane8_y']] = boundary.B_8.str.split(expand=True)
    boundary[['lane9_x', 'lane9_y']] = boundary.B_9.str.split(expand=True)
    boundary[['lane10_x', 'lane10_y']] = boundary.B_10.str.split(expand=True)
    boundary[['lane11_x', 'lane11_y']] = boundary.B_11.str.split(expand=True)
    boundary[['lane12_x', 'lane12_y']] = boundary.B_12.str.split(expand=True)
    list_1 = ['lane1_x', 'lane1_y', 'lane2_x', 'lane2_y', 'lane3_x', 'lane3_y', 'lane4_x', 'lane4_y', 'lane5_x',
              'lane5_y',
              'lane6_x', 'lane6_y', 'lane7_x', 'lane7_y', 'lane8_x', 'lane8_y', 'lane9_x', 'lane9_y',
              'lane10_x', 'lane10_y', 'lane11_x', 'lane11_y', 'lane12_x', 'lane12_y']
    boundary[list_1] = boundary[list_1].astype(float)
    lanes = boundary[list_1].to_numpy() * 0.3048

    return centerline, lanes





if __name__ == '__main__':
    _, enviromental_vehicle_ID, _, min_total_time, max_total_time = data_processing()

    print("The environmental vehicle IDs of vechicle {}:".format(INTERESTED_CAR_ID))
    print(enviromental_vehicle_ID)

    print("The simulation time is {}".format(max_total_time- min_total_time))
