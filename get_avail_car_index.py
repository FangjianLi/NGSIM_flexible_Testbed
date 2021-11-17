import pandas as pd

DATA_FILE_DIR = 'NGSIM_us101.txt'
TIME_INTERVAL = 100  # ms
START_TIME = 25000  # ms
END_TIME = 40000
INTERESTED_LANES = [1, 2, 3]


def get_ids_and_data():
    data_1 = pd.read_csv('NGSIM_us101.txt', sep='\t')
    data_1['Global_Time'] = data_1['Global_Time'] - data_1['Global_Time'].min()
    print(len(data_1['Vehicle_ID'].unique()))
    data_1_A = data_1[(data_1['Lane_ID'].isin(INTERESTED_LANES)) & (data_1['Global_Time'] < END_TIME) & (
                data_1['Global_Time'] > START_TIME) & (data_1['Global_Time'] % TIME_INTERVAL == 0)]
    basic_vehicle_ID = data_1_A['Vehicle_ID'].unique().tolist()

    return data_1_A, basic_vehicle_ID




if __name__ == '__main__':
    _, id = get_ids_and_data()
    print('The basic vehicle IDs are:', id)
    print('Please choose one number from it')