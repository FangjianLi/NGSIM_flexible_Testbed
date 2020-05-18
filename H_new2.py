import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
"""
data_1=pd.read_csv('/home/fangjil/Desktop/i101_trajectories-0750am-0805am.txt',header=None,sep="\s+", usecols=range(18),nrows=1000000,
                   names=['Vehicle_ID', 'Frame_ID','Total_Frames','Global_Time','Local_X','Local_Y','Global_X','Global_Y','v_length','v_Width',\
                          'v_Class','v_Vel','v_ACC','Lane_ID','Preceding','Following','Space_Headway','Time_Headway'])

useful_columns=['Vehicle_ID', 'Total_Frames','Global_Time','Global_X','Global_Y','v_length','v_Width','v_Class','v_Vel','v_ACC','Lane_ID','Preceding','Following','Space_Headway','Time_Headway']
data_1=data_1[useful_columns]
data_1.to_csv('/home/fangjil/Desktop/NGSIM_us101.txt',sep='\t',index=False)
"""







TIME_INTERVAL=100 #ms
START_TIME=25000 #ms
END_TIME=40000
INTERESTED_LANES=[1,2,3]
INTERESTED_CAR_ID=44

#Lane change delta

DELTA_1=-4
DELTA_2=-5
DELTA_3=-5

lane_change_delta_1=[0,DELTA_1,DELTA_2,DELTA_3]

INSTANCE_1=42
INSTANCE_2=56
INSTANCE_3=79

lane_change_instance_1=[0,INSTANCE_1,INSTANCE_2,INSTANCE_3]




data_1=pd.read_csv('NGSIM_us101.txt',sep='\t')
data_1['Global_Time']=data_1['Global_Time']-data_1['Global_Time'].min()
print(len(data_1['Vehicle_ID'].unique()))
data_1_A=data_1[(data_1['Lane_ID'].isin(INTERESTED_LANES)) & (data_1['Global_Time']<END_TIME)& (data_1['Global_Time']>START_TIME) &(data_1['Global_Time']%TIME_INTERVAL==0)]
data_1_B=data_1[(data_1['Lane_ID'].isin(INTERESTED_LANES))]
print(data_1_A.shape)

car_interested_data=data_1_A.loc[data_1_A["Vehicle_ID"]==INTERESTED_CAR_ID,data_1_A.columns]
car_interested_data=car_interested_data.sort_values(by='Global_Time')
print(car_interested_data['Global_Time'])
total_length=car_interested_data['Global_Time'].shape[0]


basic_vehicle_ID=data_1_A['Vehicle_ID'].unique().tolist()

print('The basic vehicle ID are:', basic_vehicle_ID)

environment_vehicle_ID=set()

basic_vehicle_ID.remove(INTERESTED_CAR_ID)

for index in basic_vehicle_ID:
    time_min=data_1_A.loc[data_1_A['Vehicle_ID']==index,'Global_Time'].min()
    time_max=data_1_A.loc[data_1_A['Vehicle_ID']==index,'Global_Time'].max()
    for i in range(time_min,time_max,100):
        x1=data_1_A.loc[(data_1_A['Vehicle_ID']==index) & (data_1_A['Global_Time']==i),'Global_X'].tolist()
        y1=data_1_A.loc[(data_1_A['Vehicle_ID']==index) & (data_1_A['Global_Time']==i),'Global_Y'].tolist()
        xr=data_1_A.loc[(data_1_A['Vehicle_ID']==INTERESTED_CAR_ID) & (data_1_A['Global_Time']==i),'Global_X'].tolist()
        yr=data_1_A.loc[(data_1_A['Vehicle_ID']==INTERESTED_CAR_ID) & (data_1_A['Global_Time']==i),'Global_Y'].tolist()
        distance1=np.sqrt((x1[0]-xr[0])**2+(y1[0]-yr[0])**2)*0.3048
        if distance1<40:
            environment_vehicle_ID.add(index)




#environment_vehicle_ID=[32,37,39,40,43,47,49,54,23,56,26,27]  #this is the results from previous one

min_total_time=car_interested_data['Global_Time'].min()
max_total_time=car_interested_data['Global_Time'].max()

class car_generator:
    def __init__(self, veh_id):
        self.id=veh_id
        self.X= np.zeros(total_length)
        self.Y= np.zeros(total_length)
        self.vel=np.zeros(total_length)
        self.lane=np.zeros(total_length)
        self.preceding=np.zeros(total_length)

        min_data_time=data_1_A.loc[data_1_A['Vehicle_ID']==veh_id,'Global_Time'].min()
        max_data_time=data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Global_Time'].max()

        start_time_diff=int((min_data_time-min_total_time)/TIME_INTERVAL)
        end_time_diff=int((max_data_time-max_total_time)/TIME_INTERVAL)

        data_length=data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Lane_ID'].shape[0]
        lane_id=data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Lane_ID'].tolist()
        self.lane[0+max(0,start_time_diff):total_length+min(0,end_time_diff)]=lane_id[0+max(0,-start_time_diff):data_length+min(0,-end_time_diff)]
        car_x=data_1_A.loc[data_1_A['Vehicle_ID']==veh_id,'Global_X'].tolist()
        self.X[0+max(0,start_time_diff):total_length+min(0,end_time_diff)] = car_x[0+max(0,-start_time_diff):data_length+min(0,-end_time_diff)]
        self.X*=0.3048
        car_y=data_1_A.loc[data_1_A['Vehicle_ID']==veh_id,'Global_Y'].tolist()
        self.Y[0+max(0,start_time_diff):total_length+min(0,end_time_diff)] = car_y[0+max(0,-start_time_diff):data_length+min(0,-end_time_diff)]
        self.Y*=0.3048
        car_vel=data_1_A.loc[data_1_A['Vehicle_ID']==veh_id,'v_Vel'].tolist()
        self.vel[0+max(0,start_time_diff):total_length+min(0,end_time_diff)] = car_vel[0+max(0,-start_time_diff):data_length+min(0,-end_time_diff)]
        self.vel*=0.3048
        self.length=data_1_A.loc[data_1_A['Vehicle_ID']==veh_id,'v_length'].tolist()[0]
        self.length*=0.3048
        self.width=data_1_A.loc[data_1_A['Vehicle_ID']==veh_id,'v_Width'].tolist()[0]
        self.width*=0.3048
        car_preceding = data_1_A.loc[data_1_A['Vehicle_ID'] == veh_id, 'Preceding'].tolist()
        self.preceding[0 + max(0, start_time_diff):total_length + min(0, end_time_diff)] = car_preceding[0 + max(0, -start_time_diff):data_length + min(0, -end_time_diff)]


def get_lane_sequence(lane_change_delta, lane_change_index, choose_1):
    lane_id=[car_interested.lane[0]]
    lane_index=[0]

    for i in range(total_length):
        if car_interested.lane[i] != lane_id[-1]:
            lane_id.append(car_interested.lane[i])
            lane_index.append(i)

    print("The time instances of the interested car are", lane_index)

    lane_index=np.array(lane_index)


    if choose_1==1:
        lane_index+=np.array(lane_change_delta)[:len(lane_index)]
    else:
        lane_index=np.array(lane_change_index)[:len(lane_index)]

    print("The time instances of the tested car are", lane_index)
    print("The lane ID of the interested car are", lane_index)

    lane_index=lane_index.tolist()

    lane_index.append(total_length)

    lane_change_sequence=np.zeros(total_length)



    for j in range(len(lane_index)-1):

        lane_change_sequence[lane_index[j]:lane_index[j+1]]=lane_id[j]

    return lane_change_sequence




class neighbor_car:
    ID=0
    X_r=0
    Y_r=0
    Distance=0



def get_relevant_cars(i,Car_X,Car_Y,Car_Lane):
    #define the interested cars

    S_F_car=neighbor_car()
    L_F_car=neighbor_car()
    L_B_car=neighbor_car()
    R_F_car=neighbor_car()
    R_B_car=neighbor_car()


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

    for car_index in environment_vehicle_ID:
        car_a = car_generator(car_index)
        if car_a.lane[i] == Car_Lane:
            same_lane_index.append(car_index)
            dis_vector = np.array([car_a.X[i] - Car_X, car_a.Y[i] - Car_Y])
            dist_x = np.dot(trans_matrix, dis_vector)
            same_lane_distance.append(dist_x)
            x_f.append(car_a.X[i] - car_a.length * np.cos(np.radians(40)))
            y_f.append(car_a.Y[i] + car_a.length * np.sin(np.radians(40)))
        # print(same_lane_distance)
        # print(min(j for j in same_lane_distance if j>0))

        if car_a.lane[i] == Car_Lane - 1:
            left_lane_index.append(car_index)
            dis_vector_l = np.array([car_a.X[i] - Car_X, car_a.Y[i] - Car_Y])
            dist_x_l = np.dot(trans_matrix, dis_vector_l)
            left_lane_distance.append(dist_x_l)
            x_l.append(car_a.X[i] - car_a.length * np.cos(np.radians(40)))
            x_l_f.append(car_a.X[i])
            y_l.append(car_a.Y[i] + car_a.length * np.sin(np.radians(40)))
            y_l_f.append(car_a.Y[i])
        # print(same_lane_distance)
        # print(min(j for j in same_lane_distance if j>0))

        if car_a.lane[i] == Car_Lane + 1:
            right_lane_index.append(car_index)
            dis_vector_r = np.array([car_a.X[i] - Car_X, car_a.Y[i] - Car_Y])
            dist_x_r = np.dot(trans_matrix, dis_vector_r)
            right_lane_distance.append(dist_x_r)
            x_r.append(car_a.X[i] - car_a.length * np.cos(np.radians(40)))
            x_r_f.append(car_a.X[i])
            y_r.append(car_a.Y[i] + car_a.length * np.sin(np.radians(40)))
            y_r_f.append(car_a.Y[i])
        # print(same_lane_distance)
        # print(min(j for j in same_lane_distance if j>0))

        # target_sl=min(j for j in same_lane_distance if j>0)
    list_sl = list(j for j in same_lane_distance if j > 0)

    if list_sl:
        index_1 = np.argwhere(same_lane_distance == min(list_sl))
        S_F_car.ID = same_lane_index[np.asscalar(index_1[0])]
        S_F_car.X_r = x_f[np.asscalar(index_1[0])]
        S_F_car.Y_r = y_f[np.asscalar(index_1[0])]
        S_F_car.Distance = same_lane_distance[np.asscalar(index_1[0])]

    list_l_f = list(j for j in left_lane_distance if j > 0)

    # target_l_f = min(j for j in left_lane_distance if j > 0)

    if list_l_f:
        index_2 = np.argwhere(left_lane_distance == min(list_l_f))
        L_F_car.ID = left_lane_index[np.asscalar(index_2[0])]
        L_F_car.X_r = x_l[np.asscalar(index_2[0])]
        L_F_car.Y_r = y_l[np.asscalar(index_2[0])]
        L_F_car.Distance = left_lane_distance[np.asscalar(index_2[0])]

    list_l_r = list(j for j in left_lane_distance if j <= 0)
    # target_l_r = max(j for j in left_lane_distance if j <= 0)

    if list_l_r:
        index_3 = np.argwhere(left_lane_distance == max(list_l_r))
        L_B_car.ID = left_lane_index[np.asscalar(index_3[0])]
        L_B_car.X_r = x_l_f[np.asscalar(index_3[0])]
        L_B_car.Y_r = y_l_f[np.asscalar(index_3[0])]
        L_r_distance[i] = left_lane_distance[np.asscalar(index_3[0])]

    list_r_f = list(j for j in right_lane_distance if j > 0)
    # target_r_f = min(j for j in right_lane_distance if j > 0)

    if list_r_f:
        index_4 = np.argwhere(right_lane_distance == min(list_r_f))
        R_F_car.ID = right_lane_index[np.asscalar(index_4[0])]
        R_F_car.X_r = x_r[np.asscalar(index_4[0])]
        R_F_car.Y_r = y_r[np.asscalar(index_4[0])]
        R_F_car.Distance = right_lane_distance[np.asscalar(index_4[0])]

    list_r_b = list(j for j in right_lane_distance if j <= 0)
    # target_r_f = min(j for j in right_lane_distance if j > 0)

    if list_r_b:
        index_5 = np.argwhere(right_lane_distance == max(list_r_b))
        R_B_car.ID = right_lane_index[np.asscalar(index_5[0])]
        R_B_car.X_r = x_r_f[np.asscalar(index_5[0])]
        R_B_car.Y_r = y_r_f[np.asscalar(index_5[0])]
        R_B_car.Distance = right_lane_distance[np.asscalar(index_5[0])]


    return S_F_car, L_F_car, L_B_car, R_F_car, R_B_car


def vehicle_following_controller(interdistance, desired_h, vel, previous_h):
    k_car = 4
    # k_follow=-1
    k_follow = 1.5

    current_h=interdistance/vel
    delta_a=k_car*(current_h-previous_h)+k_follow*(current_h-desired_h)*TIME_INTERVAL/1000
    print("error:", current_h-desired_h)
    #delta_a = k_follow * (current_h - desired_h) * delta_t
    return delta_a, current_h


def speed_tacking_controller(vel,previous_vel,desired_vel):
    k_I=0.4
    k_n=0.5
    delta_a=k_n*(previous_vel-vel)+k_I*(desired_vel-vel)
    return delta_a



def lane_keeping_controller(Car_X,Car_Y,look_ahead_dis, heading_angle, previous_theta, Car_line, X1, Y1,previous_theta_f):
    kn = 0.9
    KI = 0.7
    kf = 0.05
    rotational_matrix_X=np.array([np.cos(-heading_angle),-np.sin(-heading_angle)])
    rotational_matrix_Y=np.array([np.sin(-heading_angle),np.cos(-heading_angle)])

    centerline_x=centerline[Car_line][:,0]
    centerline_y=centerline[Car_line][:,1]

    #X_1=Car_X+look_ahead_dis*np.cos(heading_angle)

    #Y_1=np.interp(X_1,centerline_x1,centerline_y1)

    X_0=Car_X+look_ahead_dis*np.cos(heading_angle)
    Y_0=Car_Y+look_ahead_dis*np.sin(heading_angle)

    distance_0=1000
    X_1=0
    Y_1=0

    index_1=np.argwhere((centerline_x<X_0+130)&(centerline_x>X_0-130))
    centerline_x1=centerline_x[index_1].flatten()
    centerline_y1=centerline_y[index_1].flatten()
    for ix in np.arange(X_0-100,X_0+100,0.1):
        iy=np.interp(ix,centerline_x1,centerline_y1)
        distance_1=np.sqrt((X_0-ix)**2+(Y_0-iy)**2)
        if distance_1<distance_0:
            distance_0=distance_1
            X_1=ix
            Y_1=iy


    #X_new=np.dot(rotational_matrix_X,np.array([X_1-Car_X,Y_1-Car_Y]))
    #Y_new=np.dot(rotational_matrix_Y,np.array([X_1-Car_X,Y_1-Car_Y]))

    X_new=np.dot(rotational_matrix_X,np.array([X_1-Car_X,Y_1-Car_Y]))
    Y_new=np.dot(rotational_matrix_Y,np.array([X_1-Car_X,Y_1-Car_Y]))

    X_front=np.dot(rotational_matrix_X,np.array([X1-Car_X,Y1-Car_Y]))
    Y_front=np.dot(rotational_matrix_Y,np.array([X1-Car_X,Y1-Car_Y]))
    theta_front=np.arctan(Y_front/X_front)

    theta=np.arctan(Y_new/X_new)

    #theta_new=np.arctan(y_new/x_new)
    #print("new,old",theta,theta_new)
    print("angle:",theta)
    change_theta=theta-previous_theta
    change_theta_f=theta_front-previous_theta_f
    delta_phi=kf*(np.clip(change_theta_f,-0.007,0.007))+kn*(np.clip(change_theta,-0.007,0.007))+KI*np.clip(theta,-0.06,0.06)*TIME_INTERVAL/1000
    print("change_theta_f, change_theta, theta:",change_theta_f,change_theta,theta)

    return delta_phi, theta, theta_front, X_1, Y_1
def vehicle_dynamic_model(beta_0, vel, phi,r_0,PHI_0):
    m=1500
    Jz=2500
    a=1.1
    b=1.6
    C1=55000
    C2=60000
    delta_beta=1/m/vel*(-(C1+C2)*beta_0-(m*vel+1/vel*(a*C1-b*C2))*r_0+C1*phi)*TIME_INTERVAL/1000
    delta_r=1/Jz*((-a*C1+b*C2)*beta_0+1/vel*(-a**2*C1-b**2*C2)*r_0+a*C1*phi)*TIME_INTERVAL/1000
    beta_a=beta_0+delta_beta
    r_a=r_0+delta_r
    PHI_a=PHI_0+r_0*TIME_INTERVAL/1000 #check this
    delta_X=(np.cos(PHI_0)*vel*np.cos(beta_0)-np.sin(PHI_0)*vel*np.sin(beta_0))*TIME_INTERVAL/1000
    delta_y=(np.sin(PHI_0)*vel*np.cos(beta_0)+np.cos(PHI_0)*vel*np.sin(beta_0))*TIME_INTERVAL/1000
    return delta_X, delta_y, r_a, beta_a,PHI_a



class test_car_c:
    def __init__(self,duplicate_car,front_duplicate_car):
        self.X = np.zeros(total_length+1)
        self.Y = np.zeros(total_length+1)
        self.PHI = np.zeros(total_length+1) #heading angle
        self.beta = np.zeros(total_length+1)
        self.vel = np.zeros(total_length+1)
        self.h = np.zeros(total_length+1)
        self.a = np.zeros(total_length+1)
        self.theta = np.zeros(total_length+1) #
        self.theta_f=np.zeros(total_length+1)
        self.phi = np.zeros(total_length+1)
        self.r = np.zeros(total_length+1)
        self.width = duplicate_car.width
        self.length=duplicate_car.length

        self.X[0] = duplicate_car.X[0]
        self.Y[0] = duplicate_car.Y[0]
        self.PHI[0] = np.radians(-42)
        self.vel[0] = duplicate_car.vel[0]
        car_f_x=front_duplicate_car.X[0] - front_duplicate_car.length * np.cos(np.radians(40))
        car_f_y=front_duplicate_car.Y[0] + front_duplicate_car.length * np.sin(np.radians(40))
        self.h[0] = np.sqrt((car_f_x - duplicate_car.X[0]) ** 2 + (car_f_y - duplicate_car.Y[0]) ** 2) / duplicate_car.vel[0] #
        self.a[0] = (duplicate_car.vel[1] - duplicate_car.vel[0]) / TIME_INTERVAL*1000
        self.vel[-1] = duplicate_car.vel[0]





boundary=pd.read_excel('us101_road.xlsx')
centerline_1=pd.read_csv('./us_101_traj/us_101_lane1',sep='\s+', names=['X','Y'])
centerline_1a=centerline_1.to_numpy()*0.3048
centerline_2=pd.read_csv('./us_101_traj/us_101_lane2',sep='\s+', names=['X','Y'])
centerline_2a=centerline_2.to_numpy()*0.3048
centerline_3=pd.read_csv('./us_101_traj/us_101_lane3',sep='\s+', names=['X','Y'])
centerline_3a=centerline_3.to_numpy()*0.3048


centerline={
    1: centerline_1a,
    2: centerline_2a,
    3: centerline_3a
}

boundary[['lane1_x','lane1_y']]=boundary.B_1.str.split(expand=True)
boundary[['lane2_x','lane2_y']]=boundary.B_2.str.split(expand=True)
boundary[['lane3_x','lane3_y']]=boundary.B_3.str.split(expand=True)
boundary[['lane4_x','lane4_y']]=boundary.B_4.str.split(expand=True)
boundary[['lane5_x','lane5_y']]=boundary.B_5.str.split(expand=True)
boundary[['lane6_x','lane6_y']]=boundary.B_6.str.split(expand=True)
boundary[['lane7_x','lane7_y']]=boundary.B_7.str.split(expand=True)
boundary[['lane8_x','lane8_y']]=boundary.B_8.str.split(expand=True)
boundary[['lane9_x','lane9_y']]=boundary.B_9.str.split(expand=True)
boundary[['lane10_x','lane10_y']]=boundary.B_10.str.split(expand=True)
boundary[['lane11_x','lane11_y']]=boundary.B_11.str.split(expand=True)
boundary[['lane12_x','lane12_y']]=boundary.B_12.str.split(expand=True)
list_1=['lane1_x','lane1_y','lane2_x','lane2_y','lane3_x','lane3_y','lane4_x','lane4_y','lane5_x','lane5_y',
        'lane6_x','lane6_y','lane7_x','lane7_y','lane8_x','lane8_y','lane9_x','lane9_y',
        'lane10_x','lane10_y','lane11_x','lane11_y','lane12_x','lane12_y']
boundary[list_1]=boundary[list_1].astype(float)
lanes=boundary[list_1].to_numpy()*0.3048


def get_lane_ID(car_X,car_Y):
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

    if car_Y>Y_2:
        lane_ID=1
    elif car_Y>Y_3:
        lane_ID=2
    elif car_Y>Y_4:
        lane_ID=3
    else:
        lane_ID=10

    return lane_ID


for i in environment_vehicle_ID:
    globals()['car_{}'.format(i)]=car_generator(i)


car_interested=car_generator(INTERESTED_CAR_ID) # this is our chosen car

"""
car_23=car_generator(23)
car_26=car_generator(26)
car_27=car_generator(27)
car_32=car_generator(32)
car_37=car_generator(37)
car_39=car_generator(39)
car_40=car_generator(40)
car_43=car_generator(43)
car_47=car_generator(47)
car_49=car_generator(49)
car_54=car_generator(54)
car_56=car_generator(56)
"""



lane_intervals=get_lane_sequence(lane_change_delta_1, lane_change_instance_1, 0)

print(lane_intervals)

#same lane searching
front_vehicle=np.zeros(total_length)
X_f=np.zeros(total_length)
Y_f=np.zeros(total_length)
S_f_distance=np.zeros(total_length)

#left lane searching
left_front_vehicle=np.zeros(total_length)
X_l_f=np.zeros(total_length)
Y_l_f=np.zeros(total_length)
L_f_distance=np.zeros(total_length)

left_rear_vehicle=np.zeros(total_length)
X_l_r=np.zeros(total_length)
Y_l_r=np.zeros(total_length)
L_r_distance=np.zeros(total_length)

#right lane searching
right_front_vehicle=np.zeros(total_length)
X_r_f=np.zeros(total_length)
Y_r_f=np.zeros(total_length)
R_f_distance=np.zeros(total_length)

right_rear_vehicle=np.zeros(total_length)
X_r_b=np.zeros(total_length)
Y_r_b=np.zeros(total_length)
R_b_distance=np.zeros(total_length)

# the initial value of the test car


test_car=test_car_c(car_interested,globals()['car_{}'.format(int(car_interested.preceding[0]))])


front_vehicle_2 = np.zeros(total_length)
X_f_2 = np.zeros(total_length)
Y_f_2 = np.zeros(total_length)

#check the cose

near_point_X=np.zeros(total_length)
near_point_Y=np.zeros(total_length)

#
lane_ID_list=np.zeros(total_length)

for i in range(total_length):

    print(i)

    subject_car_X=car_interested.X[i]
    subject_car_Y=car_interested.Y[i]
    subject_car_Lane=car_interested.lane[i]

    S_F_car1, L_F_car1, L_B_car1, R_F_car1, R_B_car1=get_relevant_cars(i, subject_car_X, subject_car_Y, subject_car_Lane)

    front_vehicle[i]=S_F_car1.ID
    X_f[i]=S_F_car1.X_r
    Y_f[i]=S_F_car1.Y_r
    S_f_distance[i]=S_F_car1.Distance

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


    lane_ID_list[i] = get_lane_ID(test_car.X[i], test_car.Y[i])




    S_F_car2, L_F_car2, L_B_car2, R_F_car2, R_B_car2 = get_relevant_cars(i, test_car.X[i], test_car.Y[i],lane_intervals[i])



    front_vehicle_2[i]=S_F_car2.ID
    X_f_2[i]=S_F_car2.X_r
    Y_f_2[i]=S_F_car2.Y_r

    interdistance0=np.sqrt((X_f_2[i]-test_car.X[i])**2+(Y_f_2[i]-test_car.Y[i])**2)

    print(interdistance0)


    delta_a, h_1 = vehicle_following_controller(interdistance0, 1.05, test_car.vel[i], test_car.h[i])

    if (interdistance0>5*test_car.vel[i]) | (front_vehicle_2[i]==0):
        delta_a=speed_tacking_controller(test_car.vel[i], test_car.vel[i-1], 20)
        h_1=5


    if i%1==0:
        delta_a1=delta_a


    test_car.a[i+1]=test_car.a[i]+delta_a1
    test_car.vel[i+1]=test_car.vel[i]+test_car.a[i]*TIME_INTERVAL/1000
    test_car.h[i+1]=h_1


    delta_phi, theta_1, theta_f, X1, Y1 = lane_keeping_controller(test_car.X[i], test_car.Y[i], 10,
                                                                        test_car.PHI[i], test_car.theta[i],
                                                                        lane_intervals[i],
                                                                        X_f_2[i], Y_f_2[i], test_car.theta_f[i])


    if i%1==0:
        delta_phi_1=delta_phi


    test_car.phi[i + 1] = test_car.phi[i] + delta_phi_1
    test_car.theta[i + 1] = theta_1
    test_car.theta_f[i + 1] = theta_f
    near_point_X[i] = X1
    near_point_Y[i] = Y1





    #delta_X, delta_Y, r_a, beta_a, PHI_a =vehicle_dynamic_model(test_car.beta[i], test_car.vel[i], test_car.phi[i], test_car.r[i], test_car.PHI[i])
    delta_X, delta_Y, r_a, beta_a, PHI_a = vehicle_dynamic_model(test_car.beta[i], test_car.vel[i], test_car.phi[i], test_car.r[i], test_car.PHI[i])

    test_car.X[i+1]=test_car.X[i]+delta_X
    test_car.Y[i+1]=test_car.Y[i]+delta_Y
    test_car.r[i+1]=r_a
    test_car.beta[i+1]=beta_a
    test_car.PHI[i+1]=PHI_a


fig=plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7,7)
#ax=plt.axes(xlim=(1966360,1966660),ylim=(570600,570900))

ax=plt.axes(xlim=(1966360,1966660),ylim=(570600,570900))


plt.plot(lanes[:,0],lanes[:,1],'lightsteelblue',label="lane1") #boundary 1
plt.plot(lanes[:,2],lanes[:,3],'lightsteelblue',label="lane2")
plt.plot(lanes[:,4],lanes[:,5],'lightsteelblue',label="lane3")
plt.plot(lanes[:,6],lanes[:,7],'lightsteelblue',label="lane4")
plt.plot(lanes[:,8],lanes[:,9],'lightsteelblue',label="lane5")
plt.plot(lanes[:,10],lanes[:,11],'lightsteelblue',label="lane6")
plt.plot(lanes[:,12],lanes[:,13],'lightsteelblue',label="lane7")#boundary_5
plt.plot(lanes[:,14],lanes[:,15],'lightsteelblue',label="lane8")#boundary_4
plt.plot(lanes[:,16],lanes[:,17],'lightsteelblue',label="lane9")#boundary_3
plt.plot(lanes[:,18],lanes[:,19],'lightsteelblue',label="lane10")#boundary_2
plt.plot(lanes[:,20],lanes[:,21],'lightsteelblue',label="lane11")#boundary_6
plt.plot(lanes[:,22],lanes[:,23],'lightsteelblue',label="lane12")
plt.plot(centerline_1a[:,0],centerline_1a[:,1],'orange',ls=':')
plt.plot(centerline_2a[:,0],centerline_2a[:,1],'orange',ls=':')
plt.plot(centerline_3a[:,0],centerline_3a[:,1],'orange',ls=':')


#plt.plot(car_44_data['Global_X']*0.3048,car_44_data['Global_Y']*0.3048,'--k')
plt.plot(car_interested.X,car_interested.Y,'--c',linewidth=1)
plt.xlabel('x(m)')
plt.ylabel('y(m)')
#plt.legend()



line1, =ax.plot([0,1],[0,1],'r')
line1_x=np.zeros(2)
line1_y=np.zeros(2)

line2, =ax.plot([0,1],[0,1],'darkviolet')

line3, =ax.plot([0,1],[0,1],'lime')

line4, =ax.plot([0,1],[0,1],'k')

line5, =ax.plot([0,1],[0,1],'magenta')

line6, =ax.plot([0,1],[0,1],'k')

def get_patch(car_no):
    patch_no=plt.Rectangle((0,0),car_no.length,car_no.width,-40,color='g')
    return patch_no


patch_car_interested=plt.Rectangle((0,0),car_interested.length,car_interested.width,-40,color='b')
patch_car_test=plt.Rectangle((0,0),test_car.length,test_car.width,-40,color='r')

for j in environment_vehicle_ID:
    globals()['patch_car_{}'.format(j)]=get_patch(globals()['car_{}'.format(j)])




def get_xy(car_no, i):
    x1=car_no.X[i]-0.5*car_no.width*np.sin(np.radians(40))-car_no.length*np.cos(np.radians(40))
    y1=car_no.Y[i]-0.5*car_no.width*np.cos(np.radians(40))+car_no.length*np.sin(np.radians(40))
    return (x1,y1)


def init():
    ax.add_patch(patch_car_interested)
    ax.add_patch(patch_car_test)




    line1.set_xdata([0,1])
    line1.set_ydata([0,1])
    line2.set_xdata([0,1])
    line2.set_ydata([0,1])
    line3.set_xdata([0,1])
    line3.set_ydata([0,1])
    line4.set_xdata([0,1])
    line4.set_ydata([0,1])
    line5.set_xdata([0,1])
    line5.set_ydata([0,1])
    line6.set_ydata([0,1])

    animation_list = [line1, line2, line3, line4, line5, patch_car_interested, patch_car_test]

    for j in environment_vehicle_ID:
        ax.add_patch(globals()['patch_car_{}'.format(j)])
        animation_list.append(globals()['patch_car_{}'.format(j)])

    return animation_list






def animate(i):

    if right_front_vehicle[i]==0:
        line4.set_visible(False)
    else:
        line4.set_visible(True)

    if right_rear_vehicle[i]==0:
        line5.set_visible(False)
    else:
        line5.set_visible(True)

    if left_front_vehicle[i]==0:
        line2.set_visible(False)
    else:
        line2.set_visible(True)

    if front_vehicle[i]==0:
        line1.set_visible(False)
    else:
        line1.set_visible(True)

    if left_rear_vehicle[i]==0:
        line3.set_visible(False)
    else:
        line3.set_visible(True)

    patch_car_interested.xy = get_xy(car_interested, i)
    patch_car_test.xy = get_xy(test_car, i)





    #x1_test=test_car.X[i]-0.5*test_car.width*np.sin(test_car.PHI[i])-test_car.length*np.cos(test_car.PHI[i])
    #y1_test=test_car.Y[i]-0.5*test_car.width*np.cos(test_car.PHI[i])+test_car.length*np.sin(test_car.PHI[i])
    #patch_car_test.xy=(x1_test,y1_test)
    #patch_car_test.angle=np.degrees(test_car.PHI[i])

    line1.set_xdata([car_interested.X[i],X_f[i]])
    line1.set_ydata([car_interested.Y[i],Y_f[i]])
    line2.set_xdata([car_interested.X[i],X_l_f[i]])
    line2.set_ydata([car_interested.Y[i],Y_l_f[i]])
    line3.set_xdata([car_interested.X[i],X_l_r[i]])
    line3.set_ydata([car_interested.Y[i],Y_l_r[i]])
    line4.set_xdata([car_interested.X[i],X_r_f[i]])
    line4.set_ydata([car_interested.Y[i],Y_r_f[i]])
    line5.set_xdata([car_interested.X[i],X_r_b[i]])
    line5.set_ydata([car_interested.Y[i],Y_r_b[i]])

    line6.set_xdata([test_car.X[i],near_point_X[i]])
    line6.set_ydata([test_car.Y[i],near_point_Y[i]])

    animation_list = [line1, line2, line3, line4, line5, patch_car_interested, patch_car_test]

    for j in environment_vehicle_ID:
        globals()['patch_car_{}'.format(j)].xy=get_xy(globals()['car_{}'.format(j)],i)

        animation_list.append(globals()['patch_car_{}'.format(j)])

    return animation_list



anim=animation.FuncAnimation(fig,animate,init_func=init,frames=np.arange(0,total_length,1),
                            interval=TIME_INTERVAL,blit=True)




plt.show()
