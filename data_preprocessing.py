import pandas as pd
# preprocess the NGSIM original data, the NGSIM data is not attached in the repository but can be downloaded from the NGSIM website

data_1=pd.read_csv('directory-of-NGSIM-data',header=None,sep="\s+", usecols=range(18),nrows=1000000,
                   names=['Vehicle_ID', 'Frame_ID','Total_Frames','Global_Time','Local_X','Local_Y','Global_X','Global_Y','v_length','v_Width',\
                          'v_Class','v_Vel','v_ACC','Lane_ID','Preceding','Following','Space_Headway','Time_Headway'])

useful_columns=['Vehicle_ID', 'Total_Frames','Global_Time','Global_X','Global_Y','v_length','v_Width','v_Class','v_Vel','v_ACC','Lane_ID','Preceding','Following','Space_Headway','Time_Headway']
data_1=data_1[useful_columns]
data_1.to_csv('/home/fangjil/Desktop/NGSIM_us101.txt',sep='\t',index=False)