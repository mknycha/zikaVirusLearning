import pandas as pd
import numpy as np

zika = pd.read_csv('cdc_zika.csv', low_memory=False)
zika.report_date[pd.isnull(zika.report_date)] #Identify missing records
zika.drop(['time_period', 'time_period_type'], inplace = True, axis=1)#Drop columns which have all missing values
zika.dropna(axis=0, how='any', inplace=True)#Drop rows with all missings - there is no rows with only some values missing
zika.report_date.unique()#Check dates format - underscore is used insted of '-'
zika.report_date = zika.report_date.str.replace('_', '-')#Replace underscores with '-'
zika.report_date.unique()
zika.report_date = pd.to_datetime(zika.report_date)#Change date formatting
zika.location = zika.location.str.lower()
zika.data_field = zika.data_field.str.lower()
zika.drop_duplicates(inplace=True)#Dropping duplicated rows
zika.to_csv('zika_cleared.csv')
