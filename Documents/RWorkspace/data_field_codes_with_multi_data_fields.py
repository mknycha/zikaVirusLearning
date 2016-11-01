

import pandas as pd

import numpy as np



zika = pd.read_csv('zika_cleared.csv', low_memory=False)





grouped_by_code = zika.groupby('data_field_code')



data_field_codes = grouped_by_code.groups.keys()




#Returns all data_field_code records with more than 1 data_field entry

for key in data_field_codes:

    if len(grouped_by_code.get_group(key).data_field.unique()) > 1:

        print(key, grouped_by_code.get_group(key).data_field.unique())


