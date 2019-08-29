import numpy as np
import os

import struct
# read textfile into string

#we are converting the txt file from DROR to binary file
#also the txt files start one number later, e.g 00000001 should be 000000000

snowless_moose_dir = "/home/wavelab/snow_removal_results/moose/2019_8_22_15_51_26_multiplier_7/velodyne_points"
snowless_moose_dir_binary = "/home/wavelab/snow_removal_results/moose/2019_8_22_15_51_26_multiplier_7"

snowless_moose_filenames = sorted(os.listdir(snowless_moose_dir))  # get all of them
print(snowless_moose_filenames)

for f in snowless_moose_filenames:


    name = int(f.split('.')[0])
    #name = f.split('.')[0]
    #print(name)
    #print("yolo")
    #name=int(name.split('_')[1])
    new_number = name - 1
    new_name = format(new_number, '010')
    filename = os.path.join(snowless_moose_dir, f)



    textstring = np.loadtxt(filename)
    #print(textstring.shape)

    data_file = open(snowless_moose_dir_binary+"/filtered/filtered_%s.bin" %new_name, 'wb')
    for p in textstring:
        data_file.write(struct.pack("4f", p[0], p[1], p[2], p[3]))

    data_file.close()

