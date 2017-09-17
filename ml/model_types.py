#!/usr/bin/env python

"""
This function maps a model_num to the necessary data needed to load samples from the mongo database and from json files.

The most important models are:
0 - first task (curves)
1 - second task without foot
2 - second task with foot
10 - 45 degree task
11 - 30 degree task
12 - 15 degree task

There are some additional ones used for testing:
3 - uses object2 instead of object_l/r
6 - curves with sm/ml
20/21/22 - like 10/11/12 but without l/r

@param model_num: identifier of the model used
@param load_test: if true, the test data is loaded (should only be done once at the end)
@return the labels (classes) of the neural network, the sample_ids needed to load the data from the mongoDB, and the filename to load from/save to a json file
"""


def get_labels(model_num, load_test=False):
    labels = []
    sample_ids = [  1000, 2000, 
                    1001, 2001, 3001, 
                    1002, 2002, 3002, 
                    1003, 2003, 3003, 4003, 4103,
                    1004, 2004, 2104, 3004, 4004,
                    1010, 2010, 3010, 4010,
                    1005, 1007, 2005, 3005,
                    1006, 1009, 2006, 3006,
                    1012, 1021, 1023, 1032]
    angle_ids = [100000,101000,102000,103000,1000000,104000,105000,106000]

    if model_num == 0:
        labels = ['straight','curve_s_l','curve_s_r','curve_m_l','curve_m_r','curve_l_l','curve_l_r']

    elif model_num == 1:
        labels = ['straight','curve_m_l','curve_m_r','wall','object_l','object_r']

    elif model_num == 2 or model_num == 102:
        labels = ['straight','curve_m_l','curve_m_r','wall','foot','object_l','object_r']

    elif model_num == 3:
        labels = ['straight','curve_m_l','curve_m_r','wall','object2']
        
    elif model_num == 6:
        labels = ['straight','curve_sm_l','curve_ml_l','curve_sm_r','curve_ml_r']

    elif model_num in [10,20]:
        sample_ids_old = [9001, 9101, 9002, 9102, 9003, 9103, 9004, 9104, 9005, 9105, 9006, 9106, 9107, 9008]
        sample_ids_fail = []

        labels = ['angle_0', 'angle_180']
        sample_ids = []
        for i in range(0,181,45):
            new_s = []
            for s in angle_ids:
                new_s.append(s+i)
                new_s.append(-s-i)
            for s in new_s:
                if s not in sample_ids_fail: sample_ids.append(s)
            if i != 0 and i != 180:
                if model_num >= 20:
                    labels.append('angle_%s' % i)
                else:
                    labels.append('angle_%s_l' % i)
                    labels.append('angle_%s_r' % i)
        sample_ids.extend(sample_ids_old)
        
    elif model_num in [11,21]:
        sample_ids_old = [9001, 9101, 9002, 9102, 9003, 9103, 9004, 9104, 9005, 9105, 9006, 9106, 9107, 9008]
        sample_ids_fail = []

        labels = ['angle_0', 'angle_180']
        sample_ids = []
        for i in range(0,181,30):
            new_s = []
            for s in angle_ids:
                new_s.append(s+i)
                new_s.append(-s-i)
            for s in new_s:
                if s not in sample_ids_fail: sample_ids.append(s)
            if i != 0 and i != 180:
                if model_num >= 20:
                    labels.append('angle_%s' % i)
                else:
                    labels.append('angle_%s_l' % i)
                    labels.append('angle_%s_r' % i)
        sample_ids.extend(sample_ids_old)
    
    elif model_num in [12,22]:
        sample_ids_old = [9001, 9101, 9002, 9102, 9003, 9103, 9004, 9104, 9005, 9105, 9006, 9106, 9107, 9008]
        sample_ids_fail = []

        labels = ['angle_0', 'angle_180']
        sample_ids = []
        for i in range(0,181,15):
            new_s = []
            for s in angle_ids:
                new_s.append(s+i)
                new_s.append(-s-i)
            for s in new_s:
                if s not in sample_ids_fail: sample_ids.append(s)
            if i != 0 and i != 180:
                if model_num >= 20:
                    labels.append('angle_%s' % i)
                else:
                    labels.append('angle_%s_l' % i)
                    labels.append('angle_%s_r' % i)
        sample_ids.extend(sample_ids_old)
    else:
        raise ValueError('Unknown model')

    if load_test: sample_ids = [1]

    return labels, sample_ids, ('data_%s%s.json' % (model_num, '_test' if load_test else ''))































