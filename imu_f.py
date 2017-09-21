#!/usr/bin/env python

"""
This script provides fakes an imu, by sending random data.
"""
import numpy as np
def get_vals():
    data = np.random.normal(0, 1, 6)
    return [{'accel':  list(data)[:3], 'gyro': list(data)[3:] }]


def stop():
    return 0

