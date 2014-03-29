# -*- coding: utf-8 -*-

# Collector for the IMU device
import sys
sys.path.append ("../lib")
import adc
import os
import common
import common_gpio as gpio

SKIP = True

def get_info (params):
    '''
    Get information from the battery monitor
    params: dictionary with the context needed
    '''

    if type (params) != dict:
        return (False, "Unable to get the information from the battery monitor. The params parameter is not of type dict, is of type %s" % type (params))

    keys = ['bm']
    if not common.check_keys (keys, params):
        return (False, "Unable to get the information from the battery monitor. The params parameter does not contain the necesary key value 'bm'")

    values = adc.readadc (params['bm'])
    for i in xrange (3):
        cell = values [i]
        params['bm']['cell%s'% i] = cell

    return (True, None)

def initialize (params):
    
    if type (params) != dict:
        return (False, "Unable to initialize the battery monitor. The params parameter is not of type dict, is of type %s" % type (params))
    
    # Create battery monitor context
    params['bm'] = adc.Init (vref=params['vref'], min_channel=5, max_channel=7)
    if params['bm']['error'][0]:
        return (False, "Unable to initialize the battery monitor. The gyro context could not be created. The error was %s" % BM['error'][1])

    return (True, None)
