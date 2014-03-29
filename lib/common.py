# -*- coding: utf-8 -*-

# Common code 

def check_keys (keys, dictionary):
    ''' Check if keys received are in the dictionary '''
    
    for key in keys:
        if not key in dictionary.keys():
            return False

    return True

def print_data (params):

    prints = []

    pos = "\tPosition:\n"
    pos += "\tCF-Pitch -> %s, Pitch -> %s\n" \
        % (params['imu']['p'],params['imu']['accel']['p'])
    pos += "\tCF-Roll -> %s, Roll -> %s\n" \
        % (params['imu']['r'], params['imu']['accel']['r'])
    pos += "\tCF-Heading -> %s, Heading -> %s\n" \
        % (params['imu']['h'], params['imu']['mag']['h'])
    pos += "\tAltitude -> %s" % params['imu']['a']

    prints.append (pos)

    temps = "\tTemperatures: Gyro -> %s    Accel|Mag -> %s    Alt -> %s"\
        % (params['imu']['tg'],
           params['imu']['tam'],
           params['imu']['tp'])
    prints.append (temps)

    bm = "\tCell0: Voltage -> %s    Percentage -> %s\n"\
        % (params['bm']['cell0'][0],
           params['bm']['cell0'][1])

    bm += "\tCell1: Voltage -> %s    Percentage -> %s\n"\
        % (params['bm']['cell1'][0],
           params['bm']['cell1'][1])

    bm += "\tCell2: Voltage -> %s    Percentage -> %s"\
        % (params['bm']['cell2'][0],
           params['bm']['cell2'][1])
    prints.append (bm)

    raw_data = "\tRaw data:\n"
    
    raw_data += "\tGyro: x -> %s    y -> %s    z -> %s\n" \
        % (params['imu']['gyro']['x'],
           params['imu']['gyro']['y'],
           params['imu']['gyro']['z'])
    raw_data += "\tGyro: minX -> %s    maxX -> %s    meanX -> %s\n" \
        % (params['imu']['gyro']['minX'],
           params['imu']['gyro']['maxX'],
           params['imu']['gyro']['meanX'])
    raw_data += "\tGyro: minY -> %s    maxY -> %s    meanY -> %s\n" \
        % (params['imu']['gyro']['minY'],
           params['imu']['gyro']['maxY'],
           params['imu']['gyro']['meanY'])
    raw_data += "\tGyro: minZ -> %s    maxZ -> %s    meanZ -> %s\n" \
        % (params['imu']['gyro']['minZ'],
           params['imu']['gyro']['maxZ'],
           params['imu']['gyro']['meanZ'])

    raw_data += "\tAccel: x -> %s    y -> %s    z -> %s   |A| = %s\n"\
        % (params['imu']['accel']['x'],
           params['imu']['accel']['y'],
           params['imu']['accel']['z'],
           params['imu']['accel']['a_abs'])

    raw_data += "\tMag: x -> %s    y -> %s    z -> %s\n"\
        % (params['imu']['mag']['x'],
           params['imu']['mag']['y'],
           params['imu']['mag']['z'])

    raw_data += "\tAlt: pressure -> %s"\
        % (params['imu']['alt']['p'])

    prints.append (raw_data)

    for item in prints:
        print item
    
    return

def print_dict (params, indent):

    if indent == 1:
        print "/"

    if type (params) == dict:
        for item in params.keys():
            message = ""
            for i in xrange (indent):
                # if i == indent - 1:
                #     message += "|"
                message += "|---"
                
            message += "%s" % item
            print message
            if type (params[item]) == dict:                
                print_dict (params[item], indent+1)
                
    return
