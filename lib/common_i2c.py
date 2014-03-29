# -*- coding: utf-8 -*-

# Common code to interface with the I2C bus for the Raspberry Pi

import bitOps

class Register (object):
    '''
    Class for a register of the device that storages its address,
    a mirror of its value, its operating mode and its name.
    '''
    __addr = 0x0
    __mode = 'r'
    __name = None
    
    def __init__ (self, name, addr, mode = 'r'):
        '''
        name:         name of the register
        addr:         register address 
        mode:         readable/write mode
        '''
        self.__name = name
        self.__addr = addr
        self.__mode = mode
        
    def get_addr (self):
        '''
        Function to get the address of the register
        '''
        return self.__addr
    
    def get_mode (self):
        '''
        Function to get the mode of the register
        '''
        return self.__mode
    
    def get_name (self):
        '''
        Function to get the name of the register
        '''
        return self.__name

def __write(i2c, slaveAddr, register, mask, value, get_current_value = True):
    '''
    Function to write a value in a register given a mask
    i2c: SMBus object connected to the I2C device interface
    slaveAddr: address of the slave device
    register: instance of the class Register
    mask:     mask to set the value
    value:    value to set in the register
    '''
    if not 'w' in register.get_mode ():
        return (False, 'Access to the register %s not valid. The register is only readable.' % register.get_name())
    
    try:
        # Check if the current value has to be obtained
        if get_current_value:
            # Get the current value
            current = i2c.read_byte_data(slaveAddr, register.get_addr())  # Get current value
    
            # Get the new value
            new = bitOps.SetValueUnderMask(value, current, mask)
        else:
            new = value

        # Write the value through the i2C bus
        i2c.write_byte_data(slaveAddr, register.get_addr(), new)
    except IOError as e:
        return (False, "Unable to write data trhough the i2c. The error was: I/O error (%s): %s" %(e.errno, e.strerror))
    
    return (True, None)
        
def __read(i2c, slaveAddr, register, mask):
    '''
    Function to read a value from a register given a mask
    i2c: SMBus object connected to the I2C device interface
    slaveAddr: address of the slave device
    register: instance of the class Register
    mask:     mask to get the value
    '''
    # Get current value
    try:
        current = i2c.read_byte_data(slaveAddr, register.get_addr())   
    except IOError as e:
        return (False, "Unable to read data trhough the i2c. The error was: I/O error (%s): %s" %(e.errno, e.strerror))

    return (True, bitOps.GetValueUnderMask(current, mask))

def __read_match(i2c, slaveAddr, register, mask, dictionary):
    '''
    Function to read a value from a register given a mask
    returning the name in the dictionary
    i2c: SMBus object connected to the I2C device interface
    slaveAddr: address of the slave device
    register: instance of the class Register
    mask:     mask to get the value
    dictionary: value/name mapping
    '''
    # Get current value
    (status, current) = __read(i2c, slaveAddr, register, mask)
    if not status:
        return (False, current)

    # Return name that maps the current value
    for key in dictionary.keys():
        if dictionary[key] == current:
            return (True, key)

    return (False, "The value does not match with any value in the dictionary %s" % dictionary)

        
def __read_block(i2c, slaveAddr, start_address, nbytes):
    '''
    Function to read a block of data from a device 
    i2c: SMBus object connected to the I2C device interface
    slaveAddr: address of the slave device
    register: instance of the class Register. It is first register of
    the block of data
    nbytes: number of bytes of the block
    '''
    # Get the block of data
    if nbytes < 1:
        nbytes = 1
    try:
        block = i2c.read_i2c_block_data(slaveAddr, start_address, nbytes)   
    except IOError as e:
        return (False, "Unable to read the block of data trhough the i2c. The error was: I/O error (%s): %s" %(e.errno, e.strerror))

    return (True, block)

def __write_block(i2c, slaveAddr, start_address, values):
    '''
    Function to read a block of data from a device 
    i2c: SMBus object connected to the I2C device interface
    slaveAddr: address of the slave device
    register: instance of the class Register. It is first register of
    the block of data
    values: values of the block to write
    '''
    # Write the block of data
    try:
        i2c.write_i2c_block_data(slaveAddr, start_address, values)   
    except IOError as e:
        return (False, "Unable to read the block of data trhough the i2c. The error was: I/O error (%s): %s" %(e.errno, e.strerror))

    return (True, None)

def write(DEV, register, mask, value, get_current_value = True):
    ''' wrapper to the write function '''
    return __write (DEV['bus'], DEV['addr'], register, mask, value, get_current_value)
    
def read(DEV, register, mask):
    ''' wrapper to the corresponding common_i2c function '''
    return __read (DEV['bus'], DEV['addr'], register, mask)

def read_match(DEV, register, mask, dictionary):
    ''' wrapper to the corresponding common_i2c function '''
    return __read_match (DEV['bus'], DEV['addr'], register, mask, dictionary)

def read_block (DEV, start_address, nbytes):
    ''' wrapper to the corresponding common_i2c function '''
    return __read_block (DEV['bus'], DEV['addr'], start_address, nbytes)

def write_block (DEV, start_address, values):
    ''' wrapper to the corresponding common_i2c function '''
    return __write_block (DEV['bus'], DEV['addr'], start_address, values)
