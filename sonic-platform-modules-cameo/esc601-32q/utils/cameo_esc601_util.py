#!/usr/bin/env python
#
# Copyright (C) 2019 Cameo Networks, Inc.


"""
Usage: %(scriptName)s [options] command object

options:
    -h | --help     : this help message
    -d | --debug    : run with debug mode
    -f | --force    : ignore error during installation or clean 
command:
    install     : install drivers and generate related sysfs nodes
    clean       : uninstall drivers and remove related sysfs nodes
    show        : show all systen status
    sff         : dump SFP eeprom
    set         : change board setting with fan|led|sfp    
"""

import os
import commands
import sys, getopt
import logging
import re
import time
import json

PROJECT_NAME = 'esc601_32q'
verbose = False
DEBUG = False
FORCE = 0

PLATFORM_INSTALL_INFO_FILE="/etc/sonic/platform_install.json"

# default is 'i2c-0', we will choose the correct one from 'i2c-0' and 'i2c-1'.
DEFAULT_BASE_BUS = 'i2c-0'
BASE_BUS = 'i2c-0'

I2C_BASE_BUS = {
    'i2c-0':{
        'path':'/sys/bus/i2c/devices/i2c-0',
        'status':'INSTALLED'
    },
    'i2c-1':{
        'path':'/sys/bus/i2c/devices/i2c-1',
        'status':'INSTALLED'
    }
}

switch_install_order = [
'PCA9548_0x73',
'PCA9548_0x74_1',
'PCA9548_0x74_2',
'PCA9548_0x74_3',
'PCA9548_0x74_4',
'PCA9548_0x77'
]

I2C_SWITCH_LIST = {
    # i2c switches
    'PCA9548_0x73': {
        'parent':'base',
        'driver':'pca9548',
        'i2caddr': '0x73',
        'path': ' ',
        'bus_map': [0,0,0,0,0,0,0,0],
        'status':'NOTINST'
    },
    'PCA9548_0x77': {
        'parent':'base',
        'driver':'pca9548',
        'i2caddr': '0x77',
        'path': ' ',
        'bus_map': [0,0,0,0,0,0,0,0],
        'status':'NOTINST'
    },
    'PCA9548_0x74_1': {
        'parent':'PCA9548_0x73',
        'parent_ch': 4,
        'driver':'pca9548',
        'i2caddr': '0x74',
        'path': ' ',
        'bus_map': [0,0,0,0,0,0,0,0],
        'status':'NOTINST'
    },
    'PCA9548_0x74_2': {
        'parent':'PCA9548_0x73',
        'parent_ch': 5,
        'driver':'pca9548',
        'i2caddr': '0x74',
        'path': ' ',
        'bus_map': [0,0,0,0,0,0,0,0],
        'status':'NOTINST'
    },
    'PCA9548_0x74_3': {
        'parent':'PCA9548_0x73',
        'parent_ch': 6,
        'driver':'pca9548',
        'i2caddr': '0x74',
        'path': ' ',
        'bus_map': [0,0,0,0,0,0,0,0],
        'status':'NOTINST'
    },
    'PCA9548_0x74_4': {
        'parent':'PCA9548_0x73',
        'parent_ch': 7,
        'driver':'pca9548',
        'i2caddr': '0x74',
        'path': ' ',
        'bus_map': [0,0,0,0,0,0,0,0],
        'status':'NOTINST'
    }
}

I2C_DEVICES = {
    # sys eeprom
    'SYS_EEPROM': {
        'parent':'base',
        'driver':'24c64smbus',
        'i2caddr': '0x56',
        'path': ' ',
        'status':'NOTINST'
    },
    # NCT7511Y sensor & fan control
    'NCT7511Y(U73)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 0,
        'driver':'nct7511',
        'i2caddr': '0x2e',
        'path': ' ',
        'status':'NOTINST'
    },
    # G781 sensors
    'G781(U94)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 1,
        'driver':'g781',
        'i2caddr': '0x4c',
        'path': ' ',
        'status':'NOTINST'
    },
    'G781(U4)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 2,
        'driver':'g781',
        'i2caddr': '0x4c',
        'path': ' ',
        'status':'NOTINST'
    },
    'G781(U34)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 3,
        'driver':'g781',
        'i2caddr': '0x4c',
        'path': ' ',
        'status':'NOTINST'
    },
    # PSU
    'PSU1': {
        'parent':'PCA9548_0x77',
        'parent_ch': 4,
        'driver':'zrh2800k2',
        'i2caddr': '0x58',
        'path': ' ',
        'status':'NOTINST'
    },
    'PSU2': {
        'parent':'PCA9548_0x77',
        'parent_ch': 4,
        'driver':'zrh2800k2',
        'i2caddr': '0x59',
        'path': ' ',
        'status':'NOTINST'
    },
    'TPS53681(0x6C)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 5,
        'driver':'tps53679',
        'i2caddr': '0x6c',
        'path': ' ',
        'status':'NOTINST'
    },
    'TPS53681(0x6E)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 5,
        'driver':'tps53679',
        'i2caddr': '0x6e',
        'path': ' ',
        'status':'NOTINST'
    },
    'TPS53681(0x70)': {
        'parent':'PCA9548_0x77',
        'parent_ch': 5,
        'driver':'tps53679',
        'i2caddr': '0x70',
        'path': ' ',
        'status':'NOTINST'
    },
    # mcp3425 adc
    'MCP3425': {
        'parent':'PCA9548_0x77',
        'parent_ch': 6,
        'driver':'mcp3425_smbus',
        'i2caddr': '0x68',
        'path': ' ',
        'status':'NOTINST'
    }
}

SFP_GROUPS = {
    'SFP-G01' :{
        'number': 8,
        'parent':'PCA9548_0x74_1',
        'channels':[0,1,2,3,4,5,6,7],
        'driver':'optoe1',
        'i2caddr': '0x50',
        'paths': [],
        'status':'NOTINST'
    },
    'SFP-G02' :{
        'number': 8,
        'parent':'PCA9548_0x74_2',
        'channels':[0,1,2,3,4,5,6,7],
        'driver':'optoe1',
        'i2caddr': '0x50',
        'paths': [],
        'status':'NOTINST'
    },
    'SFP-G03' :{
        'number': 8,
        'parent':'PCA9548_0x74_3',
        'channels':[0,1,2,3,4,5,6,7],
        'driver':'optoe1',
        'i2caddr': '0x50',
        'paths': [],
        'status':'NOTINST'
    },
    'SFP-G04' :{
        'number': 8,
        'parent':'PCA9548_0x74_4',
        'channels':[0,1,2,3,4,5,6,7],
        'driver':'optoe1',
        'i2caddr': '0x50',
        'paths': [],
        'status':'NOTINST'
    }
}



if DEBUG == True:
    print sys.argv[0]
    print 'ARGV      :', sys.argv[1:]


def main():
    global DEBUG
    global args
    global FORCE

    if len(sys.argv) < 2:
        show_help()

    options, args = getopt.getopt(sys.argv[1:], 'hdf', ['help',
                                                        'debug',
                                                        'force',
                                                        ])
    if DEBUG == True:
        print options
        print args
        print len(sys.argv)

    for opt, arg in options:
        if opt in ('-h', '--help'):
            show_help()
        elif opt in ('-d', '--debug'):
            DEBUG = True
            logging.basicConfig(level=logging.INFO)
        elif opt in ('-f', '--force'):
            FORCE = 1
        else:
            logging.info('no option')
    for arg in args:
        if arg == 'install':
            do_install()
        elif arg == 'clean':
            do_uninstall()
        elif arg == 'show':
            devices_info()
        else:
            show_help()

    return 0


def show_help():
    print __doc__ % {'scriptName': sys.argv[0].split("/")[-1]}
    sys.exit(0)


def show_set_help():
    cmd = sys.argv[0].split("/")[-1] + " " + args[0]
    print  cmd + " [led|sfp|fan]"
    print  "    use \"" + cmd + " led 0-4 \"  to set led color"
    print  "    use \"" + cmd + " fan 0-100\" to set fan duty percetage"
    print  "    use \"" + cmd + " sfp 1-54 {0|1}\" to set sfp# tx_disable"
    sys.exit(0)

def log_os_system(cmd, show):
    logging.info('Run :' + cmd)
    status, output = commands.getstatusoutput(cmd)
    logging.info(cmd + "with result:" + str(status))
    logging.info("      output:" + output)
    if status:
        logging.info('Failed :' + cmd)
        if show:
            print('Failed ({}):'.format(status) + cmd)
    return status, output


def driver_check():
    ret, lsmod = log_os_system("lsmod| grep cameo", 0)
    logging.info('mods:' + lsmod)
    if len(lsmod) == 0:
        return False
    return True


kos = [
    'depmod -a',
    'modprobe i2c_dev',
    'modprobe x86-64-cameo-esc601-32q',
    'modprobe nct7511',
    'modprobe mcp3425_smbus',
    'modprobe at24_smbus',
    'modprobe at24',
    'modprobe zrh2800k2',
    'modprobe tps53679'
]


def driver_install():
    global FORCE
    for i in range(0, len(kos)):
        status, output = log_os_system(kos[i], 1)
        if status:
            if FORCE == 0:
                return status
    return 0


def driver_uninstall():
    global FORCE
    for i in range(0, len(kos)):
        rm = kos[-(i + 1)].replace("modprobe", "modprobe -rq")
        rm = rm.replace("insmod", "rmmod")
        status, output = log_os_system(rm, 1)
        if status:
            if FORCE == 0:
                return status
    return 0

def check_base_bus():
    global I2C_SWITCH_LIST
    global I2C_DEVICES
    global SFP_GROUPS
    global BASE_BUS
    # we start check with the first i2c switch to install which on base bus
    switch = I2C_SWITCH_LIST[switch_install_order[0]]
    for bbus in I2C_BASE_BUS.keys():
        install_path = I2C_BASE_BUS[bbus]['path']
        cmd = "echo {} {} > {}/new_device".format(switch['driver'], switch['i2caddr'], install_path)
        status, output = log_os_system(cmd, 1)
        time.sleep(1)
        cmd = "ls /sys/bus/i2c/devices/{}-00{}/channel-0".format(bbus[-1],switch['i2caddr'][-2:])
        result, output = log_os_system(cmd, 1)
        #uninstall 
        cmd = "echo {} > {}/delete_device".format(switch['i2caddr'], install_path)
        status, output = log_os_system(cmd, 1)
        if result == 0:
            BASE_BUS = bbus
            break

    logging.info('Base bus is {}'.format(BASE_BUS))

    #exchange all base bus
    for dev_name in I2C_SWITCH_LIST.keys():
        if I2C_SWITCH_LIST[dev_name]['parent'] == 'base':
            I2C_SWITCH_LIST[dev_name]['parent'] = BASE_BUS
    for dev_name in I2C_DEVICES.keys():
        if I2C_DEVICES[dev_name]['parent'] == 'base':
            I2C_DEVICES[dev_name]['parent'] = BASE_BUS
    for dev_name in SFP_GROUPS.keys():
        if SFP_GROUPS[dev_name]['parent'] == 'base':
            SFP_GROUPS[dev_name]['parent'] = BASE_BUS


def get_next_bus_num():
    num_list = []
    device_list = os.listdir("/sys/bus/i2c/devices")
    for x in device_list:
        t = re.match(r'i2c-(\d+)', x)
        if t:
            num_list.append(int(t.group(1)))
    logging.info('next_bus_id is {}'.format(max(num_list)+1))
    return max(num_list)+1

def install_i2c_switch():
    
    for switch_name in switch_install_order:
        next_bus_id = get_next_bus_num()
        switch = I2C_SWITCH_LIST[switch_name]
        if switch['parent'] in I2C_BASE_BUS:
            install_path = I2C_BASE_BUS[switch['parent']]['path']
        else:
            install_path = I2C_SWITCH_LIST[switch['parent']]['path']

        if 'parent_ch' in switch:
            install_path = install_path+"/channel-{}".format(switch['parent_ch'])
            if I2C_SWITCH_LIST[switch['parent']]['status'] != 'INSTALLED':
                continue
    
        cmd = "echo {} {} > {}/new_device".format(switch['driver'], switch['i2caddr'], install_path)
        status, output = log_os_system(cmd, 1)
        if status != 0:
            switch['status'] = 'FAILED'
            continue
        
        if switch['parent'] in I2C_BASE_BUS:
            switch['path'] = "/sys/bus/i2c/devices/{}-00{}".format(switch['parent'][-1],switch['i2caddr'][-2:])
        else:
            switch['path'] = "/sys/bus/i2c/devices/{}-00{}".format(I2C_SWITCH_LIST[switch['parent']]['bus_map'][switch['parent_ch']],switch['i2caddr'][-2:])
        
        # add delay to make sure the root switch for sfp is installed completely,
        # so we can start the installation of next switch
        if switch_name == 'PCA9548_0x73':
            time.sleep(1)

        #Check if bus are actually created
        for busid in range(next_bus_id,next_bus_id+8):
            if not os.path.exists("/sys/bus/i2c/devices/i2c-{}".format(busid)):
                print("Fail to create bus when install {}".format(switch_name))
                switch['status'] = 'FAILED'
                break
        else:
            # exit loop normally; not breakout
            switch['bus_map'] = list(range(next_bus_id,next_bus_id+8))
            switch['status'] = 'INSTALLED'
        
def remove_install_status():
    if os.path.exists(PLATFORM_INSTALL_INFO_FILE):
        os.remove(PLATFORM_INSTALL_INFO_FILE)

def restore_install_status():
    output = []
    output.append(I2C_SWITCH_LIST)
    output.append(I2C_DEVICES)
    output.append(SFP_GROUPS)
    jsondata = json.dumps(output)
    with open(PLATFORM_INSTALL_INFO_FILE,'w') as fd:
        fd.write(jsondata)

def update_hwmon():
    for dev_name in I2C_DEVICES.keys():
        dev = I2C_DEVICES[dev_name]
        if dev['status'] == 'INSTALLED':
            if os.path.exists(dev['path']+'/hwmon'):
                dev['hwmon_path'] = os.path.join(dev['path']+'/hwmon', os.listdir(dev['path']+'/hwmon')[0])
    

def install_i2c_device():
    for dev_name in I2C_DEVICES.keys():
        dev = I2C_DEVICES[dev_name]
        if dev['parent'] in I2C_BASE_BUS:
            install_path = I2C_BASE_BUS[dev['parent']]['path']
        else:
            install_path = I2C_SWITCH_LIST[dev['parent']]['path']
        
        if 'parent_ch' in dev:
            install_path = install_path+"/channel-{}".format(dev['parent_ch'])
            if I2C_SWITCH_LIST[dev['parent']]['status'] != 'INSTALLED':
                continue
    
        cmd = "echo {} {} > {}/new_device".format(dev['driver'], dev['i2caddr'], install_path)
        status, output = log_os_system(cmd, 1)
        if status != 0:
            dev['status'] = 'FAILED'
            continue
 
        if dev['parent'] in I2C_BASE_BUS:
            dev['path'] = "/sys/bus/i2c/devices/{}-00{}".format(dev['parent'][-1],dev['i2caddr'][-2:])
        else:
            dev['path'] = "/sys/bus/i2c/devices/{}-00{}".format(I2C_SWITCH_LIST[dev['parent']]['bus_map'][dev['parent_ch']],dev['i2caddr'][-2:])
        
        dev['status'] = 'INSTALLED'
            

def install_sfp():
    for sfp_group_name in SFP_GROUPS.keys():
        sfp_group = SFP_GROUPS[sfp_group_name]
        if sfp_group['parent'] in I2C_BASE_BUS:
            install_path = I2C_BASE_BUS[sfp_group['parent']]['path']
        else:
            install_path = I2C_SWITCH_LIST[sfp_group['parent']]['path']
        
        # parent switch is not installed, skip this sfp group
        if I2C_SWITCH_LIST[sfp_group['parent']]['status'] != 'INSTALLED':
            sfp_group['paths'] = ['n/a']*sfp_group['number']
            continue
        
        for n in range(0,sfp_group['number']):
            sfp_install_path = install_path+"/channel-{}".format(sfp_group['channels'][n])
            cmd = "echo {} {} > {}/new_device".format(sfp_group['driver'], sfp_group['i2caddr'], sfp_install_path)
            status, output = log_os_system(cmd, 1)
            if status != 0:
                sfp_group['status'] = 'FAILED'
                sfp_group['paths'].append("n/a")
                continue
                
            if sfp_group['parent'] in I2C_BASE_BUS:
                sfp_group['paths'].append("/sys/bus/i2c/devices/{}-00{}".format(sfp_group['parent'][-1],sfp_group['i2caddr'][-2:]))
            else:
                sfp_group['paths'].append("/sys/bus/i2c/devices/{}-00{}".format(I2C_SWITCH_LIST[sfp_group['parent']]['bus_map'][sfp_group['channels'][n]],sfp_group['i2caddr'][-2:]))
        
        # if all sfps in a group are success
        if len(sfp_group['paths']) == sfp_group['number']:
            sfp_group['status'] = "INSTALLED"

def uninstall_sfp():
    for sfp_group_name in SFP_GROUPS.keys():
        sfp_group = SFP_GROUPS[sfp_group_name]
        if sfp_group['parent'] in I2C_BASE_BUS:
            uninst_path = I2C_BASE_BUS[sfp_group['parent']]['path']
        else:
            uninst_path = I2C_SWITCH_LIST[sfp_group['parent']]['path']
        
        # sfp is not installed, skip this sfp group
        if sfp_group['status'] != 'INSTALLED':
            continue
            
        for n in range(0,sfp_group['number']):
            sfp_uninst_path = uninst_path+"/channel-{}".format(sfp_group['channels'][n])
            cmd = "echo {} > {}/delete_device".format(sfp_group['i2caddr'], sfp_uninst_path)
            status, output = log_os_system(cmd, 1)
            
def uninstall_i2c_device():
    for dev_name in I2C_DEVICES.keys():
        dev = I2C_DEVICES[dev_name]
        if dev['parent'] in I2C_BASE_BUS:
            uninst_path = I2C_BASE_BUS[dev['parent']]['path']
        else:
            uninst_path = I2C_SWITCH_LIST[dev['parent']]['path']
        
        # device is not installed, skip this device
        if dev['status'] != 'INSTALLED':
            continue
                
        if 'parent_ch' in dev:
            uninst_path = uninst_path+"/channel-{}".format(dev['parent_ch'])
            
        cmd = "echo {} > {}/delete_device".format(dev['i2caddr'], uninst_path)
        status, output = log_os_system(cmd, 1)

def uninstall_i2c_switch():
    for switch_name in reversed(switch_install_order):
        switch = I2C_SWITCH_LIST[switch_name]
        if switch['parent'] in I2C_BASE_BUS:
            uninst_path = I2C_BASE_BUS[switch['parent']]['path']
        else:
            uninst_path = I2C_SWITCH_LIST[switch['parent']]['path']
        
        # switch is not installed, skip this switch
        if switch['status'] != 'INSTALLED':
            continue
        
        if 'parent_ch' in switch:
            uninst_path = uninst_path+"/channel-{}".format(switch['parent_ch'])
            
        cmd = "echo {} > {}/delete_device".format(switch['i2caddr'], uninst_path)
        status, output = log_os_system(cmd, 1)

def hw_adjustment():
    global SFP_GROUPS
    global I2C_DEVICES
    global switch_install_order
    if bmc_is_exist():
        switch_install_order.remove('PCA9548_0x77')
        for device_name in I2C_DEVICES.keys():
            device = I2C_DEVICES[device_name]
            if device['parent'] == 'PCA9548_0x77':
                device['status'] = 'viaBMC'
    
    if hwver_before_0x10():
        I2C_DEVICES['SYS_EEPROM']['driver'] = '24c04'
        if not bmc_is_exist():
            I2C_DEVICES['TPS53681(0x6C)']['parent'] = 'PCA9548_0x73'
            I2C_DEVICES['TPS53681(0x6X)']['parent_ch'] = 3
            I2C_DEVICES['TPS53681(0x6E)']['parent'] = 'PCA9548_0x73'
            I2C_DEVICES['TPS53681(0x6E)']['parent_ch'] = 3
            I2C_DEVICES['TPS53681(0x70)']['parent'] = 'PCA9548_0x73'
            I2C_DEVICES['TPS53681(0x70)']['parent_ch'] = 3

def set_led_control():
    cmd = "echo 1 > /sys/class/hwmon/hwmon2/device/ESC601_LED/led_ctrl"
    status, output = log_os_system(cmd, 1)
    if status:
        print output

def device_install():
    remove_install_status()
    hw_adjustment()
    check_base_bus()
    set_led_control()
    install_i2c_switch()
    # add delay to make sure all switch is installed completely,
    # so we can start install other slave device safely.
    time.sleep(1)
    install_i2c_device()
    install_sfp()
    update_hwmon()
    restore_install_status()
        
    
def device_uninstall():
    global SFP_GROUPS
    global I2C_DEVICES
    global I2C_SWITCH_LIST
    try:
        with open(PLATFORM_INSTALL_INFO_FILE) as fd:
            install_info = json.load(fd)
            SFP_GROUPS = install_info[2]
            I2C_DEVICES = install_info[1]
            I2C_SWITCH_LIST = install_info[0]
            uninstall_sfp()
            uninstall_i2c_device()
            uninstall_i2c_switch()
        remove_install_status()
    except IOError as e:
        print(e)
        print("Platform install information file is not exist, please do install first")
        

i2c_prefix = '/sys/bus/i2c/devices/'

def get_attr_value(attr_path):
    retval = 'ERR'
    if not os.path.isfile(attr_path):
        return retval

    try:
        with open(attr_path, 'r') as fd:
            retval = fd.read()
    except Exception as error:
        logging.error("Unable to open ", attr_path, " file !")

    retval = retval.rstrip('\r\n')
    fd.close()
    return retval

def bmc_is_exist():
    value = ''
    bmc_filePath = '/sys/class/hwmon/hwmon2/device/ESC601_BMC/bmc_present'
    if os.path.exists(bmc_filePath):
       value = get_attr_value(bmc_filePath)
       if value.find('not') < 0:
            return True
       else:
            return False
    else:
       return False

def hwver_before_0x10():
    value = ''
    filePath = '/sys/class/hwmon/hwmon2/device/ESC601_SYS/hw_version'
    if os.path.exists(filePath):
       value = get_attr_value(filePath)
       if int(value[-4:],16) < 0x10:
            return True
       else:
            return False
    else:
       return False

def do_install():
    print "Checking system...."
    if driver_check() == False:
        print "No driver, installing...."
        status = driver_install()
        if status:
            if FORCE == 0:
                return status
    else:
        print PROJECT_NAME.upper() + " drivers detected...."
    if not device_exist():
        print "No device, installing...."
        status = device_install()
        if status:
            if FORCE == 0:
                return status
    else:
        print PROJECT_NAME.upper() + " devices detected...."
    return


def do_uninstall():
    print "Checking system...."
    if not device_exist():
        print PROJECT_NAME.upper() + " has no device installed...."
    else:
        print "Removing device...."
        status = device_uninstall()
        if status:

            if FORCE == 0:
                return status

    if driver_check() == False:
        print PROJECT_NAME.upper() + " has no driver installed...."
    else:
        print "Removing installed driver...."
        status = driver_uninstall()
        if status:
            if FORCE == 0:
                return status

    return

def devices_info():
    bus_list = []
    with open(PLATFORM_INSTALL_INFO_FILE) as fd:
        install_info = json.load(fd)
        for i in range(0,2):
            for device_name in install_info[i].keys():
                device = install_info[i][device_name]
                print("{} :".format(device_name))
                if device['parent'] in I2C_BASE_BUS:
                    print("  On Bus: {}".format(device['parent']))
                else:
                    print("  On Bus: i2c-{}".format(install_info[0][device['parent']]['bus_map'][device['parent_ch']]))
                print("  i2c Address: {}".format(device['i2caddr']))
                print("  status: {}".format(device['status']))
                if device['status'] == 'INSTALLED':
                    print("  install path: {}".format(device['path']))
                    if device.get('hwmon_path'):
                        print("  hwmon_path: {}".format(device['hwmon_path']))
                print(' ')
        
        for sfp_group_name in install_info[2].keys():
            bus_list = []
            sfp_group = install_info[2][sfp_group_name]
            print("{} :".format(sfp_group_name))
            print("  sfp number: {}".format(sfp_group['number']))
            for n in range(0,sfp_group['number']):
                bus_list.append("i2c-{}".format(install_info[0][sfp_group['parent']]['bus_map'][sfp_group['channels'][n]]))
            print("  On Bus: {}".format(bus_list)) 
            print("  status: {}".format(sfp_group['status']))
            print("  install path: {}".format(', '.join(sfp_group['paths']))) 
            print(' ')
        
def device_exist():
    ret1, log = log_os_system("ls " + i2c_prefix + "*0056", 0)
    return not ret1


if __name__ == "__main__":
    main()
