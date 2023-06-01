#!/usr/bin/env python
# coding: utf-8

# Running this script will synchronize the cameras (choses a master camera, rest are slaves) and take 60 pictures on each camera, with 3 second interval between the pictures)

# make sure to shut down all kernels and restart the kernel if we rerun to avoid access errors

# #### Part I- taken from py_scheduled_action_commands.ipynb

# In[1]:


import time
import numpy as np
import cv2
from matplotlib import pyplot as plt # pip3 install matplotlib
from arena_api.system import system
from arena_api import enums
from arena_api.buffer import BufferFactory
from arena_api.enums import PixelFormat
from arena_api.__future__.save import Writer


# In[2]:


TAB1 = "  "
TAB2 = "    "

# Delta time in nanoseconds to set action command
DELTA_TIME_NS =  3000000000 #1/fps
start_time = int(time.time())
FILE_NAME_PATTERN = f"calibration/{start_time}/<serial>/image_"
# number of images to acquire and save
NUM_IMAGES = 60
# Creating global system nodemap
sys_tl_map = system.tl_system_nodemap


# ## Create Devices & Initialize attributes

# In[3]:


"""
This function will let users know that a device is needed and
gives them a chance to connect a device instead of raising an exception
"""
tries = 0
tries_max = 6
sleep_time_secs = 10
while tries < tries_max:  # Wait for device for 60 seconds
    devices = system.create_device() #obtain list of cameras
    if not devices:
        print(
            f'Try {tries+1} of {tries_max}: waiting for {sleep_time_secs}'
            f'secs for a device to be connected!')
        for sec_count in range(sleep_time_secs):
            time.sleep(1)
            print(f'{sec_count + 1 } seconds passed ',
                    '.' * sec_count, end='\r')
        tries += 1
    else:
        break
else:
    raise Exception(f'No device found! Please connect a device and run '
                    f'the example again.')


# In[4]:


def store_initial(device):
    '''
    obtains initial attributes to restore them back after sync
    '''
    dev_map = device.nodemap
    exposure_auto_initial = dev_map['ExposureAuto'].value
    trigger_source_initial = dev_map['TriggerSource'].value
    action_uncond_initial = dev_map['ActionUnconditionalMode'].value
    action_selector_initial = dev_map['ActionSelector'].value
    action_group_key_initial = dev_map['ActionGroupKey'].value
    action_group_mask_initial = dev_map['ActionGroupMask'].value
    transfer_control_mode_initial = dev_map['TransferControlMode'].value    
    ptp_enable_initial = dev_map['PtpEnable'].value
    action_command_dev_key_initial = sys_tl_map['ActionCommandDeviceKey'].value
    action_command_grp_key_initial = sys_tl_map['ActionCommandGroupKey'].value
    action_command_grp_mask_initial = sys_tl_map['ActionCommandGroupMask'].value
    action_command_target_ip_initial = sys_tl_map['ActionCommandTargetIP'].value

    return [ exposure_auto_initial, trigger_source_initial, action_uncond_initial, 
    action_selector_initial, action_group_key_initial, action_group_mask_initial,
    transfer_control_mode_initial, ptp_enable_initial,
    action_command_dev_key_initial, action_command_grp_key_initial, 
    action_command_grp_mask_initial, action_command_target_ip_initial]


# In[5]:


def restore_initial(initial_vals, device):
    dev_map = device.nodemap

    dev_map['ExposureAuto'].value = initial_vals[0]
    dev_map['TriggerSource'].value = initial_vals[1]
    dev_map['ActionUnconditionalMode'].value = initial_vals[2]
    dev_map['ActionSelector'].value = initial_vals[3]
    dev_map['ActionGroupKey'].value = initial_vals[4]
    dev_map['ActionGroupMask'].value = initial_vals[5]
    dev_map['TransferControlMode'].value = initial_vals[6]
    dev_map['PtpEnable'].value = initial_vals[7]   
    sys_tl_map['ActionCommandDeviceKey'].value = initial_vals[8]
    sys_tl_map['ActionCommandGroupKey'].value = initial_vals[9]
    sys_tl_map['ActionCommandGroupMask'].value = initial_vals[10]
    sys_tl_map['ActionCommandTargetIP'].value = initial_vals[11]
    
    


# ## Print Devices

# In[6]:


"""
Use max supported packet size. We use transfer control to ensure that
only one camera is transmitting at a time.
"""
print(devices)
for device in devices:
    device.tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True

print(f'{TAB1}Stream Auto Negotiate Packet Size Enabled :'
        f''' {device.tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value}''')


# ## Set exposure time to the maximum

# In[7]:


"""
Manually set exposure time
    In order to get synchronized images, the exposure time
    must be synchronized.
"""
for device in devices:
    dev_map = device.nodemap
    nodes = dev_map.get_node(['ExposureAuto', 'ExposureTime'])

    nodes['ExposureAuto'].value = 'Off'

    exposure_time_node = nodes['ExposureTime']

    min_device_exposure_time = exposure_time_node.min
    max_device_exposure_time = exposure_time_node.max
    
    exposure_time_node.value = max_device_exposure_time
    
    print(f'''{TAB1}Exposure Time : {dev_map['ExposureTime'].value}''')


# In[8]:


"""
Enable trigger mode and set source to action
To trigger a single image using action commands, trigger mode must
be enabled, the source set to an action command, and the selector
set to the start of a frame.
"""
for device in devices:
    dev_map = device.nodemap

    dev_map['TriggerMode'].value = 'On'
    dev_map['TriggerSource'].value = 'Action0'
    dev_map['TriggerSelector'].value = 'FrameStart'

    #print(dev_map["AcquisitionMode"].value)
print(f'''{TAB1}Trigger Source : {dev_map['TriggerSource'].value}''')


# In[9]:


"""
Prepare the device to receive an action command
Action unconditional mode allows a camera to accept action from an
application without write access. The device key, group key, and
group mask must match similar settings in the system's TL node map.
"""
for device in devices:
    dev_map = device.nodemap

    dev_map['ActionUnconditionalMode'].value = 'On'
    dev_map['ActionSelector'].value = 0
    dev_map['ActionDeviceKey'].value = 1
    dev_map['ActionGroupKey'].value = 1
    dev_map['ActionGroupMask'].value = 1

print(f'{TAB1}Action commands: prepared')


# In[10]:


"""
Enable user controlled transfer control
Synchronized cameras will begin transmiting images at the same time.
To avoid missing packets due to collisions, we will use transfer
control to control when each camera transmits the image.
"""
for device in devices:
    dev_map = device.nodemap

    dev_map['TransferControlMode'].value = 'UserControlled'
    dev_map['TransferOperationMode'].value = 'Continuous'
    dev_map['TransferStop'].execute()

print(f'{TAB1}Transfer Control: prepared')


# #### Negotiate master/slave

# In[11]:


"""
Synchronize devices by enabling PTP
Enabling PTP on multiple devices causes them to negotiate amongst
themselves so that there is a single master device while all the
rest become slaves. The slaves' clocks all synchronize to the
master's clock.
"""
for device in devices:
    device.nodemap['PtpEnable'].value = True

print(f'''{TAB1}PTP Enabled : {device.nodemap['PtpEnable'].value}\n''')


# In[12]:


"""
Prepare the system to broadcast an action command.
The device key, group key, group mask, and target IP must all match
similar settings in the devices' node maps. The target IP acts as a mask.
"""
sys_tl_map['ActionCommandDeviceKey'].value = 1
sys_tl_map['ActionCommandGroupKey'].value = 1
sys_tl_map['ActionCommandGroupMask'].value = 1
sys_tl_map['ActionCommandTargetIP'].value = 0xFFFFFFFF  # 0.0.0.0

print(f'{TAB1}System: prepared')


# # Synchronize Cameras
# #### chooses a master and rest are slaves

# In[13]:


def synchronize_cameras():
    """
    Wait for devices to negotiate their PTP relationship
    Before starting any PTP-dependent actions, it is important to
    wait for the devices to complete their negotiation; otherwise,
    the devices may not yet be synced. Depending on the initial PTP
    state of each camera, it can take about 40 seconds for all devices
    to autonegotiate. Below, we wait for the PTP status of each device until
    there is only one 'Master' and the rest are all 'Slaves'.
    During the negotiation phase, multiple devices may initially come up as
    Master so we will wait until the ptp negotiation completes.
    """
    print(f'{TAB1}Waiting for PTP Master/Slave negotiation. '
          f'This can take up to about 40s')

    while True:
        master_found = False
        restart_sync_check = False

        for device in devices:

            ptp_status = device.nodemap['PtpStatus'].value

            # User might uncomment this line for debugging
            print(f'{device} is {ptp_status}')

            # Find master
            if ptp_status == 'Master':
                if master_found:
                    restart_sync_check = True
                    break
                master_found = True

            # Restart check until all slaves found
            elif ptp_status != 'Slave':
                restart_sync_check = True
                break

        # A single master was found and all remaining cameras are slaves
        if not restart_sync_check and master_found:
            break

        time.sleep(1)


# #### Save a bunch of images
# This version of schedule_action_commands saves each image as a PNG in the images folder

# In[14]:


#function from acquisition_single_buffer_gui
def convert_buffer_to_BGR8(buffer):
    if (buffer.pixel_format == enums.PixelFormat.BGR8):
        return buffer
    return BufferFactory.convert(buffer, enums.PixelFormat.BGR8) #must be destroyed after


# In[15]:


def get_serial(device):
    '''
    Generator function for serial number
    '''
    while True:
        yield device.nodemap.get_node("DeviceSerialNumber").value


# In[16]:


def schedule_action_command():
    """
    Set up timing and broadcast action command
    Action commands must be scheduled for a time in the future.
    This can be done by grabbing the PTP time from a device, adding
    a delta to it, and setting it as an action command's execution time.
    """
    device = devices[0]
    device.nodemap['PtpDataSetLatch'].execute()
    ptp_data_set_latch_value = device.nodemap['PtpDataSetLatchValue'].value

    # Grab image from cameras
    writer = Writer()   
    for i in range(NUM_IMAGES):
        """
        Fire action command
        Action commands are fired and broadcast to all devices, but
        only received by the devices matching desired settings.
        """
        sys_tl_map['ActionCommandExecuteTime'].value = ptp_data_set_latch_value + (i+1)*DELTA_TIME_NS
        sys_tl_map['ActionCommandFireCommand'].execute()
        imgs = []
        for device in devices:
            device.nodemap['TransferStart'].execute()  
            buffer = device.get_buffer()
            device.nodemap['TransferStop'].execute()  
            
            #save the picture
            writer.register_tag("serial", generator = get_serial(device))
            
            if i < 10:  #bad fix for the image number problem 
                writer.pattern = f'{FILE_NAME_PATTERN}0{str(i)}_<serial>.png'
            else: 
                writer.pattern = f'{FILE_NAME_PATTERN}{str(i)}_<serial>.png'
            
            #save picture as RGB
            buffer_BGR8 = convert_buffer_to_BGR8(buffer)
            writer.save(buffer_BGR8)
            device.requeue_buffer(buffer)

            buffer_bytes_per_pixel = int(len(buffer_BGR8.data)/(buffer_BGR8.width * buffer_BGR8.height))
            np_array = np.asarray(buffer_BGR8.data, dtype=np.uint8) #image data as np array
            np_array_reshaped = np_array.reshape(buffer_BGR8.height, buffer_BGR8.width, buffer_bytes_per_pixel)
            np_array_shaped_rgb = cv2.cvtColor(np_array_reshaped, cv2.COLOR_BGR2RGB)
            imgs.append(np_array_shaped_rgb)

            BufferFactory.destroy(buffer_BGR8)
    
    
        fig = plt.figure()
        rows = 2
        cols = 3
        for j in range(len(imgs)):
            fig.add_subplot(rows, cols, j+1)
            plt.imshow(imgs[j])
            
        plt.show()
        print(f"Saved pictures no. {i}")
        


# In[17]:


"""
// Demonstrates action commands
// (1) manually sets exposure, trigger and action command settings
// (2) prepares devices for action commands
// (3) synchronizes devices and fire action command
// (4) retrieves images with synchronized timestamps
"""

initial_vals_arr = []
for device in devices:
    # Get device stream nodemap
    tl_stream_nodemap = device.tl_stream_nodemap

    # Enable stream auto negotiate packet size
    tl_stream_nodemap['StreamAutoNegotiatePacketSize'].value = True

    # Enable stream packet resend
    tl_stream_nodemap['StreamPacketResendEnable'].value = True
    
    # Store initial values
    initial_vals = store_initial(device)
    initial_vals_arr.append(initial_vals)

synchronize_cameras()

print(f'{TAB1}Start stream')
for device in devices:
    device.start_stream()
    
"""
Compare timestamps
Scheduling action commands amongst PTP synchronized devices
results synchronized images with synchronized timestamps.
"""
schedule_action_command()


for device in devices:
    device.stop_stream()


for i in range(0, devices.__len__()):
    restore_initial(initial_vals_arr.pop(0), devices[i])
    print('Device settings has been reset to \'Default\' user set')

print(f'{TAB1}Stop stream and destroy all devices')
system.destroy_device()

