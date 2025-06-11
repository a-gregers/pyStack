#import pyThorlabsAPT.thorlabs_apt as apt
#import pyThorlabsAPT.thorlabs_apt.core as apt_core

import time
import math

class pyThorlabsAPT():

    def __init__(self,model=None):
        self.connected = False

        #Variables used to simulate a movement
        self.time_movement_started = None
        self._position = 0
        self._last_time_position_set = 0
        self._position_target = 0
        self.speed = 2
        self.stages_info = [0, 360, 2, 1]


    def list_devices(self):
        '''
        Look for any connected device

        Returns
        -------
        list_valid_devices, list
            A list of all found valid devices. Each element of the list is a list of three strings, in the format [identity,address]

        '''
        #apt_core._cleanup()                         #By calling the _cleanup method here, we make sure that list_devices can discover devices that were plugged in after the creation of this object
        #apt_core._lib = apt_core._load_library()
        #self.list_valid_devices = apt.list_available_devices()
        self.list_valid_devices = [['abc','123456'],['def','7890']]
        return self.list_valid_devices
    
    def connect_device(self,device_addr):
        #self.list_devices()
        device_addresses = [str(dev[1]) for dev in self.list_valid_devices]
        if (str(device_addr) in device_addresses):     
            try:
                #super().__init__(serial_number=int(device_addr))
                #self.motor = apt.Motor(int(device_addr))
                Msg = device_addr
                ID = 1
            except Exception as e:
                ID = 0 
                Msg = e
        else:
            raise ValueError("The specified address is not a valid device address.")
        if(ID==1):
            self.connected = True
            #self.read_parameters_upon_connection()
        return (Msg,ID)

    def disconnect_device(self):
        if(self.connected == True):
            try:   
                #apt_core._cleanup()
                self.list_devices()
                ID = 1
                Msg = 'Succesfully disconnected.'
            except Exception as e:
                ID = 0 
                Msg = e
            if(ID==1):
                self.connected = False
            return (Msg,ID)
        else:
            #apt_core._cleanup()
            self.list_devices()
            raise RuntimeError("Device is already disconnected.")
    
    #Function to simulate a real stage. They mimic the functions in core.py

    @property
    def position(self):
        if self._position_target == self._position:
            return self._position

        current_time = time.time()
        self._position =  abs(self._position-self._position_target)/(self._position_target-self._position) *self.speed*(current_time-self._last_time_position_set) + self._position
        self._last_time_position_set = time.time()
        if abs(self._position_target - self._position) <0.1:
            self._position_target = self._position
        return self._position

    @position.setter
    def position(self, value):
        self._last_time_position_set = time.time()
        self._position_target = value

    @property
    def is_in_motion(self):
        if self._position_target == self._position:
            return False
        else:
            return True

    def move_by(self, value):
        self.position = self._position + value

    def get_stage_axis_info(self):
        return self.stages_info 
    
    def stop_profiled(self):
        self._position_target = self._position

    def move_home(self):
        self.position = 0