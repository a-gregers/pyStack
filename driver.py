import pyThorlabsAPT.thorlabs_apt as apt
import pyThorlabsAPT.thorlabs_apt.core as apt_core
import traceback, time

"""
The class pyThorlabsAPT acts as a wrapper for apt.Motor, and inherits all its methods. It provides improved functions to find all devices, and connect and disconnect to a device 
"""

class pyThorlabsAPT(apt.Motor):

    def __init__(self,model=None):
        self.connected = False
        #Ideally, we would initialize the library here by using the first two lines of code in the list_devices method below.
        #However, the method list_devices (which might be called by the user later) needs to re-initialize the library in order to discover devices that were plugged in after that last library re-initialization
        #To avoid slowing down the start-up time, we do not initialize the library in the __init__ method, but only in the list_devices method.
        #The drawback is that the user need to call the method list_devices before connecting to a device (even if the device address is already known)

    def list_devices(self):
        '''
        Look for any connected device

        Returns
        -------
        list_valid_devices, list
            A list of all found valid devices. Each element of the list is a list of three strings, in the format [identity,address]

        '''
        self.list_valid_devices = apt.list_available_devices()
        return self.list_valid_devices
    
    def connect_device(self,device_addr):
        # self.list_devices()
        device_addresses = [str(dev[1]) for dev in self.list_valid_devices]
        if (str(device_addr) in device_addresses):     
            try:
                super().__init__(serial_number=int(device_addr))
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
        if not self.connected:
            raise RuntimeError("Device is already disconnected.")

        # Only drop the connection flag; leave the DLL loaded
        self.connected = False
        return ("Successfully disconnected.", 1)
            


    