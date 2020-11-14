from m5.params import *
from m5.objects.XBar import *

class CXLController(BaseXBar):
        type = 'CXLController'
        cxx_header = "mem/cxl_controller.hh"

class CXLDevice(NoncoherentXBar):
        type = 'CXLDevice'
        cxx_header = "mem/cxl_device.hh"