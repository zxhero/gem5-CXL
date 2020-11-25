from m5.params import *
from m5.objects.XBar import *

class CXLController(BaseXBar):
        type = 'CXLController'
        cxx_header = "mem/cxl_controller.hh"

class CXLDevice(BaseXBar):
        type = 'CXLDevice'
        cxx_header = "mem/cxl_device.hh"

class CXLXBar(BaseXBar):
        type = 'CXLXBar'
        cxx_header = "mem/cxlxbar.hh"