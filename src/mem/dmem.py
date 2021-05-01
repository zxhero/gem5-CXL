from m5.params import *
from m5.objects.XBar import *

class DMemLinkRequester(NoncoherentXBar):
        type = 'DMemLinkRequester'
        cxx_header = "mem/dmem_link.hh"
        NID = Param.Unsigned(2, "The node ID")

class DMemLinkResponder(NoncoherentXBar):
        type = 'DMemLinkResponder'
        cxx_header = "mem/dmem_link.hh"
        NID = Param.Unsigned(1, "The node ID")

class DMemLinkRouter(NoncoherentXBar):
        type = 'DMemLinkRouter'
        cxx_header = "mem/dmem_link.hh"
        NID = Param.Unsigned(3, "The node ID")