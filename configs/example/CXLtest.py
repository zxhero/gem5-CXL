#   -----------------------------------------
#   | Host/CXL Controller                   |
#   |        ----------------------         |
#   |        |  Packing           |         |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Serial Link + Ser | * 1     |
#   |        ----------------------         |
#   |---------------------------------------
#   -----------------------------------------
#   | Device
#   |        ----------------------         |
#   |        |  Serial Link + Des | * 1     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Unpacking         | * 1     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  DRAM sim2 wrapper | * 1     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |     Memory         |         |
#   |        ----------------------         |
#   |---------------------------------------|

#   -----------------------------------------
#   | Host/CXL Controller                   |
#   |        ----------------------         |
#   |        |  Packing           |         |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Serial Link + Ser | * 1     |
#   |        ----------------------         |
#   |---------------------------------------
#   -----------------------------------------
#   | Device
#   |        ----------------------         |
#   |        |  Serial Link + Des | * 1     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Unpacking         | * 1     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |     Simple mem     |         |
#   |        ----------------------         |
#   |---------------------------------------|

#   -----------------------------------------
#   | Host/CXL Controller                   |
#   |        ----------------------         |
#   |        |  Packing           |         |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Serial Link + Ser | * 1     |
#   |        ----------------------         |
#   |---------------------------------------
#
#   -----------------------------------------
#   |               no coherent xbar        |
#   |        ----------------------         |
#   |---------------------------------------|
#
#   -----------------------------------------
#   | CXL Device                            |
#   |        ----------------------         |
#   |        |  Serial Link + Des | * 1     |
#   |        ----------------------         |
#   |        ----------------------         |
#   |        |  Unpacking         | * 1     |
#   |        ----------------------         |
#   |---------------------------------------|
#
#   |---------------------------------------|
#   |        ----------------------         |
#   |        |     Simple mem     |         |
#   |        ----------------------         |
#   |---------------------------------------|

from __future__ import print_function
from __future__ import absolute_import

import sys
import argparse

import m5
from m5.objects import *
from m5.util import *
addToPath('../')
from common import MemConfig
#from common import HMC
def add_options(parser):
    # ************CXL CONTROLLER PARAMETERS*************

    # ****************CXL DEVICE PARAMETERS***********

    # ******Noncoherent CROSSBAR PARAMETERS************
    # Flit size of the main interconnect [1]
    parser.add_option("--xbar-width", default=32, action="store", type=int,
                        help="Data width of the main XBar (Bytes)")

    # Clock frequency of the main interconnect [1]
    # This crossbar, is placed on the logic-based of the HMC and it has its
    # own voltage and clock domains, different from the DRAM dies or from the
    # host.
    parser.add_option("--xbar-frequency", default='1GHz', type=str,
                        help="Clock Frequency of the main XBar")

    # Arbitration latency of the HMC XBar [1]
    parser.add_option("--xbar-frontend-latency", default=1, action="store",
                        type=int, help="Arbitration latency of the XBar")

    # Latency to forward a packet via the interconnect [1](two levels of FIFOs
    # at the input and output of the inteconnect)
    parser.add_option("--xbar-forward-latency", default=2, action="store",
                        type=int, help="Forward latency of the XBar")

    # Latency to forward a response via the interconnect [1](two levels of
    # FIFOs at the input and output of the inteconnect)
    parser.add_option("--xbar-response-latency", default=2, action="store",
                        type=int, help="Response latency of the XBar")

    # number of cross which connects 16 Vaults to serial link[7]
    parser.add_option("--number-mem-crossbar", default=4, action="store",
                        type=int, help="Number of crossbar in HMC")

    # *****************************SERIAL LINK PARAMETERS**********************
    # Number of serial links controllers [1]
    parser.add_option("--num-links-controllers", default=4, action="store",
                        type=int, help="Number of serial links")

    # Number of packets (not flits) to store at the request side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    parser.add_option("--link-buffer-size-req", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        request side of the serial link")

    # Number of packets (not flits) to store at the response side of the serial
    #  link. This number should be adjusted to achive required bandwidth
    parser.add_option("--link-buffer-size-rsp", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        response side of the serial link")

    # Clock frequency of the each serial link(SerDes) [1]
    parser.add_option("--link-frequency", default='10GHz', type=str,
                        help="Clock Frequency of the serial links")

    # Clock frequency of serial link Controller[6]
    # clk_hmc[Mhz]= num_lanes_per_link * lane_speed [Gbits/s] /
    # data_path_width * 10^6
    # clk_hmc[Mhz]= 16 * 10 Gbps / 256 * 10^6 = 625 Mhz
    parser.add_option("--link-controller-frequency", default='625MHz',
                        type=str, help="Clock Frequency of the link\
                        controller")

    # total_ctrl_latency = link_ctrl_latency + link_latency
    # total_ctrl_latency = 4(Cycles) * 1.6 ns +  4.6 ns
    parser.add_option("--total-ctrl-latency", default='100ns', type=str,
                        help="The latency experienced by every packet\
                        regardless of size of packet")

    # Number of parallel lanes in each serial link [1]
    parser.add_option("--num-lanes-per-link", default=16, action="store",
                        type=int, help="Number of lanes per each link")

    # speed of each lane of serial link - SerDes serial interface 10 Gb/s
    parser.add_option("--serial-link-speed", default=31, action="store",
                        type=int, help="Gbs/s speed of each lane of serial\
                        link")

    # address range for each of the serial links
    parser.add_option("--serial-link-addr-range", default='1GB', type=str,
                        help="memory range for each of the serial links.\
                        Default: 1GB")

    # *****************************PERFORMANCE MONITORING*********************
    # The main monitor behind the HMC Controller
    parser.add_option("--enable-global-monitor", action="store_true",
                        help="The main monitor behind the HMC Controller")

    # The link performance monitors
    parser.add_option("--enable-link-monitor", action="store_true",
                        help="The link monitors")

    # link aggregator enable - put a cross between buffers & links
    parser.add_option("--enable-link-aggr", action="store_true", help="The\
                        crossbar between port and Link Controller")

    parser.add_option("--enable-buff-div", action="store_true",
                        help="Memory Range of Buffer is ivided between total\
                        range")

    # *******************MEMORY ARCHITECTURE **************************
    # Memory chunk for 16 vault - numbers of vault / number of crossbars
    parser.add_option("--mem-chunk", default=4, action="store", type=int,
                        help="Chunk of memory range for each cross bar in\
                        arch 0")

    # size of req buffer within crossbar, used for modelling extra latency
    # when the reuqest go to non-local vault
    parser.add_option("--xbar-buffer-size-req", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        request side of the crossbar")

    # size of response buffer within crossbar, used for modelling extra latency
    # when the response received from non-local vault
    parser.add_option("--xbar-buffer-size-resp", default=10, action="store",
                        type=int, help="Number of packets to buffer at the\
                        response side of the crossbar")


def config_cxl_subsystem(options, system):
    """
    Create the memory controllers based on the options and attach them.

    If requested, we make a multi-channel configuration of the
    selected memory controller class by creating multiple instances of
    the specific class. The individual controllers have their
    parameters set such that the address range is interleaved between
    them.
    """
    subsystem = system
    # Create a memory bus, a coherent crossbar, in this case
    #subsystem.membus = SystemXBar()
    xbar = system.membus

    # create memory ranges for the serial links
    slar0 = AddrRange(start = '0x200000000', size = '1GB')
    slar = AddrRange(start = '0x220000000', size = '512MB')
    slar2 = AddrRange(start = '0x200000000', size = '512MB')

    subsystem.cxl_controller = CXLController(
        width = 16,
        frontend_latency = 2,
        forward_latency = 3,
        response_latency = 3,
    )
    subsystem.cxl_device = CXLDevice(
        width = 16,
        frontend_latency = 2,
        forward_latency = 2,
        response_latency = 4,
    )
    subsystem.cxl_controller.seriallink = SerialLink(ranges=slar0,
                                        req_size=options.link_buffer_size_req,
                                        resp_size=options.link_buffer_size_rsp,
                                        num_lanes=options.num_lanes_per_link,
                                        link_speed=options.serial_link_speed,
                                        delay=options.total_ctrl_latency)
    subsystem.cxl_device.seriallink = SerialLink(ranges=slar,
                                        req_size=options.link_buffer_size_req,
                                        resp_size=options.link_buffer_size_rsp,
                                        num_lanes=options.num_lanes_per_link,
                                        link_speed=options.serial_link_speed,
                                        delay=options.total_ctrl_latency)
    subsystem.pciexbar = CXLXBar(
        width = 16,
        frontend_latency = 2,
        forward_latency = 1,
        response_latency = 2,
    )
    subsystem.pciexbar2 = CXLXBar(
        width = 16,
        frontend_latency = 2,
        forward_latency = 1,
        response_latency = 2,
    )
    subsystem.cxl_device2 = CXLDevice(
        width = 16,
        frontend_latency = 2,
        forward_latency = 2,
        response_latency = 4,
    )
    subsystem.cxl_device2.seriallink = SerialLink(ranges=slar2,
                                        req_size=options.link_buffer_size_req,
                                        resp_size=options.link_buffer_size_rsp,
                                        num_lanes=options.num_lanes_per_link,
                                        link_speed=options.serial_link_speed,
                                        delay=options.total_ctrl_latency)
    subsystem.pciexbar2.seriallink = SerialLink(ranges=slar2,
                                        req_size=options.link_buffer_size_req,
                                        resp_size=options.link_buffer_size_rsp,
                                        num_lanes=options.num_lanes_per_link,
                                        link_speed=options.serial_link_speed,
                                        delay=options.total_ctrl_latency)

    subsystem.cxl_controller.monitor = CommMonitor()

    xbar.mem_side_ports = subsystem.cxl_controller.cpu_side_ports
    sl = subsystem.cxl_controller.seriallink
    subsystem.cxl_controller.mem_side_ports = subsystem.cxl_controller.monitor.cpu_side_port
    subsystem.cxl_controller.monitor.mem_side_port = sl.cpu_side_port
    sl.mem_side_port = subsystem.pciexbar.cpu_side_ports

    #cxl subsystem 1
    sl2 = subsystem.cxl_device.seriallink
    subsystem.pciexbar.mem_side_ports = sl2.cpu_side_port
    sl2.mem_side_port = subsystem.cxl_device.cpu_side_ports

    #cxl subsystem 2
    sl3 = subsystem.pciexbar2.seriallink
    sl4 = subsystem.cxl_device2.seriallink
    subsystem.pciexbar.mem_side_ports = sl3.cpu_side_port
    sl3.mem_side_port = subsystem.pciexbar2.cpu_side_ports
    subsystem.pciexbar2.mem_side_ports = sl4.cpu_side_port
    sl4.mem_side_port = subsystem.cxl_device2.cpu_side_ports

    system.mem_ctrl = MemCtrl()
    mc = system.mem_ctrl
    mc.dram = DDR3_1600_8x8()
    mc.dram.range = AddrRange(start = '0x220000000', size = '512MB')
    mc.port = subsystem.cxl_device.mem_side_ports

    system.mem_ctrl2 = MemCtrl()
    mc2 = system.mem_ctrl2
    mc2.dram = DDR3_1600_8x8()
    mc2.dram.range = AddrRange(start = '0x200000000', size = '512MB')
    subsystem.cxl_device2.mem_side_ports = mc2.port

class L1Cache(Cache):
    """Simple L1 Cache with default values"""

    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    def __init__(self, options=None):
        super(L1Cache, self).__init__()
        pass

    def connectBus(self, bus):
        """Connect this cache to a memory-side bus"""
        self.mem_side = bus.cpu_side_ports

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU-side port
           This must be defined in a subclass"""
        raise NotImplementedError

class L1ICache(L1Cache):
    """Simple L1 instruction cache with default values"""

    # Set the default size
    size = '16kB'

    def __init__(self, opts=None):
        super(L1ICache, self).__init__(opts)

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU icache port"""
        self.cpu_side = cpu.icache_port

class L1DCache(L1Cache):
    """Simple L1 data cache with default values"""

    # Set the default size
    size = '64kB'

    def __init__(self, opts=None):
        super(L1DCache, self).__init__(opts)

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU dcache port"""
        self.cpu_side = cpu.dcache_port

class L2Cache(Cache):
    """Simple L2 Cache with default values"""

    # Default parameters
    size = '256kB'
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    def __init__(self, opts=None):
        super(L2Cache, self).__init__()

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports

#pd = "Simple 'hello world' example using HMC as main memory"
#parser = argparse.ArgumentParser(description=pd)
#add_options(parser)
#options = parser.parse_args()
## create the system we are going to simulate
#system = makeLinuxX86System(mem_mode='timing')
##System()
## use timing mode for the interaction between master-slave ports
##system.mem_mode = 'timing'
## set the clock fequency of the system
#clk = '1GHz'
#vd = VoltageDomain(voltage='1V')
#system.clk_domain = SrcClockDomain(clock=clk, voltage_domain=vd)
## create a simple CPU
#system.cpu = TimingSimpleCPU()
#
## Create an L1 instruction and data cache
#system.cpu.icache = L1ICache(options)
#system.cpu.dcache = L1DCache(options)
#
## Connect the instruction and data caches to the CPU
#system.cpu.icache.connectCPU(system.cpu)
#system.cpu.dcache.connectCPU(system.cpu)
#
## Create a memory bus, a coherent crossbar, in this case
#system.l2bus = L2XBar()
#
## Hook the CPU ports up to the l2bus
#system.cpu.icache.connectBus(system.l2bus)
#system.cpu.dcache.connectBus(system.l2bus)
#
## Create an L2 cache and connect it to the l2bus
#system.l2cache = L2Cache(options)
#system.l2cache.connectCPUSideBus(system.l2bus)
#
## config memory system
#config_cxl_subsystem(options, system)
#
## Connect the L2 cache to the membus
#system.l2cache.connectMemSideBus(system.membus)
#
## create the interrupt controller for the CPU and connect to the membus
#system.cpu.createInterruptController()
#system.cpu.interrupts[0].pio = system.membus.master
#system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
#system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports
## connect special port in the system to the membus. This port is a
## functional-only port to allow the system to read and write memory.
#system.system_port = system.membus.cpu_side_ports
## get ISA for the binary to run.
#isa = 'x86'
## run 'hello' and use the compiled ISA to find the binary
#binary = 'tests/test-progs/hello/bin/' + isa + '/linux/hello'
## create a process for a simple "Hello World" application
#process = Process()
## cmd is a list which begins with the executable (like argv)
#process.cmd = [binary]
## set the cpu workload
#system.cpu.workload = process
## create thread contexts
#system.cpu.createThreads()
## set up the root SimObject
#root = Root(full_system=False, system=system)
#m5.instantiate()
#
#print("Beginning simulation!")
#exit_event = m5.simulate()
#print('Exiting @ tick %i because %s' % (m5.curTick(), exit_event.getCause()))