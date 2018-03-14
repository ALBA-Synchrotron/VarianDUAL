#=============================================================================
#
# file :        MultiGauge.py
#
# description : The MultiGauge Protocol is used by many Varian devices.
#            This class implements the commands and definitions required by the protocol.
#
# project :    VacuumController Device Server
#
# $Author: srubio $
#
# copyleft :    Cells / Alba Synchrotron
#               Bellaterra
#               Spain
#
############################################################################
#
# This file is part of Tango-ds.
#
# Tango-ds is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# Tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
##########################################################################
#

class MultiGaugeProtocol(object):
    """
    The most common commands are:
    T/VC/IPCT-01 : adding new serial command, Firmware="#005?
    T/VC/IPCT-01 : adding new serial command, Remote Error="#012?
    T/VC/IPCT-01 : adding new serial command, Interlock="#013?

    T/VC/IPCT-01 : adding new serial command, HV1 P="#102?
    T/VC/IPCT-01 : adding new serial command, HV1 V="#107?
    T/VC/IPCT-01 : adding new serial command, HV1 I="#108?
    T/VC/IPCT-01 : adding new serial command, HV1Status="#130?

    T/VC/IPCT-01 : adding new serial command, HV2 P="#202?
    T/VC/IPCT-01 : adding new serial command, HV2 V="#207?
    T/VC/IPCT-01 : adding new serial command, HV2 I="#208?    
    T/VC/IPCT-01 : adding new serial command, HV2Status="#230?    
    """
    ASK = '#'
    ANSWER = '>'
    READ = '?'
    ACK = '\x06' #0x06
    NACK = '!'
    TERM = '\x0D' #0x0D

    def packMultiGauge(self, chann, comm, data=READ):
        """
        The Structure of the MultiGauge Compatible Protocol is:
        Field          No. of Bytes          Value          Description 
        Header command         1         23h         Header .#.
        Header response     1         3Eh         Header .>.
        Channel         1         30h        No Channel
                            31h        High Voltage 1
                            32h        High Voltage 2
                            33h        Gauge 1
                            34h        Gauge 2
                            35h        Serial Communication
        Command         2                 See commands description
        Data             n                 See commands description
        Terminator         1         0Dh         Carriage Return
        """
        return self.ASK + ('%1d' % chann) + ('%02d' % comm) + data + self.TERM
    
    def unpackMultiGauge(self, result, _type = int):
        if result[0]!=self.ANSWER or result[-1]!=self.TERM or len(result)<5: return None
        chann = int(result[1])
        comm = int(result[2:4])
        data = _type(result[4:-1])
        return chann, comm, data
    
    GeneralComms = {
        'Local/Remote': 10,
        'HV On/Off': 30,
        'Unit': 3,
        'Firmware': 5,
        'V Meas': 7,
        'I Meas': 8,
        'P Meas': 2,
        'Error Status': 19,
        'Serial Reset':6,
        'Device Number':1,
        'Device Type':11,
        'Remote Error':12,
        'Interlock': 13
        }
        
    DeviceTypes = """
            Spare
            500SC/Tr
            300SC/Tr
            150SC/Tr
            75-55-40SC/Tr
            20SC/Tr
            500Diode/ND
            300Diode/ND
            150Diode/ND
            75-55-40Diode/ND
            20-25Diode/ND
            """.split()        
        
        #HV on/off Command Coding 
        #Mode  Code  Description 
    OnOffCoding = {
        'ON':1, ## Any value>0 must be considered as ON!!!
        'Off': 0, #Write 0 30h HV power off
        #Write 1 31h HV power on (in compliance to the
        #Start/Protect and Fixed/Step selection made
        #using the related commands)
        #Read 0 30h HV off
        #Read 1 31h HV on
        #If full compatible MultiVac
        #Read 1 31h HV on in start/step V
        #Read 2 32h HV on in start/fixed V
        #Read 3 33h HV on in protect/step V
        #Read 4 34h HV on in protect/fixed V
        'PanelInterlock': -3, #Read -3 2Dh33h Power off caused by Interlock Panel
        'RemoteInterlock': -4, #Read -4 2Dh34h Power off caused by Remote I/O Interlock
        'CableInterlock': -5, #Read -3 2Dh33h Power off caused by Cable Interlock
        'HVTemperature': -8, #Read -8 2Dh38h Power off caused by HV Overtemperature
        'RemoteFault': -7, #Read -7 2Dh37h Power off caused by Remote I/O not Present or Remote I/O Fault
        'HVProtect': -6, #Read -6 2Dh36h Power off caused by HV Protect
        'HVShortCircuit': -7 #Read -7 2Dh37h Power off caused by HV Short Circuit
        }
        
        #Interlock Status Coding
        #BitField  Interlock type (active if 1)
    InterlockStatus = {
        'Reserved': 0x1, #01h Reserved (always 0)
        'FrontPanel': 0x2, #02h Front Panel Interlock (equal to bit 20h)
        'HV1RemoteIO': 0x4, #04h HV1 Remote I/O Interlock
        'HV1Cable': 0x8, #08h HV1 Cable interlock
        'Reserved2': 0x10, #10h Reserved (always 0)
        'FrontPanel': 0x20, #20h Front Panel Interlock (equal to bit 20h)
        'HV2RemoteIO': 0x40, #40h HV2 Remote I/O Interlock
        'HV2Cable': 0x80 #80h HV2 Cable interlock
        }
        
        #High Voltage Commands 
        #Command  Description  Mode  Channels  Format  Possible values 
    HighVoltageCommands = {
        'fixed/step':60,#fixed/step fixed/step mode Read Write HV1, HV2 Status 30h fixed 31h step
        'start/protect':61,#start/protect start/protect mode Read Write HV1, HV2 Status 30h start 31h protect
        #Psel Power supply polarity Read HV1, HV2 Status 30h negative 31h positive
        'Vmax': 63, #Vmax variable Read Write HV1, HV2 Integer (V) [3000, 7000] step 100
        #Imax Imax variable Read Write HV1, HV2 Integer (mA) [100, 400] step 10
        #Pmax Pmax variable Read Write HV1, HV2 Integer (W) [100, 400] step 10
        'Iprotect':66, #Iprotect variable Read Write HV1, HV2 Integer (mA) [10, 100] step 10
        #Vstep1 Vstep1 variable Read Write HV1, HV2 Integer (V) [3000, 7000] step 100
        #Istep1 Istep1 variable Read Write HV1, HV2 Exp. (A) [1.0E-9, 1.0E1]
        #Vstep2 Vstep2 Read Write HV1, HV2 Integer (V) [3000, 7000] step 100
        #Istep2 Istep1 variable Read Write HV1, HV2 Exp. (A) [1.0E-9, 1.0E1]
        'SetPt1':71,#SetPt1 variable (Set Point 1) Read Write HV1, HV2 Exp. (Torr) [1.0E-9, 1.0E1] (have to be greater than SP2)
        'SetPt2':72,#SetPt2 variable (Set Point 2) Read Write HV1, HV2 Exp. (Torr) [1.0E-9, 1.0E1]
        #Remote I/O Output Reads the status of the Remote I/O outputs Read HV1, HV2 BitField See Remote I/O table
        #Remote I/O Input Reads the status of the Remote I/O inputs Read HV1, HV2 BitField See Remote I/O table
        }
        
    DualControllerErrorStatus = {
        #Value  Error type  Error reference 
        #High Voltage errors 
        'HV': {
            '1': '31h High Voltage off due to front panel interlock activation Panel Interlock',
            '2': '32h High Voltage off due to Remote I/O interlock activation Remote Interlock',
            '3': '33h High Voltage off due to Cable HV interlock activation Cable Interlock',
            '4': '34h Dual fault HV not found',
            '5': '35h High Voltage off due to a general DSP determined fault HV Fault',
            '6': '36h High Voltage off due to an HV module overtemperature determined by the DSP HV Overtemperature',
            '7': '37h Remote I/O card not present or faulty R.I/O not found',
            '8': '38h Remote I/O card present, but faulty R.I/O fault',
            '9': '39h High Voltage off due to the protect function activation Protect',
            '10': '31h30h High Voltage off due to shortcircuit protection activation Short Circuit',
            '11': '31h31h High Voltage off due to an HV module overvoltage or overcurrent determined by the DSP Over Volt/Curr',
            '12': '31h32h High Voltage off due to the zero measurement protection activation Zero Meas MiniGauge errors ',
            },
        'MG': {
            '1': '31h MiniGauge off due to front panel interlock activation Panel Interlock',
            '2': '32h The selected Minigauge was not recognized Gauge not found',
            '3': '33h The Minigauge is signaling a Fault condition Gauge fault',
            '4': '34h The selected Minigauge was disconnected Gauge not connected System errors ',
            },
        'SW': {
            '1': '31h RAM failure: RAM diagnostics error Software Error',
            '2': '32h config register: incorrect value in the uC 68HC11 configuration register Software Error',
            '3': '33h test mode: invalid uC 68HC11 operating mode Software Error',
            '4': '34h copyright: violation of the signature in the ROM or the ROM was corrupted Software Error',
            '5': '35h eeprom fault: checksum or non-volatile memory write errors. Factory defaults are automatically loaded Software Error',
            '6': '36h version number: incompatible uC and Dsp versions Software Error',
            '7': '37h hv dsp not found: the Dsp does not respond during the uC initialization phase Software Error',
            '8': '38h dsp fault: the Dsp does not respond during normal operation Software Error ',
            '9': '39h invalid option: option card not configured correctly Software Error ',
            '10': '31h30h unknow option: generic execution error Software Error',
            },
        }

    ProtocolErrors = {
        #If incongruencies are detected in the composition of the
        #data packet sent to the Dual controller (in other words a
        #correct reception but an incorrect data format), the Dual
        #controller will reply with an error code identified by the
        #.!. (21h) command according to the following table.
        "1": 'Reserved (checksum error)',
        "2": 'Non existent command code',
        "3": 'Channel not valid for the selected command',
        "4": 'Write mode not allowed for the selected command',
        "5": 'Unvalid or non-congruent data transmitted',
        "6": 'Write value exceeding the allowed limits or step not allowed',
        "7": 'Data format not recognized on the protocols implemented',
        "8": 'Write not allowed to channel ON',
        "9": 'Write not allowed to channel OFF',
        ":": 'Write allowed in Serial Configuration Mode only',
        }
