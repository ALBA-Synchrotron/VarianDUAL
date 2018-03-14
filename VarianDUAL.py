#    "$Name:  $";
#    "$Header: /siciliarep/CVS/tango_ds/Vacuum/VacuumController/VarianDUAL.py,v 1.3 2007/08/28 10:39:17 srubio Exp $";
#=============================================================================
#
# file :        VarianDUAL.py
#
# description :
#
# project :    VacuumController Device Server
#
# $Author: srubio $
#
# $Revision: 12404 $
#
# $Log: VarianDUAL.py,v $
# Revision 1.3  2007/08/28 10:39:17  srubio
# Vacuum Controller modified to access Gauges as independent Devices
#
# Revision 1.2  2007/07/20 10:20:50  srubio
# Lot of things improved in communications and Exception management, also VarianDUAL first compatibilities added.
#
# Revision 1.1  2007/07/09 09:56:50  sicilia
# VarianDUAL, first version, does nothing
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

import sys
import inspect
import time
import re
import operator
import traceback
import threading

import PyTango
from PyTango import DevState,DevFailed

#from PyTango_utils.excepts import ExceptionWrapper,Catched,Catched2
from fandango.excepts import getLastException
from fandango.device import Dev4Tango,TimedQueue
from fandango.functional import isString,isSequence,toSequence
from fandango.objects import self_locked

## @note Backward compatibility between PyTango3 and PyTango7
if 'PyDeviceClass' not in dir(PyTango): PyTango.PyDeviceClass = PyTango.DeviceClass
if 'PyUtil' not in dir(PyTango): PyTango.PyUtil = PyTango.Util
if 'Device_4Impl' not in dir(PyTango): PyTango.Device_4Impl = PyTango.Device_3Impl

from MultiGauge import MultiGaugeProtocol
from VacuumController import *

#==================================================================
#   VarianDUAL Class Description:
#
#         <p>This device requires <a href="http://www.tango-controls.org/Documents/tools/fandango/fandango">Fandango module<a> to be available in the PYTHONPATH.</p>
#         This Device will manage the Varian DUAL Ion Pump Controller.
#         It will read Pressure, Voltage and Current for each Channel, Firmware version and relays state.
#         It has a background thread (SerialVacuumDevice) permanently polling the HW for new values, 
#         sending commands will interrupt this thread to perform synchronous actions.
#
#         The right CVS command to download it is:
#         cvs -d:pserver:anonymous@tango-ds.cvs.sourceforge.net:/cvsroot/tango-ds co    -r first    VacuumController
#
#==================================================================
#     Device States Description:
#
#   DevState.ON : Everything works fine
#   DevState.OFF : Both channels switched off
#   DevState.INIT : Hardware values not readed yet
#   DevState.UNKNOWN : It's not possible to communicate
#   DevState.MOVING: Current or Voltage values are changing
#   DevState.ALARM : Pressure Interlock Enabled, or Channels status differ from expected!
#   DevState.FAULT : Both cable interlock enabled!
#   DevState.DISABLE : Manual interlock enabled
#==================================================================


#class VarianDUAL(PyTango.Device_3Impl,Logger):
class VarianDUAL(Dev4Tango, MultiGaugeProtocol):
    """
    #         This Device will manage the Varian DUAL Ion Pump Controller.
    #         It will read Pressure, Voltage and Current for each Channel and Firmware version and relays state.
    #         It has a background thread (SerialVacuumDevice) permanently polling the HW for new values, 
    #         sending commands will interrupt this thread to perform synchronous actions.
    #         Last update: srubio@cells.es, 2007/09/20    
    """

    #--------- Add you global variables here --------------------------
    #State Machine methods
    @self_locked
    def is_Attr_allowed(self, req_type): 
        self.debug('In is_Attr_allowed ...')
        return bool(self.SVD and self.SVD.errors<len(self.SVD.readList) and  self.get_state() not in [PyTango.DevState.UNKNOWN] and self.SVD.init)#,PyTango.DevState.INIT] )
    
    is_HV1Status_allowed=is_Attr_allowed
    is_HV2Status_allowed=is_Attr_allowed
    is_V1_allowed=is_Attr_allowed
    is_V2_allowed=is_Attr_allowed
    is_I1_allowed=is_Attr_allowed
    is_I2_allowed=is_Attr_allowed
    is_P1_allowed=is_Attr_allowed
    is_P2_allowed=is_Attr_allowed
    is_FirmwareVersion_allowed=is_Attr_allowed
    is_Interlock_allowed=is_Attr_allowed
    is_Protocol_allowed=is_Attr_allowed
    
    #StartSequence Management
    #StartSequence = SERIAL, STEP, ON, PROTECT
    #each key will be a method callable previously added to a dictionary
    #e.g. ON1 = lambda: self.OnHv1()
    #e.g. SERIAL = lambda: self.SetMode('serial')
    #And for each element we will execute d['SERIAL'].__call__()
    #As several commands are causing Controller Death ... ON1 and ON2 must be specified instead of using ON for All.
    def WarmUp(self):
        self.debug('In WarmUp() = executeStartSequence ...')
        funcs = {}
        funcs['SERIAL']=lambda: self.SetMode('serial')
        funcs['LOCAL']=lambda: self.SetMode('local')
        funcs['PROTECT']=lambda: self.SetMode('protect')
        funcs['START']=lambda: self.SetMode('start')
        funcs['FIXED']=lambda: self.SetMode('fixed')
        funcs['STEP']=lambda: self.SetMode('step')
        funcs['ON1']=lambda: self.OnHV1()
        funcs['ON2']=lambda: self.OnHV2()
        funcs['OFF1']=lambda: self.OffHV1()
        funcs['OFF2']=lambda: self.OffHV2()
        if len(self.StartSequence)==1 and not self.StartSequence[0].startswith('#') and ',' in self.StartSequence: 
            self.StartSequence=self.StartSequence[0].split(',')
        self.StartSequence = filter(bool,(l.split('#',1)[0].strip() for l in self.StartSequence))
        if self.StartSequence:
            self.info('-'*80)
            self.info('In WarmUp() = executeStartSequence (%s)'%self.StartSequence)
            for s in self.StartSequence:
                self.info(s)
                s,c = s.split(':',1) if ':' in s else (s,'')
                try: c = True if not c else fandango.TangoEval().eval(c)
                except: c = False
                self.info('\t%s:%s'%(s,c))
                if s in funcs.keys() and callable(funcs[s]) and c:
                    self.info('In WarmUp() ... Executing %s'%s)
                    funcs[s].__call__()
                else:
                    self.info('... unknown %s'%s)
        return '\n'.join(self.StartSequence)
            
    def SetStartSequence(self,argin):
        if argin:
            if argin.upper()=='DELETE': self.StartSequence=[]
            else: self.StartSequence=argin
            db=PyTango.Database()
            db.put_device_property(self.get_name(),{'StartSequence':self.StartSequence})
        return self.StartSequence
            
    def getModeLocal(self):
        attr_Mode_read = 'Unknown'
        st = self.readCommand('ModeLocal',int)
        if st in range(3):
            attr_Mode_read=['LOCAL','REMOTE','SERIAL'][st]
        return attr_Mode_read
            
    def AsciiChecksum(self,argin):
        return '%04d'%sum(ord(i) for i in str(argin))
    
    def writeCommand(self, comm, argin, mode=False, local=False):
        """ This method provides a generic function for executing write-only commands """
        print '>'*80
        if mode:
            local = time.time()>=self.last_serial_change+60 and self.ForceLocal
            if not 'SERIAL' in self.getModeLocal(): self.SetMode('serial')
        else: local = False
        try:
            self.info('In writeCommand(%s,%s)'%(comm,argin))
            self.SVD.addComm(argin,argin)
        except Exception, e:
            exc = traceback.format_exc()
            self.error('Exception in writeCommand: '+exc)
            self.exception='Exception in writeCommand: '+str(e)
            raise Exception('Exception%s()'%comm, exc)
        if mode and local: self.SetMode('local')
        print '<'*80
        
    REGEXPS = {
        bool:'[01]',
        int:'-?[0-9]',
        long:'[0-9]{4,5}',
        float:'[0-9]\.[0-9]E[+-][0-9]{2,2}',
        str:'.*'
        }
    
    def readCommand(self, comm, _type):
        """
        Type          No. of Bytes.         Description 
        Read             1 .        ?. (3Fh3) performs reads on the Dual controller
        Status             1 .        0. (30h) = false = off,.1. (31h) = true = on 5
        Integer         5 .        xxxxx. represented in BCD4 on 5 digits (always positive)
        BitField         8         Like the integer type, but with meanings associated to the number.s single bits
        Exponential         7         .x.xEsxx. where x is BCD digits, E is the 45h character and s is the (.+. o .-. sign
        String             n         Sequence of na  characters included within the 20h and 7Fh range
        """
        result=""
        self.debug('readCommand(%s)'%comm)
        try:
            result = self.SVD.getComm(self.HVComms[comm])
            ##OJORL! The 3 first characters are discarded (not 4). The initial '>' is discarded by the SVD class.
            if result is not None: 
                self.debug('readCommand(%s): Data(%d,%s) readed: "%s"'%(comm,len(result)-3,result,result[3:]))
                result=result[3:]
            
            if result is not None and len(result):
                if re.match(self.REGEXPS[_type],result):
                    if _type is bool: _type=int
                    self.exception = ''
                    return _type(result)
                elif hasattr(self,'read_Missreadings'): 
                    self.read_Missreadings(value=result)
                    print 'Missreadings are '+str(self.missreadings)
                    PyTango.Except.throw_exception('VarianDUAL_CommFailed','Hardware read failed','VarianDUAL.readCommand(%s,%s)=%s'%(comm,str(_type),result))
            else:
                PyTango.Except.throw_exception("VarianDUAL_ValueNotUpdatedException",'Last %s read was an empty string.'%str(comm),'VarianDUAL.readCommand(%s)'%str(comm))
        
        except Exception, e:
            self.error('Exception in readCommand: '+str(e))
            #print getLastException()
            self.exception='\nLast exception: '+str(e)
            PyTango.Except.throw_exception("VarianDUAL_readCommand Exception",str(e),str(e))    
            #PyTango.Except.re_throw_exception(e,"VarianDUAL","read_"+comm+"()",result)
    
#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------
    def __init__(self,cl, name):
        #PyTango.Device_3Impl.__init__(self,cl,name)
        Dev4Tango.__init__(self,cl,name)
        VarianDUAL.init_device(self)

#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        self.info("[Device delete_device method] for device%s"%self.get_name())
        if self.SVD: self.SVD.stop()
        del self.SVD

#------------------------------------------------------------------
#    Device initialization
#------------------------------------------------------------------
    def init_device(self):
        print "In "+self.get_name()+"::init_device()"
        self.set_state(PyTango.DevState.UNKNOWN)
        self.init_my_Logger()
        self.get_device_properties(self.get_device_class())

        self.HV1Status,self.HV2Status=None,None
        self.prevHV1Code=0,1
        self.prevHV2Code=0,1
        self.voltages = [[0,0],[0,0]] #old1,new1,old2,new2
        self.currents = [[1e-9,1e-9],[1e-9,1e-9]] #old1,new1,old2,new2
        self.exception,self.init_error,self.comms_report,self.channelstatus,self.oscillation='','','','',''
        self.last_state_change=0
        self.last_serial_change=0
        self.event = threading.Event()
        try:
            if not hasattr(self,'LogLevel'): self.LogLevel = 'INFO'
            self.info(''.join(("In ", self.get_name(), "::init_device(%s)"%self.LogLevel)))
            self.startTime = time.time()
            self.statesQueue = TimedQueue(self.get_state())
            if not hasattr(self,'Refresh') or not self.Refresh:
                self.warning('Refresh attribute does not exists!')
                self.Refresh=3.
            elif self.Refresh>5.:
                self.info('Refresh period is limited to 5. seconds.')
                self.Refresh = 5.
            self.info('Refresh pause between connections set to %s seconds.'%self.Refresh)
            
            if not self.SerialLine:
                self.set_state(DevState.FAULT)
                self.set_status('SerialLine property requires a value!')
                self.error('SerialLine property requires a value!')
                #raise RuntimeError, str('SerialLine property requires a value!')
                self.SVD = None
            else:
                #The arguments for SerialVacuumDevice are:
                #    tangoDevice=SerialLineName, period=minimum time between communications, wait=time waiting for answer
                from VacuumController import SerialVacuumDevice
                SerialVacuumDevice.LogLevel = self.LogLevel
                self.SVD=SerialVacuumDevice(
                    tangoDevice=self.SerialLine,
                    period=self.Refresh,
                    wait=0.2,
                    retries=3,
                    log=self.LogLevel,
                    blackbox=self.BlackBox)
                self.HVComms = {}
                def addCommand(name,command,polling = 0):
                    self.HVComms[name] = command
                    if polling>0:
                        self.SVD.addComm(command)
                        self.SVD.setPolledComm(command,polling)
                    return
                    
                # READ COMMANDS #VARIAN DUAL PROTOCOL: MultiGauge Compatible (No Checksum)
                addCommand('HV1 V',self.packMultiGauge(1,self.GeneralComms['V Meas'],'?'),1.)
                addCommand('HV1 I',self.packMultiGauge(1,self.GeneralComms['I Meas'],'?'),1.)
                addCommand('HV1 P',self.packMultiGauge(1,self.GeneralComms['P Meas'],'?'),1.)
                addCommand('HV2 V',self.packMultiGauge(2,self.GeneralComms['V Meas'],'?'),1.)
                addCommand('HV2 I',self.packMultiGauge(2,self.GeneralComms['I Meas'],'?'),1.)
                addCommand('HV2 P',self.packMultiGauge(2,self.GeneralComms['P Meas'],'?'),1.)        
                
                addCommand('ModeLocal',self.packMultiGauge(0,self.GeneralComms['Local/Remote'],'?'),10.)
                addCommand('Remote Error',self.packMultiGauge(0,self.GeneralComms['Remote Error'],'?'),10.)
                addCommand('Interlock',self.packMultiGauge(0,self.GeneralComms['Interlock'],'?'),10.)
                addCommand('ErrorStatus',self.packMultiGauge(1,self.GeneralComms['Error Status'],'?'),10.)
                addCommand('HV1Status',self.packMultiGauge(1,self.GeneralComms['HV On/Off'],'?'),10.)
                addCommand('HV2Status',self.packMultiGauge(2,self.GeneralComms['HV On/Off'],'?'),10.)
                addCommand('HV1Step',self.packMultiGauge(1,self.HighVoltageCommands['fixed/step'],'?'),10.)
                addCommand('HV1Protect',self.packMultiGauge(1,self.HighVoltageCommands['start/protect'],'?'),10.)
                addCommand('HV2Step',self.packMultiGauge(2,self.HighVoltageCommands['fixed/step'],'?'),10.)
                addCommand('HV2Protect',self.packMultiGauge(2,self.HighVoltageCommands['start/protect'],'?'),10.)
                addCommand('HV1 IProtect',self.packMultiGauge(1,self.HighVoltageCommands['Iprotect'],'?'),10.)
                addCommand('HV2 IProtect',self.packMultiGauge(2,self.HighVoltageCommands['Iprotect'],'?'),10.)
                addCommand('HV1 PSetPoint',self.packMultiGauge(1,self.HighVoltageCommands['SetPt1'],'?'),10.)
                addCommand('HV2 PSetPoint',self.packMultiGauge(2,self.HighVoltageCommands['SetPt1'],'?'),10.)                
                
                addCommand('Firmware',self.packMultiGauge(0,self.GeneralComms['Firmware'],'?'),60.)
                addCommand('Pump1',self.packMultiGauge(1,self.GeneralComms['Device Type'],'?'),60.)
                addCommand('Pump2',self.packMultiGauge(2,self.GeneralComms['Device Type'],'?'),60.)
                
                # WRITE COMMANDS
                # THESE COMMANDS MUST NOT BE ADDED TO THE COMMON COMMANDS LIST (so will not be polled)!!!!!
                self.HVComms['Serial On'] = self.packMultiGauge(0,self.GeneralComms['Local/Remote'],'2')
                self.HVComms['Serial Off'] = self.packMultiGauge(0,self.GeneralComms['Local/Remote'],'0')
                self.HVComms['HV1 On'] = self.packMultiGauge(1,self.GeneralComms['HV On/Off'],'1')
                self.HVComms['HV1 Off'] = self.packMultiGauge(1,self.GeneralComms['HV On/Off'],'0')
                self.HVComms['HV2 On'] = self.packMultiGauge(2,self.GeneralComms['HV On/Off'],'1')
                self.HVComms['HV2 Off'] = self.packMultiGauge(2,self.GeneralComms['HV On/Off'],'0')
                self.HVComms['HV1setFixed'] = self.packMultiGauge(1,self.HighVoltageCommands['fixed/step'],'0')
                self.HVComms['HV1setStep'] = self.packMultiGauge(1,self.HighVoltageCommands['fixed/step'],'1')
                self.HVComms['HV1setStart'] = self.packMultiGauge(1,self.HighVoltageCommands['start/protect'],'0')
                self.HVComms['HV1setProtect'] = self.packMultiGauge(1,self.HighVoltageCommands['start/protect'],'1')
                self.HVComms['HV2setFixed'] = self.packMultiGauge(2,self.HighVoltageCommands['fixed/step'],'0')
                self.HVComms['HV2setStep'] = self.packMultiGauge(2,self.HighVoltageCommands['fixed/step'],'1')
                self.HVComms['HV2setStart'] = self.packMultiGauge(2,self.HighVoltageCommands['start/protect'],'0')
                self.HVComms['HV2setProtect'] = self.packMultiGauge(2,self.HighVoltageCommands['start/protect'],'1')
    
                self.SVD.period = max((self.Refresh,len(self.SVD.pollingList)*.1))
    
                self.SVD.start() #self.SVD.updateThread.start()
                self.WarmUp()
        except Exception,e:
            self.error('Exception in VarianDUAL.init_device(): '+str(e))
            self.init_error='Exception in VarianDUAL.init_device(): '+str(e)
            print getLastException()
            PyTango.Except.throw_exception("VarianDUAL_initDeviceException",str(e),str(e))    
            #PyTango.Except.re_throw_exception(e,"VarianDUAL","read_"+comm+"()",result)
            
        self.info("Device Server "+self.get_name()+' waiting for request.')

#------------------------------------------------------------------
#    Always excuted hook method
#------------------------------------------------------------------
    def always_executed_hook(self):
        self.debug("In "+self.get_name()+"::always_executed_hook()")
        """ State Machine Description:
        Communications are prioritary:
            DevState.INIT if variables has not been readed yet
            DevState.UNKNOWN if it's not possible to do it (2 minutes w/out reading)
        Other states are determined by HVxStatus attributes::
            DevState.ALARM=PressureInterlock,PLCInterlock
            DevState.DISABLE=PanelInterlock
            DevState.FAULT=HardwareError,CableInterlock
        """
        try:
            if self.SerialLine and self.SVD: 
                state = self.get_state()
                prev,channelstatus=state,''
                wrongstates={
                    'PanelInterlock': DevState.DISABLE, #Read -3 2Dh33h Power off caused by Interlock Panel
                    'RemoteInterlock': DevState.ALARM, #Read -4 2Dh34h Power off caused by Remote I/O Interlock
                    'CableInterlock': DevState.FAULT, #Read -3 2Dh33h Power off caused by Cable Interlock
                    'HVTemperature': DevState.FAULT, #Read -8 2Dh38h Power off caused by HV Overtemperature
                    'RemoteFault': DevState.FAULT, #Read -7 2Dh37h Power off caused by Remote I/O not Present or Remote I/O Fault
                    'HVProtect': DevState.ALARM, #Read -6 2Dh36h Power off caused by HV Protect
                    'HVShortCircuit': DevState.FAULT #Read -7 2Dh37h Power off caused by HV Short Circuit
                    }
                        
                #Checking Communications status
                if self.SVD.init == False: #If done in 2 lines to avoid changing to ON by default
                    state,channelstatus = DevState.INIT,'Hardware values not read yet, started at %s'%time.ctime(self.startTime)
                elif self.SVD.errors>=len(self.SVD.readList) or self.SVD.lasttime<time.time()-2*60:
                    state,channelstatus=DevState.UNKNOWN,'Unable to communicate with the device since %s'%time.ctime(self.SVD.lasttime)
                    #self.set_state(state)
                
                #Checking interlock state through HV1/HV2 Status
                else: 
                    error_status = self.read_ErrorStatus()
                    self.read_HV1Status()
                    self.read_HV2Status()
                    channelstatus='High Voltage 1 is %s, High Voltage 2 is %s\n'%(self.HV1Status,self.HV2Status)
                    if error_status: 
                        channelstatus+='ERROR: %s\n'%error_status
                        print '*'*80
                        print channelstatus
                        print '*'*80
                    if self.HV1Status in wrongstates.keys(): 
                        state=wrongstates[self.HV1Status]
                    elif self.HV2Status in wrongstates.keys(): 
                        state=wrongstates[self.HV2Status]
                    elif error_status: state=DevState.FAULT
                    elif not any([s.lower()=='on' for s in [self.HV1Status,self.HV2Status] if s]): state=DevState.OFF                           
                    elif self.DefaultStatus and any(s.lower()!=ss.lower() for s,ss in zip((self.HV1Status,self.HV2Status),self.DefaultStatus.split(','))): 
                        state=DevState.ALARM
                        channelstatus+='Channel Status (%s,%s) differ from defaults (%s)'%(self.HV1Status,self.HV2Status,self.DefaultStatus)
                    else: state=DevState.ON
                    
                    #Checking oscillations
                    for old,new in self.voltages:
                        if old and new and abs(old-new)>=100:
                            state=DevState.MOVING
                            self.oscillation = 'Voltage oscillates between %s and %s\n' % (old,new)
                            break
                    for old,new in self.currents:
                        if old and new and not (.5<(old/new)<1.5 ):
                            state=DevState.MOVING
                            self.oscillation = 'Current oscillates between %s and %s\n' % (old,new)
                            break                     
            
                if prev!=state and self.statesQueue.index(state) is None:
                    #Any Wrong State should be kept at least for 20 seconds!
                    self.statesQueue.append(state,1 if state==DevState.ON else 20)
                
                #Getting next state to process (it will be actual state if there's no new states in queue)
                state=self.statesQueue.pop()
                if state == DevState.MOVING: channelstatus+=self.oscillation
                else: self.oscillation = ''
                
                self.comms_report=self.SVD.getReport()
                status = '\n'.join(s for s in [channelstatus,self.Description,self.init_error,self.comms_report,'',self.exception.replace('\n',''),] if s)
                if state is None: self.error('The StateQueue is EMPTY!!!')
                elif prev!=state:
                    self.info('*'*80)
                    self.info('%s.State changed from %s to %s'%(self.get_name(),str(prev),str(state)))
                    self.info(status)
                    self.info('*'*80)
                    self.last_state_change=time.time()
                    self.set_state(state)
                self.set_status(status[:200])
            else:
                self.set_state(DevState.FAULT)
                self.set_status('SerialLine property must be initialized!')
                self.error(self.get_status())
        
        except Exception,e:
            self.error('Exception in always_executed_hook: %s'%str(e))
            print traceback.format_exc()                

#==================================================================
#
#    VarianDUAL read/write attribute methods
#
#==================================================================
#------------------------------------------------------------------
#    Read Attribute Hardware
#------------------------------------------------------------------
    def read_attr_hardware(self,data):
        self.debug("In "+self.get_name()+"::read_attr_hardware()")



#------------------------------------------------------------------
#    Read V1 attribute
#------------------------------------------------------------------
    @self_locked
    def read_V1(self, attr):
        self.debug("In "+self.get_name()+"::read_V1()")
        
        #    Add your own code here        
        attr_V1_read = self.readCommand('HV1 V',long)
        self.voltages[0][0],self.voltages[0][1] = self.voltages[0][1],attr_V1_read
        attr.set_value(attr_V1_read)        


#------------------------------------------------------------------
#    Read V2 attribute
#------------------------------------------------------------------
    @self_locked
    def read_V2(self, attr):
        self.debug("In "+self.get_name()+"::read_V2()")
        
        #    Add your own code here
        
        attr_V2_read = self.readCommand('HV2 V',long)
        self.voltages[1][0],self.voltages[1][1] = self.voltages[1][1],attr_V2_read
        attr.set_value(attr_V2_read)


#------------------------------------------------------------------
#    Read I1 attribute
#------------------------------------------------------------------
    @self_locked
    def read_I1(self, attr):
        self.debug("In "+self.get_name()+"::read_I1()")
        
        #    Add your own code here
        
        attr_I1_read = self.readCommand('HV1 I',float)
        self.currents[0][0],self.currents[0][1] = self.currents[0][1],attr_I1_read
        attr.set_value(attr_I1_read)


#------------------------------------------------------------------
#    Read I2 attribute
#------------------------------------------------------------------
    @self_locked
    def read_I2(self, attr):
        self.debug("In "+self.get_name()+"::read_I2()")
        
        #    Add your own code here
        
        attr_I2_read = self.readCommand('HV2 I',float)
        self.currents[1][0],self.currents[1][1] = self.currents[1][1],attr_I2_read
        attr.set_value(attr_I2_read)


#------------------------------------------------------------------
#    Read P1 attribute
#------------------------------------------------------------------
    @self_locked
    def read_P1(self, attr):
        self.debug("In "+self.get_name()+"::read_P1()")
        
        #    Add your own code here
        
        attr_P1_read = self.readCommand('HV1 P',float)
        attr.set_value(attr_P1_read)


#------------------------------------------------------------------
#    Read P2 attribute
#------------------------------------------------------------------
    @self_locked
    def read_P2(self, attr):
        self.debug("In "+self.get_name()+"::read_P2()")
        
        #    Add your own code here
        
        attr_P2_read = self.readCommand('HV2 P',float)
        attr.set_value(attr_P2_read)
        
#------------------------------------------------------------------
#    Read IonPumpsConfig attribute
#------------------------------------------------------------------
    def read_IonPumpsConfig(self, attr):
        self.info( "In "+ self.get_name()+ "::read_IonPumpsConfig()")
        
        #    Add your own code here
        last_read_time,last_read_value = getattr(self,'_last_IonPumpsConfig',(0,[]))
        #if time.time()>(last_read_time+60):
            #commands = [self.packMultiGauge(i,self.GeneralComms['Device Type'],'?') for i in (1,2)]
            #result = self.SendCommand(commands,'\n').strip().split('\n')
        result = [self.readCommand('Pump%d'%i,str) for i in (1,2)]
        self.info( "\tPumps : %s"%(result))
        attr_IonPumpsConfig_read = [r.strip().replace(' ','').lower() for r in result]
        attr.set_value(attr_IonPumpsConfig_read, len(attr_IonPumpsConfig_read))
        self._last_IonPumpsConfig = (time.time(),attr_IonPumpsConfig_read)
        #else:
            #self.info("... getting IonPumps config from cache ...")
            #attr.set_value_date_quality(last_read_value,last_read_time,PyTango.AttrQuality.ATTR_VALID,len(last_read_value))

#------------------------------------------------------------------
#    Write IonPumpsConfig attribute
#------------------------------------------------------------------
    def write_IonPumpsConfig(self, attr):
        self.info( "In "+ self.get_name()+ "::write_IonPumpsConfig()")
        data=[]
        attr.get_write_value(data)
        #    Add your own code here
        if len(data)!=2:
            PyTango.Except.throw_exception('WrongDataLenght','Data length should be equal to NumOfChannels',
                'write_IonPumpsConfig')
        commands = []
        for chan in (1,2):
            numbers = [i for i,s in enumerate(self.DeviceTypes)
                if data[chan-1].lower().strip().replace(' ','')==s.lower().strip().replace(' ','')
                ]
            if not numbers:
                PyTango.Except.throw_exception('UknownType','%s not in IonPumpTypes list'%data,'write_IonPumpsConfig')
            commands.append(self.packMultiGauge(chan,self.GeneralComms['Device Number'],chr(0x30+numbers[0])))
        self.info("\tSendCommand(%s)"%commands)
        answer = self.SendCommand(commands,separator='\n')
        if any(a.strip()!=self.ACK for a in answer.strip().split('\n')):
            PyTango.Except.throw_exception('NACK','ACK not received: %s'%answer,'write_IonPumpsConfig')

#------------------------------------------------------------------
#    Read IonPumpTypes attribute
#------------------------------------------------------------------
    def read_IonPumpTypes(self, attr):
        self.debug( "In "+ self.get_name()+ "::read_IonPumpTypes()")
        
        #    Add your own code here
        attr_IonPumpTypes_read = self.DeviceTypes[:]
        attr.set_value(attr_IonPumpTypes_read, len(attr_IonPumpTypes_read))

        
#------------------------------------------------------------------
#    Read ProtectSetPoint attribute
#------------------------------------------------------------------
    def read_IProtectSetPoints(self, attr):
        self.debug( "In "+ self.get_name()+ "::read_IProtectSetPoint()")
        
        #    Add your own code here
        #setpoints = lambda d:','.join(s[3:] for s in map(astor.proxies[d].SendCommand,['#171?\n\r','#172?\n\r']))
        protect1 = self.readCommand('HV1 IProtect',long)
        protect2 = self.readCommand('HV2 IProtect',long)
        attr_IProtectSetPoint_read = map(str,[protect1,protect2])
        attr.set_value(attr_IProtectSetPoint_read, len(attr_IProtectSetPoint_read))
        
#---- PressureSetPoints attribute State Machine -----------------

    def is_IProtectSetPoints_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.ON,PyTango.DevState.ALARM,PyTango.DevState.MOVING]:
            #    End of Generated Code
            #    Re-Start of Generated Code
            if req_type == PyTango.AttReqType.WRITE_REQ:
                return False
        return True
        
#------------------------------------------------------------------
#    Write ProtectSetPoint attribute
#------------------------------------------------------------------
    def write_IProtectSetPoints(self, attr):
        self.debug( "In "+ self.get_name()+ "::write_IProtectSetPoint()")
        data=[]
        attr.get_write_value(data)

        #    Add your own code here
        #[proxies['%s22/vc/ipct-%02d'%(d,i)].SendCommand('#1711.0E-07') for d,i,j in (('fe',1,1),('fe',1,2),('bl',1,1),('bl',1,2),('bl',2,1))]
        if len(data)!=2:
            PyTango.Except.throw_exception('WrongDataLenght','Data length should be equal to 2',
                'write_IProtectSetPoint')
        data = map(int,data)
        self.SendCommand([
            self.packMultiGauge(1,self.HighVoltageCommands['Iprotect'],'%05d'%data[0]),
            self.packMultiGauge(2,self.HighVoltageCommands['Iprotect'],'%05d'%data[1])
            ])
                
#------------------------------------------------------------------
#    Read CurrentSetPoints attribute
#------------------------------------------------------------------
    def read_PressureSetPoints(self, attr):
        self.debug( "In "+ self.get_name()+ "::read_PressureSetPoints()")
        
        #    Add your own code here
        #setpoints = lambda d:','.join(s[3:] for s in map(astor.proxies[d].SendCommand,['#171?\n\r','#172?\n\r']))
        protect1 = self.readCommand('HV1 PSetPoint',float)
        protect2 = self.readCommand('HV2 PSetPoint',float)
        attr_PressureSetPoints_read = ['%1.1e'%s for s in (protect1,protect2)]
        attr.set_value(attr_PressureSetPoints_read, len(attr_PressureSetPoints_read))
        
#---- PressureSetPoints attribute State Machine -----------------

    def is_PressureSetPoints_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.ON,PyTango.DevState.ALARM,PyTango.DevState.MOVING]:
            #    End of Generated Code
            #    Re-Start of Generated Code
            if req_type == PyTango.AttReqType.WRITE_REQ:
                return False
        return True


#------------------------------------------------------------------
#    Write CurrentSetPoints attribute
#------------------------------------------------------------------
    def write_PressureSetPoints(self, attr):
        self.debug( "In "+ self.get_name()+ "::write_PressureSetPoints()")
        data=[]
        attr.get_write_value(data)

        #    Add your own code here
        #[proxies['%s22/vc/ipct-%02d'%(d,i)].SendCommand('#1711.0E-07') for d,i,j in (('fe',1,1),('fe',1,2),('bl',1,1),('bl',1,2),('bl',2,1))]
        if len(data)!=2:
            PyTango.Except.throw_exception('WrongDataLenght','Data length should be equal to 2',
                'write_PressureSetPoints')
        data = map(float,data)
        self.SendCommand([
            #self.packMultiGauge(1,self.HighVoltageCommands['Iprotect'],'%05d'%data[0]),
            self.packMultiGauge(1,self.HighVoltageCommands['SetPt1'],'%1.1e'%data[0]),
            self.packMultiGauge(2,self.HighVoltageCommands['SetPt1'],'%1.1e'%data[1])
            ])

#------------------------------------------------------------------
#    Read Interlock attribute
#------------------------------------------------------------------
    def read_Interlock(self, attr):
        self.debug("In "+self.get_name()+"::read_Interlock()")
        
        #    Add your own code here
        
        attr_Interlock_read = bool(int(self.readCommand('Interlock',str)))
        attr.set_value(attr_Interlock_read)


#------------------------------------------------------------------
#    Read FirmwareVersion attribute
#------------------------------------------------------------------
    def read_FirmwareVersion(self, attr):
        self.debug("In "+self.get_name()+"::read_FirmwareVersion()")
        
        #    Add your own code here
        attr_FirmwareVersion_read = self.readCommand('Firmware',str)
        attr.set_value(attr_FirmwareVersion_read)
        
#------------------------------------------------------------------
#    Read SerialLine attribute
#------------------------------------------------------------------
    def read_SerialLine(self, attr):
        self.debug("In "+self.get_name()+"::read_SerialLine()")
        
        #    Add your own code here
        attr_SerialLine_read = self.SerialLine[:]
        attr.set_value(attr_SerialLine_read)        

#------------------------------------------------------------------
#    Read ModeLocal attribute
#------------------------------------------------------------------
    def read_ModeLocal(self, attr):
        self.debug("In "+self.get_name()+"::read_Mode()")
        
        #    Add your own code here
        attr_Mode_read = self.getModeLocal()
        attr.set_value(attr_Mode_read)
        try:
          if (time.time()>self.last_serial_change+60 
                  and str(self.ForceLocal.lower())  in ('true','yes') 
                  and 'LOCAL' not in attr_Mode_read):
              self.info('>'*80)
              self.info('ForceLocal=%s; Forcing LOCAL mode.'%self.ForceLocal)
              self.SetMode('local')
        except: self.warning(traceback.format_exc())


#------------------------------------------------------------------
#    Read ModeStep attribute
#------------------------------------------------------------------
    def read_ModeStep(self, attr):
        self.debug("In "+self.get_name()+"::read_Mode()")
        
        #    Add your own code here
        attr_Mode_read = 'Unknown'
        st1 = self.readCommand('HV1Step',bool)
        st2 = self.readCommand('HV2Step',bool)
        attr_Mode_read=['FIXED','STEP'][st1]
        if st2!=st1:
            attr_Mode_read+=';'+['FIXED','STEP'][st2]
        attr.set_value(attr_Mode_read)
        try:
          if (time.time()>self.last_serial_change+60 
                  and str(self.ForceStep.lower()) in ('true','yes')
                  and 'FIXED' in attr_Mode_read):
              self.info('>'*80)
              self.info('ForceStep=%s; Forcing STEP mode.'%self.ForceStep)
              self.SetMode('step')
        except: self.warning(traceback.format_exc())              


#------------------------------------------------------------------
#    Read ModeProtect attribute
#------------------------------------------------------------------
    def read_ModeProtect(self, attr):
        self.debug("In "+self.get_name()+"::read_Mode()")
        
        #    Add your own code here
        attr_Mode_read = 'Unknown'
        st1 = self.readCommand('HV1Protect',bool)
        st2 = self.readCommand('HV2Protect',bool)
        attr_Mode_read=['START','PROTECT'][st1]
        if st2!=st1:
            attr_Mode_read+=';'+['START','PROTECT'][st2]
        attr.set_value(attr_Mode_read)
        try:
          if (time.time()>self.last_serial_change+60 
                  and str(self.ForceProtect.lower()) in ('true','yes')
                  and 'START' in attr_Mode_read 
                  and self.HV1Status.lower()=='on'):
              self.info('>'*80)
              self.info('ForceProtect=%s: Forcing PROTECT mode.'%self.ForceProtect)
              self.SetMode('protect')
        except: self.warning(traceback.format_exc())


#------------------------------------------------------------------
#    Read HV1Status attribute
#------------------------------------------------------------------
    def read_HV1Status(self, attr=None):
        self.debug("In "+self.get_name()+"::read_HV1Status()")
        
        #    Add your own code here
        attr_HV1Status_read = "Unknown"
        st = self.readCommand('HV1Status',int)
        for k,v in self.OnOffCoding.iteritems():
            if v==st: attr_HV1Status_read = k
        self.HV1Status=attr_HV1Status_read
        if attr:
            attr.set_value(attr_HV1Status_read)
            
#------------------------------------------------------------------
#    Read HV2Status attribute
#------------------------------------------------------------------
    def read_HV2Status(self, attr=None):
        self.debug("In "+self.get_name()+"::read_HV2Status()")
        
        #    Add your own code here
        attr_HV2Status_read = "Unknown"
        st = self.readCommand('HV2Status',int)
        for k,v in self.OnOffCoding.iteritems():
            if v==st: attr_HV2Status_read = k
        self.HV2Status=attr_HV2Status_read
        if attr:
            attr.set_value(attr_HV2Status_read)

#------------------------------------------------------------------
#    Read ErrorStatus attribute
#------------------------------------------------------------------
    def read_ErrorStatus(self,attr=None):
        aname = attr and attr.get_name() or 'ErrorStatus'
        now=time.time()
        if aname=='ErrorStatus':
            st = self.readCommand('ErrorStatus',int)
        elif aname=='HV1Code':
            st = self.readCommand('HV1Status',int)
            if st and st!=self.prevHV1Code[1] and now<=self.prevHV1Code[0]+30:
                st=self.prevHV1Code[1]
        elif aname=='HV2Code':
            st = self.readCommand('HV2Status',int)
            if st and st!=self.prevHV2Code[1] and now<=self.prevHV2Code[0]+30:
                st=self.prevHV2Code[1]
        else: raise Exception('Error reading %s'%attr.get_name())
        if attr: attr.set_value(st)
        else: return self.DualControllerErrorStatus['HV'].get(str(st),'')
        
    read_HV1Code=read_ErrorStatus
    read_HV2Code=read_ErrorStatus
    
#------------------------------------------------------------------
#    Read Missreadings attribute
#------------------------------------------------------------------
    def read_Missreadings(self,attr=None,value=None):
        if not hasattr(self,'missreadings'): setattr(self,'missreadings',[])
        if value is not None and value not in self.missreadings: self.missreadings.append(str(value))
        if len(self.missreadings)>256: self.missreadings=self.missreadings[-256:-1]+[self.missreadings[-1]]
        if attr is not None: attr.set_value(self.missreadings)
        return self.missreadings

#------------------------------------------------------------------
#    Read Protocol attribute
#------------------------------------------------------------------
    def read_Protocol(self, attr):
        self.debug("In "+self.get_name()+"::read_Protocol()")
        
        #    Add your own code here
        attr_Protocol_read = \
        " Use #005?\\r for testing DUAL firmware version ...\n\n"\
        "The Structure of the MultiGauge Compatible Protocol is:\n"\
        "\tField          No. of Bytes          Value          Description \n"\
        "\tHeader command         1         23h         Header .#.\n"\
        "\tHeader response     1         3Eh         Header .>.\n"\
        "\tChannel         1         30h        No Channel\n"\
        "\t                    31h        High Voltage 1\n"\
        "\t                    32h        High Voltage 2\n"\
        "\t                    33h        Gauge 1\n"\
        "\t                    34h        Gauge 2\n"\
        "\t                    35h        Serial Communication\n"\
        "\tCommand         2                 See commands description\n"\
        "\tData             n                 See commands description\n"\
        "\tTerminator         1         0Dh         Carriage Return\n"
        attr.set_value(attr_Protocol_read)

    def read_BlackBox(self,attr=None):
        self.__blackbox = self.SVD.blackbox.to_string().split('\n')
        attr.set_value(self.__blackbox, len(self.__blackbox))
        
    def SaveBlackBox(self,filename):
        print 'In SaveBlackBox(%s)'%filename
        if filename!='': self.SVD.blackbox.save(filename)
        return self.SVD.blackbox.to_string()
        
#==================================================================
#
#    VarianDUAL command methods
#
#==================================================================

#------------------------------------------------------------------
#    SendCommand command:
#
#    Description: This command allows to send an array of characters directly to the device.
#                
#    argin:  DevString
#    argout: DevString
#------------------------------------------------------------------
    def SendCommand(self, argin, separator = '', mode=True):
        self.info("In "+self.get_name()+"::SendCommand(%s)"%argin)
        #    Add your own code here
        if mode:
            local = time.time()>=self.last_serial_change+60 and self.ForceLocal
            if not 'SERIAL' in self.getModeLocal(): self.SetMode('serial')
        else: local = False
        self.SVD.stop()
        if self.SVD.Alive:
            self.warning('SVD IS STILL ALIVE!!!')
        try:            
            result,argin = '',toSequence(argin)
            for arg in argin:
                result+=self.SVD.serialComm(arg)+separator
        except Exception, e:
            self.error(str(e))
            self.set_status(self.get_status())
            PyTango.Except.throw_exception('SendCommandError',str(e),'SendCommand')
        self.SVD.start()
        if '!' in result:
            code = result[result.index('!')+1]
            self.error('ProtocolError%s: %s'%(code,self.ProtocolErrors[code]))
            #raise Exception('ProtocolError%s: %s'%(code,self.ProtocolErrors[code]))
            PyTango.Except.throw_exception('ProtocolError',self.ProtocolErrors[code],'SendCommand')
        if mode and local: self.SetMode('local')
        return result
        
#------------------------------------------------------------------
#    SetMode command:
#
#    Description: Changes between Local and Serial modes
#                
#    argin:  DevString
#------------------------------------------------------------------
    def SetMode(self, argin):
        self.info("In "+self.get_name()+"::SetMode(%s)"%argin)
        #    Add your own code here
        SERIAL_WAIT = 60
        if argin.lower()=='serial':
            if time.time()>=self.last_serial_change+SERIAL_WAIT:
                self.info('\tsetting SERIAL mode ...')
                self.writeCommand('Serial On',self.HVComms['Serial On'])
                self.info('\tserial command sent, waiting for repply ...')
                self.last_serial_change=time.time()
                self.SVD.setPolledNext(self.HVComms['ModeLocal'])
                while time.time()<self.last_serial_change+1.:
                    self.event.wait(.05)
                    if 'SERIAL' in self.getModeLocal(): break
                if 'SERIAL' not in self.getModeLocal():
                    print self.warning('SetMode(SERIAL) failed?')
                    #raise Exception('SetMode(SERIAL) failed!')
                self.info('Mode changed to serial in %s seconds.'%(time.time()-self.last_serial_change))
            else:
                self.info('\tSERIAL mode already set %d seconds ago.'%(time.time()-self.last_serial_change))
        elif argin.lower()=='local':
            self.writeCommand('Serial Off',self.HVComms['Serial Off'])
            self.last_serial_change=0
        elif argin.lower() in ['fixed','step','start','protect']:
            local = time.time()>=self.last_serial_change+SERIAL_WAIT and self.ForceLocal
            self.SetMode('serial')
            if argin.lower()=='fixed':
                self.writeCommand('HV1setFixed',self.HVComms['HV1setFixed'])
                self.writeCommand('HV2setFixed',self.HVComms['HV2setFixed'])
                self.ForceStep = False
            elif argin.lower()=='step':
                self.writeCommand('HV1setStep',self.HVComms['HV1setStep'])
                self.writeCommand('HV2setStep',self.HVComms['HV2setStep'])
            elif argin.lower()=='start':
                self.writeCommand('HV1setStart',self.HVComms['HV1setStart'])
                self.writeCommand('HV2setStart',self.HVComms['HV2setStart'])
            elif argin.lower()=='protect':
                self.writeCommand('HV1setProtect',self.HVComms['HV1setProtect'])
                self.writeCommand('HV2setProtect',self.HVComms['HV2setProtect'])
                self.ForceProtect = False
            if local: self.SetMode('local')
        return 'DONE'

#------------------------------------------------------------------
#    On command:
#
#    Description: It enables both High Voltage Outputs of the device.
#                
#------------------------------------------------------------------
    def On(self):
        self.info("In "+self.get_name()+"::On()")
        #    Add your own code here
        status = str(self.DefaultStatus or 'On,On').lower().strip()
        local = time.time()>=self.last_serial_change+60 and self.ForceLocal
        self.SetMode('serial')
        if status.startswith('on'): 
            self.writeCommand('HV1 On',self.HVComms['HV1 On'])
            self.SVD.setPolledNext(self.HVComms['HV1Status'])
        if status.endswith('on'):
            self.writeCommand('HV2 On',self.HVComms['HV2 On'])
            self.SVD.setPolledNext(self.HVComms['HV2Status'])
        if local: self.SetMode('local')
        return 'DONE'

#------------------------------------------------------------------
#    Off command:
#
#    Description: It disables both high voltage outputs of the device
#                
#------------------------------------------------------------------
    def Off(self):
        self.info("In "+self.get_name()+"::Off()")
        #    Add your own code here
        local = time.time()>=self.last_serial_change+60 and self.ForceLocal
        self.SetMode('serial')
        self.writeCommand('HV1 Off',self.HVComms['HV1 Off'])
        self.SVD.setPolledNext(self.HVComms['HV1Status'])
        self.writeCommand('HV2 Off',self.HVComms['HV2 Off'])
        self.SVD.setPolledNext(self.HVComms['HV2Status'])
        if local: self.SetMode('local')
        return 'DONE'

#------------------------------------------------------------------
#    OnHV1 command:
#
#    Description: It enables High Voltage Channel 1
#                
#------------------------------------------------------------------
    def OnHV1(self):
        self.info("In "+self.get_name()+"::OnHV1()")
        #    Add your own code here
        self.writeCommand('HV1 On',self.HVComms['HV1 On'],mode=True)
        self.SVD.setPolledNext(self.HVComms['HV1Status'])
        return 'DONE'

#------------------------------------------------------------------
#    OnHV2 command:
#
#    Description: It enables High Voltage Channel 2
#                
#------------------------------------------------------------------
    def OnHV2(self):
        self.info("In "+self.get_name()+"::OnHV2()")
        #    Add your own code here
        self.writeCommand('HV2 On',self.HVComms['HV2 On'],mode=True)
        self.SVD.setPolledNext(self.HVComms['HV2Status'])
        return 'DONE'

#------------------------------------------------------------------
#    OffHV1 command:
#
#    Description: It disables High Voltage Channel 1
#                
#------------------------------------------------------------------
    def OffHV1(self):
        self.info("In "+self.get_name()+"::OffHV1()")
        #    Add your own code here
        self.writeCommand('HV1 Off',self.HVComms['HV1 Off'],mode=True)
        self.SVD.setPolledNext(self.HVComms['HV1Status'])
        return 'DONE'

#------------------------------------------------------------------
#    OffHV2 command:
#
#    Description: It disables High Voltage Channel 2
#                
#------------------------------------------------------------------
    def OffHV2(self):
        self.info("In "+self.get_name()+"::OffHV2()")
        #    Add your own code here
        self.writeCommand('HV2 Off',self.HVComms['HV2 Off'],mode=True)
        self.SVD.setPolledNext(self.HVComms['HV2Status'])
        return 'DONE'

#==================================================================
#
#    VarianDUALClass class definition
#
#==================================================================
class VarianDUALClass(PyTango.PyDeviceClass):

    #    Class Properties
    class_property_list = {
        }


    #    Device Properties
    device_property_list = {
        'SerialLine':
            [PyTango.DevString,
            "SerialLine Device Server to connect with",
            [''] ],
        'Refresh':
            [PyTango.DevDouble,
            "Period (in seconds) for the internal refresh thread (1 entire cycle).",
            [ 3.0 ] ],
        'StartSequence':
            [PyTango.DevVarStringArray,
            "Commands available are: START/PROTECT, FIXED/STEP, ON1,ON2, . Conditions like ON1:bl/vc/pir/p < 1e-4 can be used to have control over warmup.",
            ['#LOCAL/SERIAL, START/PROTECT, FIXED/STEP, ON1:bl/vc/ccg/p1<1e-5, ON2:bl/vc/ccg/p2<1e-5'] ],
        'ForceStep':
            [PyTango.DevString,
            "YES to force Step Mode always enabled",
            ['YES'] ],
        'ForceProtect':
            [PyTango.DevString,
            "YES to force Protect Mode always enabled",
            ['YES'] ],
        'ForceLocal':
            [PyTango.DevString,
            "YES to force Local Mode always enabled",
            ['YES'] ],            
        'DefaultStatus':
            [PyTango.DevString,
            "On/Off,On/Off; the expected status for each channel, empty if not used",
            [''] ],
        'Description':
            [PyTango.DevString,
            "This string field will appear in the status and can be used to add extra information about equipment location",
            [''] ],
        'LogLevel':
            [PyTango.DevString,
            "This property selects the log level (DEBUG/INFO/WARNING/ERROR)",
            ['INFO'] ],
        'BlackBox':
            [PyTango.DevLong,
            "Lenght of the serial line buffer to be kept for debugging",
            [0] ],
        }


    #    Command definitions
    cmd_list = {
        'SendCommand':
            [[PyTango.DevString, "Command to send (literally)"],
            [PyTango.DevString, "Answer received to command sended (literally)"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
            } ],
        'SetMode':
            [[PyTango.DevString, "Switches between configuration modes: LOCAL/SERIAL , STEP/FIXED , START/PROTECT"],
            [PyTango.DevString, "Switches between configuration modes: LOCAL/SERIAL , STEP/FIXED , START/PROTECT"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'On':
            [[PyTango.DevVoid, "Switches On both High voltage channels (managed by DefaultStatus)"],
            [PyTango.DevString, "Switches On both High voltage channels (managed by DefaultStatus)"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'Off':
            [[PyTango.DevVoid, "Switchs Off both High voltage channels (managed by DefaultStatus)"],
            [PyTango.DevString, "Switchs Off both High voltage channels (managed by DefaultStatus)"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'OnHV1':
            [[PyTango.DevVoid, "Switchs On the 1st High voltage channel"],
            [PyTango.DevString, "Switchs On the 1st High voltage channel"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'OnHV2':
            [[PyTango.DevVoid, "Switchs On the 2nd High voltage channel"],
            [PyTango.DevString, "Switchs On the 2nd High voltage channel"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'OffHV1':
            [[PyTango.DevVoid, "Switchs Off the 1st High voltage channel"],
            [PyTango.DevString, "Switchs Off the 1st High voltage channel"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'OffHV2':
            [[PyTango.DevVoid, "Switchs Off the 2nd High voltage channel"],
            [PyTango.DevString, "Switchs Off the 2nd High voltage channel"],
            {
                'Display level':PyTango.DispLevel.EXPERT,
             } ],
        'WarmUp':
            [[PyTango.DevVoid, "Executes StartSequence"],
            [PyTango.DevString, "Executes StartSequence"],
            {'Display level':PyTango.DispLevel.EXPERT,} ],
        'AsciiChecksum':
            [[PyTango.DevString, "Returns the valid checksum for an Ascii Command"],
            [PyTango.DevString, "Returns the valid checksum for an Ascii Command"],
            {'Display level':PyTango.DispLevel.EXPERT,} ],
        'SetStartSequence':
            [[PyTango.DevVarStringArray, "Set StartSequence, empty to see current, DELETE to erase it"],
            [PyTango.DevVarStringArray, "Set StartSequence, empty to see current, DELETE to erase it"],
            {'Display level':PyTango.DispLevel.EXPERT,} ],
        'SaveBlackBox':
            [[PyTango.DevString,"filename to export blackbox"],
            [PyTango.DevString,"filename to export blackbox"]],
        }


    #    Attribute definitions
    attr_list = {
        'V1':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"V",
                'format':"%5d",
                'description':"Read: Voltage Value for Channel 1\nWrite: Voltage Step for Channel 1",
            } ],
        'V2':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"V",
                'format':"%5d",
                'description':"Read: Voltage Value for Channel 2\nWrite: Voltage Step for Channel 2",
            } ],
        'I1':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mA",
                'format':"%5.2e",
                'description':"Read: Current Value for Channel 1\nWrite: Current Step for Channel 1",
            } ],
        'I2':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mA",
                'format':"%5.2e",
                'description':"Read: Current Value for Channel 2\nWrite: Current Step for Channel 2",
            } ],
        'P1':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mbar",
                'format':"%5.2e",
                'description':"Read: Pressure for Channel 1\nWrite: Max Pressure for Channel 1",
            } ],
        'P2':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mbar",
                'format':"%5.2e",
                'description':"Read: Pressure for Channel 2\nWrite: Max Pressure for Channel 2",
            } ],
        'Interlock':
            [[PyTango.DevBoolean,
            PyTango.SCALAR,
            PyTango.READ]],
        'IonPumpsConfig':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ_WRITE, 2]],       
        'IonPumpTypes':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ, 20]],
        'IProtectSetPoints':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ_WRITE, 2],
            {
                'unit':"mA",
                'format':"%4d",
                'Display level':PyTango.DispLevel.EXPERT,
             }],
        'PressureSetPoints':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ_WRITE, 2],
            {
                'unit':"mbar",
                'format':"%4d",
                'Display level':PyTango.DispLevel.EXPERT,
             }],
        'FirmwareVersion':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'SerialLine':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],             
        'ModeLocal':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'Polling period':2000,
            } ],
        'ModeStep':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'ModeProtect':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'HV1Status':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'HV2Status':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        #'Protocol': #Nobody wants to know that
            #[[PyTango.DevString,
            #PyTango.SCALAR,
            #PyTango.READ],
            #{
                #'Display level':PyTango.DispLevel.EXPERT,
            #} ],
        'ErrorStatus':
            [[PyTango.DevShort,PyTango.SCALAR,PyTango.READ],{'Display level':PyTango.DispLevel.EXPERT,} ],
        'HV1Code':
            [[PyTango.DevShort,PyTango.SCALAR,PyTango.READ],{'Display level':PyTango.DispLevel.EXPERT,} ],
        'HV2Code':
            [[PyTango.DevShort,PyTango.SCALAR,PyTango.READ],{'Display level':PyTango.DispLevel.EXPERT,} ],
        'Missreadings':
            [[PyTango.DevString,PyTango.SPECTRUM,PyTango.READ, 256],{'Display Level':PyTango.DispLevel.EXPERT,} ],
        'BlackBox':
            [[PyTango.DevString,PyTango.SPECTRUM,PyTango.READ, 1024],{'Display Level':PyTango.DispLevel.EXPERT,} ],
        }

#------------------------------------------------------------------
#    VarianDUALClass Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.PyDeviceClass.__init__(self, name)
        self.set_type(name);
        print "In VarianDUALClass  constructor"

for k in VarianDUALClass.attr_list.keys(): 
    comm = 'is_%s_allowed'%k
    if not hasattr(VarianDUAL,comm): 
        setattr(VarianDUAL,comm,VarianDUAL.is_Attr_allowed)
        

#==================================================================
#
#    VarianDUAL class main method
#
#==================================================================
if __name__ == '__main__':
    try:
        py = PyTango.PyUtil(sys.argv)
        py.add_TgClass(VarianDUALClass,VarianDUAL,'VarianDUAL')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed,e:
        print '-------> Received a DevFailed exception:',e
    except Exception,e:
        print '-------> An unforeseen exception occured....',e
