import os
import json
from logger import set_log

from utils import *
from tcp_driver import *
from control_config import *

"""
COMMAND FORMAT
byte[0]: motor address
byte[1]: function code
byte[2-]: comdand data
byte[-1]: CheckSum Use fixd checksum=0x6B
"""


class MOTOR(object):
    def __init__(self, address, BusHandle, config):
        self.address = address
        self.handle = BusHandle
        self._log = set_log('motor_log', 'motor.log')
        
        if isinstance(config, str): # is path
            if os.path.isfile(config):
                with open(config, 'r') as cfg:
                  self.config = json.load(cfg)
        elif isinstance(config, dict): # is config
            self.config = config
        self.pulseDiff = 0
        assert self.set_direction() == 0
        assert self.set_protect() == 0
        assert self.set_response() == 0

    def set_home(self, overtime=10):
        if self.set_home_parameter() != message.OK:
            #todo
            print("can't set home parameter")
            return
        if self.act_limit_home(overtime) != message.OK:
            #todo
            print("can't activate limit home")
            return
        c = 0
        last_ = self.motorStatus
        while 1:
            time.sleep(1)
            cur = self.motorStatus
            if last_[4] == True and last_[0] == False:
                if cur[0] == True and cur[4] == False:
                    self._homed = True
                    self.pulseDiff = self.realPulse
                    break
            c += 1
            last_ = cur
            if c  >= overtime:
                raise TimeoutError

    def enable_motor(self, disable: bool) -> int:
        '''
        disable: bool 0: enable, 1:disable
        '''
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = MOTOR_ENABLE[0][0]
        cmd[3] = 0X00 if disable else 0x01
        cmd[4] = checkSum(cmd[:4])
        res, rets = self.handle.cmd(cmd[:5], MOTOR_ENABLE)
        ret = rets[0]
        assert MOTOR_ENABLE[1]+1 == len(ret)
        if res == message.OK:
            if ret[1] == cmd[1] and ret[3] == MOTOR_ENABLE[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[3] == MOTOR_ENABLE[3]:
                return message.CommandRej_MotorEn
        else:
            print(message.sentence[res])


    def set_direction(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = SET_CW[0][0]
        cmd[3] = self.config['MOTION_CONFIG']['CW_REVERSE']
        cmd[4] = checkSum(cmd[:4])
        res, rets = self.handle.cmd(cmd[:5], SET_CW)
        if res == message.OK:
            ret = rets[0]
            assert SET_CW[1]+1 == len(ret)
            if ret[1] == cmd[1] and ret[3] == SET_CW[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[3] == SET_CW[3]:
                return message.CommandRej_MotorEn
        else:
            print(message.sentence[res])

    
    def set_protect(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = MODIFY_BLOCK_PROTECT[0][0]
        cmd[3] = self.config['MOTION_CONFIG']['EN_BLOCK_PROTECT']
        cmd[4] = checkSum(cmd[:4])
        res, rets = self.handle.cmd(cmd[:5], MODIFY_BLOCK_PROTECT)
        if res == message.OK:
            ret = rets[0]
            assert MODIFY_BLOCK_PROTECT[1]+1 == len(ret)
            if ret[1] == cmd[1] and ret[3] == MODIFY_BLOCK_PROTECT[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[3] == MODIFY_BLOCK_PROTECT[3]:
                return message.CommandRej_setProtect
        else:
            print(message.sentence[res])

    
    def reset_block_protect(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = RELEASE_BLOCK_PROTECT[0][0]
        cmd[3] = self.config['MOTION_CONFIG']['EN_BLOCK_PROTECT']
        cmd[4] = checkSum(cmd[:4])
        res, rets = self.handle.cmd(cmd[:5], RELEASE_BLOCK_PROTECT)
        if res == message.OK:
            ret = rets[0]
            assert RELEASE_BLOCK_PROTECT[1]+1 == len(ret)
            if ret[1] == cmd[1] and ret[3] == RELEASE_BLOCK_PROTECT[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[3] == RELEASE_BLOCK_PROTECT[3]:
                return message.CommandRej_relProtect
        else:
            print(message.sentence[res])


    def set_response(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = MODIFY_RESPONSE_MODE[0][0]
        cmd[3] = self.config['SYSTEM_CONFIG']['AUTO_RESPONSE']
        cmd[4] = checkSum(cmd[:4])
        res, rets = self.handle.cmd(cmd[:5], MODIFY_RESPONSE_MODE)
        if res == message.OK:
            ret = rets[0]
            assert MODIFY_RESPONSE_MODE[1]+1 == len(ret)
            if ret[1] == cmd[1] and ret[3] == MODIFY_RESPONSE_MODE[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[3] == MODIFY_RESPONSE_MODE[3]:
                return message.CommandRej_relProtect
        else:
            print(message.sentence[res])


    def speed_control(self, s:int, at: int, direction: bool) -> int:
        '''
        s: int speed in rpm
        at: int accelarate time in ms
        direction: bool 0: CW, 1:CCW
        sync: bool 0: unsync, 1: sync
        '''
        assert s < self.config['MOTOION_CONFIG']['MAX_SPEED']
        assert at < self.config['MOTOION_CONFIG']['MAX_ACCERATE_TIME']
        speed_base_code =  0x80 if direction else 0x00
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = SPEED_CONTROL[0][0]
        cmd[3] = ((s >> 8) & 0x0F) + speed_base_code
        cmd[4] = s & 0xFF
        cmd[5] = int(256 - (at / 50e-3)) if at != 0x00 else at
        cmd[6] = checkSum(cmd[:6])
        res, rets = self.handle.cmd(cmd[:7], SPEED_CONTROL)
        for ret in rets:
            assert SPEED_CONTROL[1]+1 == len(ret)
            if res == message.OK:
                if ret[1] == self.address and ret[2] == cmd[2] and ret[3] == SPEED_CONTROL[2]:
                    return message.OK
                elif ret[1] == cmd[1] and ret[3] == SPEED_CONTROL[3]:
                    return message.CommandRej_SpCtl
            else:
                print(message.sentence[res])
                

    def rel_position_control(self, s:int, at: int, p: int, 
                         direction: bool, overtime:int=2) -> int:
        '''
        s: int speed in rpm
        at: int accelarate time in ms
        p: int move pulse
        direction: bool 0: CW, 1:CCW
        overtime: int time tolerance over theory time
        '''
        assert s <= self.config['MOTION_CONFIG']["MAX_SPEED"]
        assert at <= self.config['MOTION_CONFIG']["MAX_ACCERATE_TIME"]

        speed_base_code =  0x80 if direction else 0x00
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = REL_POSITION_CONTROL[0][0]
        cmd[3] = ((s >> 8) & 0x0F) + speed_base_code
        cmd[4] = s & 0xFF
        cmd[5] = int(256 - (at / 50e-3)) if at != 0x00 else at
        cmd[6] = (p >> 24) & 0xff
        cmd[7] = (p >> 16) & 0xff
        cmd[8] = (p >> 8) & 0xff
        cmd[9] = (p) & 0xff
        cmd[10] = checkSum(cmd[:10])
        res, rets = self.handle.cmd(cmd[:11], REL_POSITION_CONTROL, require_response=False)
        if res == message.OK:
            return res
        else:
            print(message.sentence[res])

    def abs_position_control(self, s:int, at: int, p: int) -> int:
        '''
        s: int speed in rpm
        at: int accelarate time in ms
        p: int move pulse
        overtime: int time tolerance over theory time
        '''
        assert s <= self.config['MOTION_CONFIG']["MAX_SPEED"]
        assert at <= self.config['MOTION_CONFIG']["MAX_ACCERATE_TIME"]

        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = ABS_CORD_POSITION_CONTROL[0][0]
        cmd[3] = ((s >> 8) & 0x0F)
        cmd[4] = s & 0xFF
        cmd[5] = int(256 - (at / 50e-3)) if at != 0x00 else at
        cmd[6] = (p >> 24) & 0xff
        cmd[7] = (p >> 16) & 0xff
        cmd[8] = (p >> 8) & 0xff
        cmd[9] = (p) & 0xff
        cmd[10] = checkSum(cmd[:10])
        res, rets = self.handle.cmd(cmd[:11], ABS_CORD_POSITION_CONTROL, require_response=False)
        if res == message.OK:
            return res
        else:
            print(message.sentence[res])


    def emergency_stop(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = EMERGENCY_STOP[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, ret = self.handle.cmd(cmd[:4], EMERGENCY_STOP)
        assert EMERGENCY_STOP[1]+1 == len(ret[0])
        if res == message.OK:
            if ret[1] == cmd[1] and ret[3] == EMERGENCY_STOP[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[2] == EMERGENCY_STOP[3]:
                return message.CommandRej_MotorEn
        else:
            print(message.sentence[res])


    def set_home_parameter(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = SET_HOME_PARA[0][0]
        cmd[3] = self.config['HOME_CONFIG']['home_trigger']
        cmd[4] = self.config['HOME_CONFIG']['home_direction']
        cmd[5] = (self.config['HOME_CONFIG']['home_speed'] >> 8) & 0xFF
        cmd[6] = self.config['HOME_CONFIG']['home_speed'] & 0xFF
        cmd[7] = self.config['HOME_CONFIG']['end_limit']
        cmd[8] = checkSum(cmd[:8])
        res, rets = self.handle.cmd(cmd[:9], SET_HOME_PARA)
        assert SET_HOME_PARA[1]+1 == len(rets[0])
        if res == message.OK:
            ret = rets[0]
            if ret[1] == cmd[1] and ret[3] == SET_HOME_PARA[2]:
                return message.OK
            elif ret[1] == cmd[1] and ret[3] == SET_HOME_PARA[3]:
                return message.CommandRej_MotorEn
        else:
            print(message.sentence[res])


    def act_limit_home(self, overtime=10):
        '''
        block command
        '''
        self._homefailed = False
        self._inhome = False
        self._homed = False
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = ACT_LIMIT_HOME[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4], ACT_LIMIT_HOME)

        if res == message.OK:
            for ret in rets:
                assert ACT_LIMIT_HOME[1]+1 == len(ret)
                if ret[1] == self.address and ret[2] == cmd[2]:
                    self._homefailed = ret[3] == 0x00
                    self._inhome = ret[3] == 0x01
                    self._homed = ret[3] == 0x02
            return message.OK  
        else:
            message.alm(self._log, res)


    def _posCtl_status(self, posCtl_ret):
        status = posCtl_ret[3]
        self._posCtl_Failed = False
        self._posCtl_Start = False
        self._posCtl_Finish = False
        self._posCtl_OverLimit = False
        if status == 0:
            self._posCtl_Failed = True
        if status == 1:
            self._posCtl_Start = True
        if status == 2:
            self._posCtl_Finish = True
        if status == 3:
            self._posCtl_OverLimit = True
        return [self._posCtl_Failed,
                self._posCtl_Start,
                self._posCtl_Finish,
                self._posCtl_OverLimit]
        

    @property
    def realSpeed(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_REAL_SPEED[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4], GET_REAL_SPEED)
        if res == message.OK:
            for ret in rets:
                if ret[1] == self.address and ret[2] == cmd[2]:
                    value = -1 * recovery_WORD(ret[3:5])
                else:
                    message.alm(self._log, message.CommandErr_getAbsPos)
            if self.config['MOTION_CONFIG']['CW_REVERSE'] == 1:
                return -1 * value 
            else:
                return value
        else:
            message.alm(self._log, res)


    @property
    def absEncode(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_ABS_ENCODER[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4],GET_ABS_ENCODER)
        if res == message.OK:
            for ret in rets:
                inp = recovery_DWORD(ret[3:7])
                sp = recovery_WORD(ret[7:9])
            return (inp,sp)
        else:
            message.alm(self._log, res)


    @property
    def realPulse(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_REL_PULSE[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4],GET_REL_PULSE)
        if res == message.OK:
            for ret in rets:
                value = int(recovery_TWORD(ret[3:9])/5.11)
            if self.config['MOTION_CONFIG']['CW_REVERSE'] == 1:
                return -1 * value 
            else:
                return value
        else:
            message.alm(self._log, res)

    @property
    def realAngleErr(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_ANGLE_DEVATION[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4],GET_ANGLE_DEVATION)
        if res == message.OK:
            for ret in rets:
                value = recovery_DWORD(ret[3:7])
            return int(value / 142.22)
        else:
            message.alm(self._log, res)
        
    @property
    def homeStatus(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_AUTOHOME_STATUS[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4], GET_AUTOHOME_STATUS)
        if res == message.OK:
            for ret in rets:
                if ret[1] == self.address and ret[2] == cmd[2]:
                    self._inhoming = ret[3] == 0x01
                    self._homed = ret[3] == 0x02
                    self._homefailed = ret[3] == 0x03
                else:
                    message.alm(self._log, message.RetErr_WrongAddr)
            return [self._inhoming, self._homed, self._homefailed]
        else:
            return [False, False, False]

    @property
    def motorStatus(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_MOTOR_STATUS[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4], GET_MOTOR_STATUS)
        if res == message.OK:
            for ret in rets:
                if ret[1] == self.address and ret[2] == cmd[2]:
                    self._stop = ret[3] == 0x01
                    self._inAccerlate = ret[3] == 0x02
                    self._inDeccerlate = ret[3] == 0x03
                    self._inSpeed = ret[3] == 0x04
                    self._Homing = ret[3] == 0x05
                    self._Caling = ret[3] == 0x06
                else:
                    message.alm(self._log, message.RetErr_WrongAddr)
            return [ret[3]==i for i in range(1,7)]
        else:
            message.alm(self._log, res)

    @property
    def motorProtect(self):
        val = None
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_BLOCKED_STATUS[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4], GET_BLOCKED_STATUS)
        if res == message.OK:
            for ret in rets:
                if ret[1] == self.address and ret[2] == cmd[2]:
                    val = ret[3] == 0x01
                else:
                    message.alm(self._log, message.RetErr_WrongAddr)
            return val
        else:
            message.alm(self._log, res)


    @property
    def version(self):
        cmd = bytearray(18)
        cmd[0] = CMD_HEAD
        cmd[1] = self.address
        cmd[2] = GET_VERSION[0][0]
        cmd[3] = checkSum(cmd[:3])
        res, rets = self.handle.cmd(cmd[:4],GET_VERSION)
        if res == message.OK:
            ret = rets[0]
            print(ret[3], ret[4:7])
        else:
            message.alm(self._log, res)



def main():

    BusHandle = Handle('10.90.252.53', 1030, 'win', 0.1)
    motor_1 = MOTOR(3, BusHandle, './XAXIS_CONFIG.json')
    motor_2 = MOTOR(2, BusHandle, './YAXIS_CONFIG.json')
    motor_1.reset_block_protect()
    motor_2.reset_block_protect()
    
    print(motor_1.motorStatus)
    print(motor_2.motorStatus)
    motor_1.abs_position_control(100,2,18000)
    motor_2.abs_position_control(100,2,-18000)
    print(motor_1.motorStatus)
    print(motor_2.motorStatus)
    exit()

    motor_1.set_home(100)
    motor_1.motorStatus
    motor_2.set_home(100)
    motor_2.motorStatus
    print(motor_1._Homing, motor_2._Homing)
    while not(motor_1._stop and motor_2._stop):
        time.sleep(1)
        motor_1.motorStatus
        motor_2.motorStatus
    input('press any key to continue')
    motor_1.rel_position_control(1000, 3, 3200, 1)
    motor_1.motorStatus
    motor_2.rel_position_control(1000, 3, 3200, 0)
    motor_2.motorStatus
    while not (motor_1._stop and motor_2._stop):
        time.sleep(1)
        motor_1.motorStatus
        motor_2.motorStatus

    input('press any key to continue')
    n = 0
    while 1:
        motor_1.rel_position_control(1000, 1, 9600, 1)
        motor_1.motorStatus
        motor_2.rel_position_control(1000, 1, 9600, 0)
        motor_2.motorStatus
        while not (motor_1._stop and motor_2._stop):
            time.sleep(1)
            motor_1.motorStatus
            motor_2.motorStatus
        
        motor_1.rel_position_control(1000, 1, 9600, 0)
        motor_1.motorStatus
        if not motor_1._stop:
            pass
        else:
            print('control error, motor not move')
        motor_2.rel_position_control(1000, 1, 9600, 1)
        motor_2.motorStatus
        if not motor_1._stop:
            pass
        else:
            print('control error, motor not move')
        while not (motor_1._stop and motor_2._stop):
            time.sleep(1)
            motor_1.motorStatus
            motor_2.motorStatus
        n += 1
        print('**************************\n',n)

    BusHandle.close()
    

if __name__ == '__main__':
    main()
