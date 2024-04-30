import serial
import time
from MSG import message

try:
    import RPi.GPIO as GPIO 
except ImportError:
    print("Platform is not Rasberry Pi")


CMD_HEAD = 0xFA
RES_HEAD = 0xFB

def motion_time(s, at, p):
    return p / (s * 4000 / 60) + 2*at/1000

def recovery_TWORD(arr):
    return (arr[0]<<48) | (arr[1]<<36) | (arr[2]<<24) | (arr[3]<<16) | (arr[4]<<8) | arr[5]

def recovery_DWORD(arr):
    return (arr[0]<<24) | (arr[1]<<16) | (arr[2]<<8) | arr[3]

def recovery_WORD(arr):
    return (arr[0]<<8) | arr[1]

def bit_flag(byte):
    bits = [1,2,4,8,16,32,64,128]
    return [(byte & b) == b for b in bits]

def checkSum(arr):
    return sum(arr) & 0xFF

class Handle(object):
    def __init__(self, motor_id, boudrate, device='win', IDLE_time=0.0005) -> None:
        self.conn = serial.Serial(motor_id, boudrate)
        self.IDLE = IDLE_time
        self.device = device
        if device == 'win':
            pass
        elif device == 'rpi':
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(4, GPIO.OUT)

    def close(self):
        self.conn.close()

    def check_cmd(self, commands):
        if commands[0] != CMD_HEAD:
            return message.BusCmdErr
        exp_sum = checkSum(commands[:-1])
        cmd_sum = commands[-1]
        
        if cmd_sum == exp_sum:
            return message.OK
        else:
            return message.CheckSumErr
    
    def check_ret(self, ret, command_code):
        st, ed = None, None
        cmd_accu = None
        rets = []
        for i,b in enumerate(ret):
            if b == RES_HEAD and st is None:
                st = i
                cmd_accu = [RES_HEAD]
                continue
            if st is not None:
                if len(cmd_accu) == command_code[1]:
                    if b == checkSum(cmd_accu):
                        ed = i
                        rets.append(cmd_accu+[ret[ed]])
                        st, ed = None, None
                else:
                    cmd_accu.append(b)
        
        if st is None and rets == []:
            return message.RetErr_WrongAddr, None
        if ed is None and rets == []:
            return message.RetErr_WrongCheckSum, None
        return message.OK, rets

    def cmd(self, commands, command_code, require_response=True):
        # check checksum
        if self.device == 'rpi':
            GPIO.output(4, GPIO.HIGH)
        res = self.check_cmd(commands)
        print([c for c in commands])
        if res == message.OK:
            pass
        else:
            message.alm(res)
        self.conn.write(commands)
        self.conn.flush()
        if require_response:
            time.sleep(self.IDLE)
            # waiting return
            res, ret = self.read(command_code)
            if res == message.OK:
                return res, ret
        else:
            return res, None
            

    def read(self, command_code):
        if self.device == 'rpi':
            GPIO.output(4, GPIO.LOW)
        ret = []
        while self.conn.inWaiting()>0:
            ret += self.conn.read(self.conn.inWaiting())
        if len(ret) > 0:
            print(ret)
            res, ret = self.check_ret(ret, command_code)
            print(ret)
            if res == message.OK:
                return res, ret
            else:
                message.alm(res)
        else:
            return message.OK, []

