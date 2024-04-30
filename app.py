import time
import threading
from functools import wraps


rcv = None
exit_flag = False
cmd_lock = False
hmi_map = {}
hmi_info = {}
hmi_motor = {}



from motor import MOTOR
from utils import *
from tcp_driver import *
from MSG import *
from control_config import *
from logger import set_log

class AXIS(MOTOR):
    def __init__(self, axis_name, address, BusHandle, config):
        super().__init__(address, BusHandle, config)
        self.name = axis_name


class SERVO(object):
    def __init__(self, port, boudrate, device, bus_idle, fresh_hz=2) -> None:
        self._log = set_log('servo_log','servo.log')
        self.BusHandle = Handle(port, boudrate,device, bus_idle)
        self.sleep = 1/fresh_hz
        self.axis = {}
        self.axis_map = {}
        self.axis_status = {} # [referenced, blocked, real_abs_pulse, real_speed]
        self.referenced = False
        self.restart_required = False
        pass

    def record_command(func):
        @wraps(func)
        def wrapper(self, *args, **kwargs):
            self._log.info('execute: ' + func.__name__)
            return func(self, *args, **kwargs)
        return wrapper
    

    @property
    def all_axis_stoped(self):
        for k in self.axis:
            # print(self.axis[k].name+': '+str(self.axis[k].motorStatus))
            if not self.axis[k].motorStatus[0]:
                return False
        return True

    def close(self):
        self.BusHandle.close()


    def operation_failed(self, axis_num, operation):
        r = input(
            f'No.{axis_num} Axis {self.axis[axis_num].name} {operation} failed, continue? input:Y/N\n',
            # timeout=50
            )
        # r = 'n'
        if r.lower() == 'y':
            pass
        elif r.lower() == 'n':
            self.close()
            raise ConnectionError
        else:
            self.close()
            raise ValueError
        

    @record_command
    def add_axis(self, axis_num:int, axis_name:str, address, config):
        if axis_num in self.axis:
            raise KeyError('Exist axis_num')
        try:
            a = AXIS(axis_name, address, self.BusHandle, config)
            self.axis[axis_num] = a
            self.axis_map[axis_name] = axis_num
        except Exception as e:
            print(e)
            self.operation_failed(axis_num, 'add axis')
        
    
    def info_show(self):
        global hmi_map
        global hmi_info
        global hmi_motor

        for k in self.axis:
            try:
                hmi_motor[k] = self.axis[k].motorStatus
                info_arr = [
                    self.axis[k]._homed,
                    self.axis[k].motorProtect,
                    self.axis[k].realPulse,
                    self.axis[k].realSpeed
                ]
                self.axis_status[k] = info_arr

                
            except Exception as e:
                print(e)
                self.operation_failed(k, 'get status')
        hmi_map = self.axis_map
        hmi_info = self.axis_status
            
    @record_command
    def reset_protect(self, axis_num=None):
        if isinstance(axis_num, int):
            if self.axis[k].reset_block_protect() != 0:
                self.operation_failed(axis_num, 'reset protect')
            else:
                if self.axis[k].motorProtect:
                    self.restart_required = True
        else:
            for k in self.axis:
                if self.axis[k].reset_block_protect() != 0:
                    self.operation_failed(k, 'reset protect')


    def soft_limit(self, axis_num, cmd_pulse,  mode='rel'):
        axis = self.axis[axis_num]
        real_ = axis.realPulse
        if mode == 'rel':
            tar_pulse = real_ + cmd_pulse
        else:
            tar_pulse = cmd_pulse
        if tar_pulse <= -5.11 or tar_pulse > axis.config['MOTION_CONFIG']['TOTAL_PULSE']:
            self._log.warning('illegal end position: start:{}, attempt:{}, tar:{}'.format(
                real_, cmd_pulse, tar_pulse
            ))
            return False
        else:
            return True


    @record_command
    def home(self, overtime=200):
        for k in self.axis:
            self.axis[k]._homed = False
            self.axis[k].set_home(overtime)
            
        self.referenced = True

    @record_command
    def position_to_rel_pulse(self, motion_request:dict):
        '''
        motion_request: dict
        {axis_name_1: {rel_puls:int, speed: int, at: int, dir: int}, axis_name_2:{}}
        '''
        cmd = [SYNC_CMD_HEAD]
        for name in motion_request:
            one_cmd = [0]*10
            m = motion_request[name]
            k = self.axis_map[name]
            s = m['speed']
            at = m['at']
            p = m['rel_puls']
            if m['dir'] == 1:
                if not self.soft_limit(k, -1 * p):
                    self.operation_failed(k, 'end position illegal')
            elif m['dir'] == 0:
                if not self.soft_limit(k, p):
                    self.operation_failed(k, 'end position illegal')
            
            assert s <= self.axis[k].config['MOTION_CONFIG']["MAX_SPEED"]
            assert at <= self.axis[k].config['MOTION_CONFIG']["MAX_ACCERATE_TIME"]
            
            d = m['dir']
            p = int(p * 5.11)
            if self.axis[k].config['MOTION_CONFIG']['CW_REVERSE'] == 1:
                d = 0 if m['dir'] == 1 else 1

            speed_base_code = 0x80 if d==0 else 0x00
            one_cmd[0] = self.axis[k].address
            one_cmd[1] = REL_CORD_POSITION_CONTROL[0][0]
            one_cmd[2] = ((s >> 8) & 0x0F) + speed_base_code
            one_cmd[3] =  s & 0xFF
            one_cmd[4] = int(256 - (at / 50e-3)) if at  != 0x00 else m['at'] 
            one_cmd[5] = (p >> 24) & 0xff
            one_cmd[6] = (p >> 16) & 0xff
            one_cmd[7] = (p >> 8) & 0xff
            one_cmd[8] = (p) & 0xff
            cmd += one_cmd
            del one_cmd
        while len(cmd) < 51:
            cmd += [0]*10
        if len(cmd) != 51:
            message.alm(self._log, message.tooMuchCommand)
        cmd += [checkSum(cmd)]
        cmd = bytearray(cmd)
        self.BusHandle.cmd(cmd, REL_CORD_POSITION_CONTROL, require_response=False)
        # confrim move
        time.sleep(0.3)
        for name in motion_request:
            k = self.axis_map[name]
            if self.axis[k].motorStatus[0]:
                if m['rel_puls'] > POSITION_THRESHOLD:
                    self.operation_failed(k, "axis move")
        return message.OK
    

    @record_command
    def position_to_position(self, motion_request:dict):
        '''
        motion_request: dict
        {axis_name_1: {abs_pos:int, speed: int, at: int}, axis_name_2:{}}
        '''
        cmd = [SYNC_CMD_HEAD]
        for name in motion_request:
            one_cmd = [0]*10
            m = motion_request[name]
            k = self.axis_map[name]
            s = m['speed']
            at = m['at']
            p = m['abs_puls']
            if not self.soft_limit(k, p, mode='abs'):
                self.operation_failed(k, 'end position illegal')
                pass
            
            assert s <= self.axis[k].config['MOTION_CONFIG']["MAX_SPEED"]
            assert at <= self.axis[k].config['MOTION_CONFIG']["MAX_ACCERATE_TIME"]
            p = p * 5.11
            if self.axis[k].config['MOTION_CONFIG']['CW_REVERSE'] == 1:
                p = -1 * p
            p = int(p)
            one_cmd[0] = self.axis[k].address
            one_cmd[1] = ABS_CORD_POSITION_CONTROL[0][0]
            one_cmd[2] = (s >> 8) & 0x0F
            one_cmd[3] =  s & 0xFF
            one_cmd[4] = int(256 - (at / 50e-3)) if at  != 0x00 else m['at'] 
            one_cmd[5] = (p >> 24) & 0xff
            one_cmd[6] = (p >> 16) & 0xff
            one_cmd[7] = (p >> 8) & 0xff
            one_cmd[8] = (p) & 0xff
            cmd += one_cmd
            del one_cmd
        while len(cmd) < 51:
            cmd += [0]*10
        if len(cmd) != 51:
            message.alm(self._log, message.tooMuchCommand)
        cmd += [checkSum(cmd)]
        cmd = bytearray(cmd)
        self.BusHandle.cmd(cmd, ABS_CORD_POSITION_CONTROL, require_response=False)
        # confrim move
        for name in motion_request:
            k = self.axis_map[name]
            if self.axis[k].motorStatus[0]:
                print(name, self.axis[k].realPulse, m['abs_puls'])
                # if abs(self.axis[k].realPulse-m['abs_puls']) > POSITION_THRESHOLD:
                #     self.operation_failed(k, "axis move")
        return message.OK
                

def service():
    global rcv
    global exit_flag
    global cmd_lock

    def startup():
        servo = SERVO('10.90.252.53', 1030, 'win', 0.2)
        servo.add_axis(0, 'X', 3, './XAXIS_CONFIG.json')
        servo.add_axis(1, 'Y', 2, './YAXIS_CONFIG.json')
        servo.home()
        return servo
    
    def hmi_open():
        global hmi_map
        global hmi_info
        global hmi_motor
        global exit_flag
        hmi = HMI()
        while not exit_flag:
            hmi.show(hmi_map, hmi_info, hmi_motor)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        hmi.close()
        exit_flag = True

    hmi_thread = threading.Thread(target=hmi_open)
    hmi_thread.start()

    act_accumulate = 0
    servo = startup()
    while not exit_flag:
        servo.info_show()
        cmd_lock = not servo.all_axis_stoped
        if cmd_lock:
            time.sleep(0.03)
            continue
        if act_accumulate >= HOME_ACCUMULATE:
            servo.home()
            act_accumulate = 0
        if isinstance(rcv, dict):
            if 'rel_pos' in rcv:
                pos = rcv['rel_pos']
                assert servo.position_to_rel_pulse(pos) == message.OK
                act_accumulate += 1
                    
            if 'reset_protect' in rcv:
                axis_num = rcv['reset_protect']
                assert servo.reset_protect(axis_num) == message.OK
            
            if 'abs_pos' in rcv:
                pos = rcv['abs_pos']
                assert servo.position_to_position(pos) == message.OK
                act_accumulate += 1

            if 'go_home' in rcv:
                servo.home()
            
            if 'reboot' in rcv:
                servo.close()
                servo = startup()

            if 'shutdown' in rcv:
                servo.close()
                exit_flag = True
        rcv = None
    print('service shutdown')


if __name__ == '__main__':
    import socket
    servo_thread = threading.Thread(target=service)
    servo_thread.start()

    serverPort = 15000
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.bind(('127.0.0.1', serverPort))
    serverSocket.listen(1)
    serverSocket.setblocking(False)
    serverSocket.settimeout(1)

    while not exit_flag:
        try:
            clientSocket, addr = serverSocket.accept()
        except:
            time.sleep(0.1)
            continue
        while not exit_flag:
            try:
                data = clientSocket.recv(2048).strip()
                dt = eval(data)
            except:
                time.sleep(0.1)
                continue
            if isinstance(dt, dict):
                # print('recive command: {}'.format(dt))
                pass
            else:
                # 向客户端发送响应数据
                resp = str({"code":500, "message":"wrong command type"}).encode('utf-8')
                clientSocket.sendall(resp)
            if not cmd_lock:
                if rcv:
                    resp=str({"code":400, "message":"Command is executing, wait and resend"}).encode('utf-8')
                    clientSocket.sendall(resp)
                else:
                    rcv = {dt['command']: dt['parameter']}
                    resp = str({"code":200, "message":"Sent"}).encode('utf-8')
                    clientSocket.sendall(resp)
            else:
                resp = str({"code":400, "message":"moving"}).encode('utf-8')
                clientSocket.sendall(resp)
            time.sleep(0.1)
        clientSocket.close()

    print('kill')

    exit_flag=True