import socket
import time

from utils import checkSum
from MSG import message
from logger import set_log

CMD_HEAD = 0xFA
RES_HEAD = 0xFB
SYNC_CMD_HEAD = 0XFC


class Handle(object):
    def __init__(self, ip_address, port, device='win', IDLE_time=0.0005) -> None:
        self.conn = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.conn.connect((ip_address, port))
        self.IDLE = IDLE_time
        self.device = device
        self._log = set_log('bus_log', 'bus.log')
        self._log.info('TCP Bus setup')

    def close(self):
        self.conn.close()

    def check_cmd(self, commands):
        if commands[0] not in (CMD_HEAD, SYNC_CMD_HEAD):
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
        self._log.info('send:{} with response={}'.format([c for c in commands], require_response))
        res = self.check_cmd(commands)
        if res == message.OK:
            pass
        else:
            message.alm(self._log, res)
        self.conn.send(commands)
        
        if require_response:
            res, ret = self.read(command_code)
            if res == message.OK:
                return res, ret
            
        else:
            time.sleep(self.IDLE)
            return res, None
            

    def read(self, command_code):
        try:
            self.conn.settimeout(3)
            ret = self.conn.recv(1024)
        except socket.timeout:
            message.alm(self._log, message.BusTimeout)
        if ret:
            # print([r for r in ret])
            res, ret = self.check_ret(ret, command_code)
            if res == message.OK:
                self._log.info('recive:{} '.format(ret))
                return res, ret
            else:
                message.alm(self._log, res)
        else:
            return message.OK, []

