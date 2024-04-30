class Message(object):
    def __init__(self) -> None:
        self.OK = 0x00
        
        self.BusTimeout=0x03
        self.BusCmdErr=0x04
        self.CheckSumErr=0x05
        self.RetErr_WrongAddr=0x06
        self.RetErr_WrongCheckSum=0x07
        self.RetErr_NoMessage=0x08

        self.tooMuchCommand = 0x09
        

        self.CommandErr_MotorEn = 0xA0
        self.CommandRej_MotorEn = 0xA1
        self.CommandErr_SpCtl = 0xA2
        self.CommandRej_SpCtl = 0xA3
        self.CommandErr_PosCtl = 0xA4
        self.CommandRej_PosCtl = 0xA5
        self.CommandErr_SetHome = 0xA6
        self.CommandRej_SetHome = 0xA7
        self.CommandErr_ActHome = 0xA8
        self.CommandRej_ActHome = 0xA9
        self.CommandErr_StopHome = 0xAA
        self.CommandRej_StopHome = 0xAB
        self.CommandErr_GetMotorStat = 0xAC
        self.CommandErr_Ver = 0xAD
        self.CommandErr_Info = 0xAE
        self.CommandErr_SyncMove = 0xB0
        self.CommandRej_SyncMove = 0xB1
        self.CommandRej_setProtect = 0xB2
        self.CommandRej_relProtect = 0xB3
        self.CommandErr_getAbsPos = 0xB4
        self.CommandErr_getRealSpeed = 0xB5
        



        self.sentence = {
            0x00:'Complete',
            0x03:'TCP Communicate timeout error',
            0x04:'Head of Bus command is Error',
            0x05:'Checksum is Error',
            0x06:'Return Message is Error: Wrong Response Head',
            0x07:'Return Message is Error: Wrong Check',
            0x08:'Command not recive any Response',
            0x09:'Too many command in multioperation command',

            0xA0:'Motor enable command Error',
            0xA1:'Motor enable not allowed',
            0xA2:'Speed Control command Error',
            0xA3:'Speed Control not allowed',
            0xA4:'Position Control command Error',
            0xA5:'Position Control not allowed',
            0xA6:'Home position set command Error',
            0xA7:'Home position set not allowed',
            0xA8:'Back to Home Positon command Error',
            0xA9:'Back to Home Positon not allowed',
            0xAA:'Terminate Back to Home operation command Error',
            0xAB:'Terminate Back to Home operation not allowed',
            0xAC:'Get Motor Status command Error',
            0xAD:'Get Board Version command Error',
            0xAE:'Get System Information command Error',
            0xB0:'Sync Move command Error',
            0xB1:'Sync Move command not allowed',
            0xB2:'Set Motor Block Protect command Error',
            0xB3:'Release Motor Block Protect command Error',
            0xB4:'Get Absoulate position pulse command Error',
            0xB5:'Get real speed command Error',
        }

    def alm(self,log_handle, code):
        log_handle.error(self.sentence[code])
        raise ValueError(self.sentence[code])

message = Message()