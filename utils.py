import sys
import selectors
import cv2
import numpy as np


def timeout_input(msg, default='', timeout=5):
    sys.stdout.write(msg)
    sys.stdout.flush()
    sel = selectors.DefaultSelector()
    sel.register(sys.stdin, selectors.EVENT_READ)
    events = sel.select(timeout)
    if events:
        key, _ = events[0]
        return key.fileobj.readline().rstrip()
    else:
        sys.stdout.write('\n')
        return default
    

def motion_time(s, at, p):
    return p / (s * 4000 / 60) + 2*at/1000

def recovery_TWORD(arr):
    b = bytearray(arr)
    ret = int.from_bytes(b, 'big', signed=True)
    #(arr[0]<<48) | (arr[1]<<36) | (arr[2]<<24) | (arr[3]<<16) | (arr[4]<<8) | arr[5]
    return ret

def recovery_DWORD(arr):
    # eturn (arr[0]<<24) | (arr[1]<<16) | (arr[2]<<8) | arr[3]
    b = bytearray(arr)
    ret = int.from_bytes(b, 'big', signed=True)
    return ret

def recovery_WORD(arr):
    # return (arr[0]<<8) | arr[1]
    b = bytearray(arr)
    ret = int.from_bytes(b, 'big', signed=True)
    return ret

def bit_flag(byte):
    bits = [1,2,4,8,16,32,64,128]
    return [(byte & b) == b for b in bits]

def checkSum(arr):
    return sum(arr) & 0xFF


class HMI(object):
    def __init__(self) -> None:
        # Create a black background image
        width, height = 800, 600
        self.background = np.zeros((height, width, 3), dtype=np.uint8)
        # Define text parameters
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.8
        self.font_thickness = 2
        self.font_color = (0, 255, 0)  # Green color

        # Add text to the image
        self.text_height = 30  # Height of each text line
        self.text_lines = []

        self.window = 'HMI'
        cv2.imshow(self.window, self.background)
    
    def show(self, map, info, flags):
        text_line = ['************************************']
        self.frame = self.background.copy()
        for n in map:
            mes = info[map[n]]
            flag = flags[map[n]]
            line = n+'   '
            line += '    POS:    '+str(mes[2])
            line += '    SPEED:    '+str(mes[3])
            text_line.append(line)
            line_2 = '        '+'REF:'+str(mes[0])
            line_2 += '    '+'PROTECTED:'+str(mes[1])
            text_line.append(line_2)
            line_3 = '        '+'STOP:   '+str(int(flag[0]))
            line_3 += '    '+'ACC:   '+str(int(flag[1]))
            line_3 += '    '+'DEC:   '+str(int(flag[2]))
            line_3 += '    '+'HOME:   '+str(int(flag[4]))
            text_line.append(line_3)
            text_line.append('************************************')

        for i, text in enumerate(text_line):
            text_position = (20, 30 + i * self.text_height)
            cv2.putText(self.frame, 
                        text, text_position, 
                        self.font, self.font_scale, 
                        self.font_color, 
                        self.font_thickness)
        cv2.imshow(self.window, self.frame)

    def close(self):
        cv2.destroyAllWindows()