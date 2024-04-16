from serial import Serial
from math import pow

class GripperControl():
    currentDeg = 0
    
    def __init__(self, port):
        self.ser = Serial(port, 115200, timeout= 1)
        self.ser.reset_input_buffer

    def XORsum(self, a):
        out = not a[0]
        for i in a:
            out = out ^ i
            
        return out
    
    def degToHex(self, deg):
        steps = f'{round(deg * 267.42857):04x}'
        return f'{steps[0:2]} {steps[2:]}'
    
    def getHexbit(self, dir, deg, show= False):
        in_bits = f'7b 01 02 0{dir} 20 {self.degToHex(deg)} 00 c8'
        bits = [int(i, 16) for i in in_bits.split()]
        msg = (''.join(in_bits)) + ' ' +  f'{self.XORsum(bits):02x} 7d'
        if show: print('Hexbits:', msg)
        
        return bytes.fromhex(msg)
        

    def drive(self, dir, deg, show = False):
        self.currentDeg += pow(-1, not dir) * deg
        hexbit = self.getHexbit(dir, deg, show)
        self.ser.write(hexbit)
        
    def driveTo(self, pos, show = False):
        deg = pos - self.currentDeg
        
        if deg >= 0: hexbit = self.getHexbit(1, deg, show)
        else: hexbit = self.getHexbit(0, -1 * deg, show)
        
        self.currentDeg+= deg
        if show: print('Current Pos:', self.currentDeg)
        
        self.ser.write(hexbit)
        
    def _open(self):
        self.driveTo(0)
    
    def _close(self):
        self.driveTo(70)
        
        

if __name__ == '__main__':
    gripper = GripperControl('COM4')

    while True:
        _in = input('Enter: ')
        try:
            _dir, _deg = [int(i) for i in _in.split()]
        except:
            _deg = int(_in)
        
        # print(_dir, _deg)
        # gripper.drive(_dir, _deg, True)
        # print(gripper.currentDeg)
        gripper.driveTo(_deg, True)
        print(gripper.degToHex(70))

        
        