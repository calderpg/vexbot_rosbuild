import serial
import time

class Qik2s9v1:

    def __init__(self, port, auto=False):
        self.qikport = serial.Serial(port, 38400, 8, 'N', 1, timeout=0.1)
        if auto == True:
            command = bytearray(1)
            command[0] = 0xaa
            self.qikport.write(command)
        self.lasterror = bytearray(1)
    
    def GoPercent(self, m0, m1):
        if abs(m0) > 1.0 or abs(m1) > 1.0:
            print "Motors commanded out of speed bounds"
            raise BaseException
        command_bytes = self.ConvertToRawBytes(m0, m1)
        self.qikport.write(command_bytes)
        self.GetErrorByte()
        
    def ConvertToRawBytes(self, m0, m1):
        command_bytes = bytearray(4)
        if m0 >= 0.0:
            command_bytes[0] = 0x88
        else:
            command_bytes[0] = 0x8a
        if m1 >= 0.0:
            command_bytes[2] = 0x8c
        else:
            command_bytes[2] = 0x8e
        command_bytes[1] = int(127.0 * abs(m0))
        command_bytes[3] = int(127.0 * abs(m1))
        return command_bytes
        
    def GetErrorByte(self):
        command_bytes = bytearray(1)
        command_bytes[0] = 0x82
        self.qikport.write(command_bytes)
        temp_error = self.qikport.read(1)
        self.lasterror = ord(temp_error[0])
        
    def Brake(self):
        command_bytes = bytearray(4)
        command_bytes[0] = 0x88
        command_bytes[2] = 0x8c
        command_bytes[1] = 0x00
        command_bytes[3] = 0x00
        self.qikport.write(command_bytes)
        
    def Stop(self):
        command_bytes = bytearray(4)
        command_bytes[0] = 0x88
        command_bytes[2] = 0x8c
        command_bytes[1] = 0x00
        command_bytes[3] = 0x00
        self.qikport.write(command_bytes)
        
    def Coast(self):
        command_bytes = bytearray(2)
        command_bytes[0] = 0x86
        command_bytes[1] = 0x87
        self.qikport.write(command_bytes)
        
    def GetLastError(self):
        byte_data = self.lasterror
        errors = ""
        if byte_data&1 != 0:
            errors = errors + "_NotApplicable_"
        if byte_data&2 != 0:
            errors = errors + "_NotApplicable_"
        if byte_data&4 != 0:
            errors = errors + "_NotApplicable_"
        if byte_data&8 != 0:
            errors = errors + "_Buffer_"
        if byte_data&16 != 0:
            errors = errors + "_Frame_"
        if byte_data&32 != 0:
            errors = errors + "_CRC_"
        if byte_data&64 != 0:
            errors = errors + "_Format_"
        if byte_data&128 != 0:
            errors = errors + "_Timeout_"
        return errors
        
if __name__ == '__main__':
    myqik = Qik2s9v1("COM3", False)
    speed = -1.0
    while speed <= 1.0:
        myqik.GoPercent(speed, speed)
        speed = speed + .1
        time.sleep(1)
    myqik.Stop()
