import serial
import time

class ArduBot:

    def __init__(self, port):
        self.arduino = serial.Serial(port, 9600, 8, 'N', 1, timeout=0.1)
        time.sleep(1)
        self.arduino.write('x')
        print self.arduino.readline()
        self.Stop()
    
    def GoPercent(self, m0, m1):
        if abs(m0) > 1.0 or abs(m1) > 1.0:
            print "Motors commanded out of speed bounds"
            raise BaseException
        command_bytes = self.ConvertToRawBytes(m0, m1)
        self.arduino.write(command_bytes)
        return self.arduino.readline()
        
    def ConvertToRawBytes(self, m0, m1):
        command_bytes = bytearray(6)
        command_bytes[0] = '$'
        if m0 >= 0.0:
            command_bytes[1] = 'F'
        else:
            command_bytes[1] = 'R'
        if m1 >= 0.0:
            command_bytes[3] = 'F'
        else:
            command_bytes[3] = 'R'
        command_bytes[2] = int(255.0 * abs(m0))
        command_bytes[4] = int(255.0 * abs(m1))
        command_bytes[5] = '\n'
        return command_bytes
        
    def Stop(self):
        self.GoPercent(0.0, 0.0)

#Startup code run only if this file is executed directly        
if __name__ == '__main__':
    mybot = ArduBot("COM19")
    speed = -1.0
    while speed <= 1.0:
        mybot.GoPercent(speed, speed)
        speed = speed + .1
        time.sleep(1)
    mybot.Stop()
