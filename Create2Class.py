
import struct
import time
try:
    import serial
except ImportError:
    print "missing serial library"
    raise

#constants
WHEEL_PERIMETER = 226.19    # wheel perimeter in mm
WHEEL_COUNTS = 511.0        # encoder counts per full revolution
DRIVE_PERIMETER = 738.2742  # 2*pi*115.4; 115.4 = Create2 centre to wheel centre
CLIFF_THRES = 100           # threshold for cliff sensors to detect a cliff
DRIVE_SPEED = 80            # suitable speed for driving straight
TURN_SPEED = 80             # suitable speed for turning (on the spot)


class Create2Class:
    def __init__(self, path):
        self.onConnect(path) 

    def sendRawCommand(self, command):
        global create2
        
        try:
            if create2 is not None:
                create2.write(command)
            else:
                print "no connection to Create2"
        except serial.SerialException:
            print "disconnected from Create2"
            create2 = None

        #debugging: print the command sent to Create2
       # print "sent " +  ' '.join([ str(ord(c)) for c in command ])

    def sendASCIICommand(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))
        self.sendRawCommand(cmd)

    def onConnect(self, path):
        """function to start serial connection with iRobot Create2"""
        global create2

        try:
            create2 = serial.Serial(path, baudrate = 115200, timeout=10)
            print "connected to Create2, beeping now, press CLEAN on the Create2 and restart node if no melody is heard."
            time.sleep(0.03)
           # sendASCIICommand('140 3 1 81 16 141 3')
           # time.sleep(0.5)
           # sendASCIICommand('140 3 1 81 16 141 3')
        except serial.SerialException:
            print "Failed to connect to Create2, did you plug in the serial cable or change the device path?"

    def drive(self, velocity, radius):
        """function to drive Create2(this doesnt seem to work on versions < 3.3.0)
        Param:  velocity:   average velocity of drive wheels in mm/s, positive=forward, negative=reverse
                radius:     turning radius in mm, centre of turning circle to centre of Create2, positive=left turn, negative=right turn.
        Special cases:  straight: 32768(0x8000) or 32767(0x0001)
                        turn in place clockwise: -1(0xFFFF)
        Return: none (for now)
        """
        v = self.twos_comp(velocity)
        r = self.twos_comp(radius)
        cmd = struct.pack(">BBBBB", 137, v[0], v[1], r[0], r[1])
        self.sendRawCommand(cmd) 

    def driveDirect(self, velocity_left, velocity_right):
        """directly control Create2's left and right drive wheels
        Param: left and right motor velocities in mm/s
        Return: none
        """
        vr = self.twos_comp(velocity_right)
        vl = self.twos_comp(velocity_left)
        cmd = struct.pack(">BBBBB", 145, vr[0], vr[1], vl[0], vl[1])
        self.sendRawCommand(cmd)


    def twos_comp(self, num):
        """computes 16 bit 2's complement on number
        Param: num (16 bit signed number)
        Return: list containing [MSB,LSB] which is 2's comp of num
        """
        if num < 0:
            num = (1<<16) + num
        return divmod(num, 256)

    def rev_twos_comp(self, num):
        """computes reverse twos complement on number 
        Param: num (list of 2 8bit numbers [MSB,LSB])
        Return: reverse complemented num
        """
        number = (ord(num[0])<<8) | ord(num[1])
        if number & 0x8000 == 1:
            return (1<<16) - number
        else:
            return number


    def readCliffSensors(self):
        """Returns a dictonary of the value of the 4 underside cliff sensors
        Param: None
        Return: {'left':wwww, 'frontleft':xxxx, 'frontright':yyyy, 'right':zzzz]
                (0 = nocliff, 1 = cliff)
        """
        cliffs = {'left': 0, 'frontleft': 0, 'frontright': 0, 'right': 0}
      
        self.sendASCIICommand('149 4 9 10 11 12')
        dataRX = create2.read(4)
        cliffs['left'] = ord(dataRX[0])
        cliffs['frontleft'] =ord( dataRX[1])
        cliffs['frontright'] = ord(dataRX[2])
        cliffs['right'] = ord(dataRX[3])
        
        return cliffs 

    def readWallSignal(self):
        """Returns the value of the wall signal sensor
                for use with virtual walls (untested)
        Param: None
        Return: Range: 0-1023
        """
        self.sendASCIICommand('142 27')
        wallSignal = create2.read(2)
        return (ord(wallSignal[0])<<8) | ord(wallSignal[1])

    def readBumpers(self):
        """Returns the current state of the front left and right bumpers
        Param: None
        Return: 0: left unpressed. right unpressed
                1: left unpressed, right   pressed
                2: left   pressed, right unpressed
                3: left   pressed, right   pressed
        """
        self.sendASCIICommand('142 7')
        bumperData = ord(create2.read(1))
        return bumperData & 3

    def readVoltage(self):
        """Returns current Create2 battery voltage in millivolts (0 to 65535 mV)
        Param: None
        Return: voltage
        """
        self.sendASCIICommand('142 22')
        voltage = create2.read(2)
        return (ord(voltage[0])<<8) | ord(voltage[1])

    def readCharge(self):
        """Returns the current charge of Create2's battery in mAh (0 to 65535 mAh)
        Param: None
        Return: charge
        """
        self.sendASCIICommand('142 25')
        charge = create2.read(2)
        return (ord(charge[0])<<8) | ord(charge[1])
        

    def readCapacity(self):
        """Returns estimated charge capacity of Create2's battery in mAh (0 to 65535 mAh)
        Param: None
        Return: capacity
       """
        self.sendASCIICommand('142 26')
        capacity = create2.read(2)
        return (ord(capacity[0])<<8) | ord(capacity[1])

    def getChargeState(self):
        """returns the current charging state as a string
        Param: None
        Return: charge state(str)
        """
        self.sendASCIICommand('142 21')
        state = ord(create2.read(1))
        if state == 0:
            return "Not charging"
        elif state == 1:
            return "Reconditioning Charging"
        elif state == 2:
            return "Full Charging"
        elif state == 3:
            return "Trickle Charging"
        elif state == 4:
            return "Waiting"
        elif state == 5:
            return "Charging Fault Condition"
        else:
            return "error"

    def getFirmware(self):
        """
        get firmware version on the Create2 (and resets the Create2)
        """
        self.sendASCIICommand('7')
        print str(create2.read(300))
        self.sendASCIICommand('128') # send the start sequence again
        self.sendASCIICommand('132') # put the Create2 into full mode

    def getDistance(self):
        #WARNING: DOES NOT WORK FOR FIRMWARES < 3.3.0
        """returns distance travelled by Create2 since last polled
            distance = (left_travelled+right_travelled)/2
        Param: None
        Return: distance travelled
        """
        self.sendASCIICommand('142 19')
        dist = create2.read(2)
        distance = rev_twos_comp(dist)
        return distance

    def getEncoderCountL(self):
        """function to grab current encoder counts on left wheel
        Parameters: None 
        return: encoder count on left wheel (-32767 to 32767)
        """
        self.sendASCIICommand('142 43')
        count_left = create2.read(2)
        left = (ord(count_left[0])<<8) | ord(count_left[1])
        if left > 32767:
            left = 0 - (65536 - left)
        return left

    def getEncoderCountR(self):
        """function to grab current encoder counts on right wheel
        Parameters: None
        return: encoder count on right wheel (-32767 to 32767)
        """
        self.sendASCIICommand('142 44')
        count_right = create2.read(2)
        right = (ord(count_right[0])<<8) | ord(count_right[1])
        if right > 32767:
            right = 0 - (65535 - right)
        return right

    def driveDistance(self, velocity_left, velocity_right, des_distance, if_straight):
        """function to drive the Create2 at a certain speed for a set distance.  (speed min: ~30mm/s)
        Param: speed_xxxx: speed to spin right and left wheel (mm/s)
               distance  : distance to drive, + forward, - back
               if_straight: 1 if driving straight (velocity_left==velocity_right), 0 if turning (velocity_left!=velocity_right)
        Return: ErrorCode: 
        """
        count0 = self.getEncoderCountL()     #lets use just the left encoder
        print "count0: " + str(count0)
        
        if if_straight == 1:
            if des_distance < 0:
                self.driveDirect(-1*velocity_left, -1*velocity_right)
            else:
                self.driveDirect(velocity_left, velocity_right)
        else:
            self.driveDirect(velocity_left, velocity_right)

        count1 = self.getEncoderCountL()
        print "count1: " + str(count1)
        cur_distance = ( ((count1-count0)/WHEEL_COUNTS)*WHEEL_PERIMETER ) 
        print "cur_dis: " + str(cur_distance)
        print "des_dis: " + str(des_distance)
        #keep refreshing distance while robot drives
        while (cur_distance < (des_distance-1)) | (cur_distance > (des_distance+1)):
            print "count0: " + str(count0)
            count1 = self.getEncoderCountL()
            print "count1: " + str(count1)
            cur_distance = ((count1-count0)/WHEEL_COUNTS)*WHEEL_PERIMETER 
            print "cur_dis: " + str(cur_distance)
            print "des_dis: " + str(des_distance)
        
        #stop robot
        self.driveDirect(0,0)
        return 0

    def driveAngle(self, des_angle):
        """function to rotate the Create2 by (+/-)angle
        Param: angle: positive(ccw rotation)
                      negative(cw rotation)
        Return: error: if any sensors were tripped
        """
        arclength = (float(abs(des_angle))/360.0) * DRIVE_PERIMETER 
        
        #since we are only reading from left encoder in driveDistance()
        if des_angle > 0:
            arclength *= -1
        else:
            arclength = arclength
        
        print arclength
        if des_angle < 0:
            self.driveDistance(TURN_SPEED, (-1)*TURN_SPEED, arclength, 0) 
        else:
            self.driveDistance((-1)*TURN_SPEED, TURN_SPEED, arclength, 0)
        return 0

    def driveToCliff(self):
        """drive straight until cliff sensor set off and get perpendicular to cliff
        Param: none
        Return: distance travelled (mm)
        """
        cliffs = self.readCliffSensors()
        count0 = self.getEncoderCountL()
        
        #drive straight until a cliff sensor goes off
        while (cliffs['left'] == 0) & (cliffs['frontleft'] == 0) & (cliffs['frontright'] == 0) & (cliffs['right'] == 0):
            self.driveDirect(DRIVE_SPEED,DRIVE_SPEED)
            cliffs = self.readCliffSensors()
        
        self.driveDirect(0,0)
        count1 = self.getEncoderCountL()
        distance = ( ((count1-count0)/WHEEL_COUNTS)*WHEEL_PERIMETER )
        cliffs = self.readCliffSensors()
        
        #straighten up if one of the side sensors go off (we want to be perpendicular to the edge)
        if (cliffs['left'] == 1):
            self.driveAngle(45)
        self.driveDirect(0,0)
        if (cliffs['right'] == 1):
            self.driveAngle(-45)
        self.driveDirect(0,0)

        self.driveDistance(DRIVE_SPEED/2, DRIVE_SPEED/2, -50, 1) #backup a bit so we dont run off the cliff (bad)

        #now we are relatively perpendicular to the cliff, use the front cliff sensors to line up with the cliff
        verifyPerpendicular = 0 # keep track of tries to line up with the cliff

        while verifyPerpendicular < 3:
            time.sleep(0.5)
            self.driveDistance(DRIVE_SPEED/3, DRIVE_SPEED/3, 2, 1) #inch forward until cliff
            self.driveDirect(0,0)
            cliffs = self.readCliffSensors()

            #this is good, means front cliff sensors are lined up
            if (cliffs['frontleft'] == 1) & (cliffs['frontright'] == 1):
                self.driveDirect(0,0)
                time.sleep(0.5)
                self.driveDistance(DRIVE_SPEED/2, DRIVE_SPEED/2, -50, 1) #back up
                verifyPerpendicular += 1
                self.sendASCIICommand('164 71 79 79 68') #display good

            #Create2 is left biased, need to rotate ccw to centre
            elif (cliffs['frontleft'] == 1) & (cliffs['frontright'] == 0):
                self.driveDirect(0,0)
                self.sendASCIICommand('164 76 69 70 84')
                time.sleep(0.5)
                #drive only the right wheel until frontright sensor is off cliff
                while (cliffs['frontright'] == 0):
                    self.driveDirect(0, TURN_SPEED)
                    cliffs = self.readCliffSensors()
                self.driveDirect(0,0)
                time.sleep(0.3)
                self.driveDistance(DRIVE_SPEED/2, DRIVE_SPEED/2, -50, 1) #back up

            #Create2 is right biased, need to rotate cw to centre
            elif (cliffs['frontleft'] == 0) & (cliffs['frontright'] == 1):
                self.driveDirect(0,0)
                self.sendASCIICommand('164 82 73 84 69') #display 'RITE'
                time.sleep(0.5)
                #drive only the left wheel until frontleft sensor is off cliff
                while (cliffs['frontleft'] == 0):
                    self.driveDirect(TURN_SPEED, 0)
                    cliffs = self.readCliffSensors()
                self.driveDirect(0,0)
                time.sleep(0.3)
                self.driveDistance(DRIVE_SPEED/2, DRIVE_SPEED/2, -50, 1) #back up
       
        return distance-50

    def readButtons(self):
        """ function to determine which button on the Create2 are pressed
        Param: None
        Return: dictionary of buttons states: (0=not pressed, 1=pressed)
        WARNING: Schdule and Clock buttons don't seem to work here
        """
        buttonStates = {'Clock': 0, 'Schedule': 0, 'Day': 0, 'Hour': 0, 'Minute': 0, 'Dock': 0, 'Spot': 0, 'Clean': 0} 
        self.sendASCIICommand('142 18')
        dataRX = create2.read(1)
        buttonStates['Clock'] = ord(dataRX) & 0b10000000
        buttonStates['Schedule'] = ord(dataRX) & 0b01000000
        buttonStates['Day'] = ord(dataRX) & 0b00100000
        buttonStates['Hour'] = ord(dataRX) & 0b00010000
        buttonStates['Minute'] = ord(dataRX) & 0b00001000
        buttonStates['Dock'] = ord(dataRX) & 0b00000100
        buttonStates['Spot'] = ord(dataRX) & 0b00000010
        buttonStates['Clean'] = ord(dataRX) & 0b00000001
        return buttonStates

    def closeCreate2(self):
        """function to destroy the object
        """
        self.sendASCIICommand('173') #stop the OI 
        create2.close() #close the serial connection

""" below is an old standalone demo of the functions in this class.  It will not work as is, can you make it work?  what do you think it does? answer at the EOF
###############################################  
####              "MAIN LOOP"              ####  
###############################################

onConnect()
time.sleep(0.03)
sendASCIICommand('137 0 0 128 0') # stop motors
time.sleep(1)
#getFirmware()
sendASCIICommand('128') #start OI
time.sleep(0.03)
sendASCIICommand('132') #full mode, who needs safeties
print "Battery voltage: " + str(readVoltage()) + "mV"
print "Battery Charge: " + str(readCharge()) + "mAh"
print "Battery Capacity: " + str(readCapacity()) + "mAh"
print "Charging State: " + str(getChargeState())
print '\n'

#while loop, press dock to exit
buttonStates = readButtons()
while buttonStates['Dock'] == 0:
    sendASCIICommand('164 80 82 69 83') #display 'PRES'
    buttonStates = readButtons()
    sendASCIICommand('139 4 0 255') # turn on LED green
    time.sleep(0.5)
    sendASCIICommand('139 4 0 0')
    buttonStates = readButtons()
    time.sleep(0.5)


    if buttonStates['Clean'] > 0:
        '''
        sendASCIICommand('140 3 1 81 16 141 3') #beep
        time.sleep(0.03)
        sendASCIICommand('140 3 1 81 16 141 3') #beep
        time.sleep(0.03)
        sendASCIICommand('140 3 1 81 16 141 3') #beep
        time.sleep(0.03)
        driveToCliff() #initially find an edge
        time.sleep(0.03)
        driveAngle(180) #turn all the way around
        length = driveToCliff() #drive to other edge
        driveDistance(DRIVE_SPEED, DRIVE_SPEED, -1*(length/2), 1) #go backwards halfway
        driveAngle(90) #rotate 90 to face the adjacent edge
        driveToCliff() #go to one edge
        driveAngle(180) #turn around
        width = driveToCliff() #drive all the way to the other edge and save distance
        driveDistance(DRIVE_SPEED, DRIVE_SPEED, -1*(width/2), 1) #go backwards, should be in centre of table now
        sendASCIICommand('140 3 1 81 16 141 3') #beep
        '''       
        #test variables
        length = 123456.789 
        width =  987654.321
        #NOW DISPLAY LENGTH AND WIDTH
        buttonStates = readButtons()
	
        #make chars_xxx variables exact multiples of 4 
        if (len(length_ascii) % 4) > 0:
            chars_len += 4
        if (len(width_ascii) % 4) > 0:
            chars_wid += 4
      
        #now we append ascii code for blank characters to the end of the digits list
        if chars_len > len(length_ascii):
            for k in range(0, (chars_len-len(length_ascii))):
                length_ascii.append(32)
        if chars_wid > len(width_ascii):
            for k in range(0, (chars_wid-len(width_ascii))):
                width_ascii.append(32)

        #now we print the digits (their ascii representation) 4 at a time, 3 seconds each
        print "length:"
        for i in range(0, chars_len, 4):
            sendRawCommand(struct.pack(">BBBBB", 164, length_ascii[i+0], length_ascii[i+1], length_ascii[i+2], length_ascii[i+3] ))
            time.sleep(3)
        print "width:"
        for i in range(0, chars_wid, 4):
       	    sendRawCommand(struct.pack(">BBBBB", 164, width_ascii[i+0], width_ascii[i+1], width_ascii[i+2], width_ascii[i+3] ))    
            time.sleep(3)

        buttonStates = readButtons()


print "done for now, stopping OI and disconnecting"
sendASCIICommand('173') # stop the OI
time.sleep(0.03)
      
create2.close()
"""  






#the above code will measure the length and width of a square surface(table), then display the measurements on the 7seg displays.
