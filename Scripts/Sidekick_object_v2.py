import board
import time
from digitalio import DigitalInOut, Direction, Pull
import pwmio
from adafruit_motor import servo
from adafruit_motor import stepper
from Kinematics import kinematicsfunctions as kf
from Kinematics import plategen

class Sidekick:
    
    def __init__(self,L1,L2,L3,Ln,Origin,Home,Effector_Location):
        
        # Dimensional Attributes (in cm)
        
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.Ln = Ln
        
        # Locational Attributes, home is subject to change, right now it is over well a12
        self.home = [90,178]
        self.origin = Origin
        self.current = Effector_Location #Angular location of servo
        self.purge = [52.1997, 90.4495]
        
        # Well Lookup Map, the default 96 well map is loaded here, but should be recalibrated with the remap method
        #self.wellids = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8', 'a9', 'a10', 'a11', 'a12', 'b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7', 'b8', 'b9', 'b10', 'b11', 'b12', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'd1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7', 'd8', 'd9', 'd10', 'd11', 'd12', 'e1', 'e2', 'e3', 'e4', 'e5', 'e6', 'e7', 'e8', 'e9', 'e10', 'e11', 'e12', 'f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'f8', 'f9', 'f10', 'f11', 'f12', 'g1', 'g2', 'g3', 'g4', 'g5', 'g6', 'g7', 'g8', 'g9', 'g10', 'g11', 'g12', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'h7', 'h8', 'h9', 'h10', 'h11', 'h12']
        #self.alltheta1 = [41.1831, 50.2859, 59.7319, 69.3711, 78.9405, 88.0794, 96.3887, 103.529, 109.302, 113.668, 116.711, 118.583, 40.0429, 48.3965, 56.8974, 65.4186, 73.7662, 81.6966, 88.9522, 95.3083, 100.615, 104.813, 107.921, 110.013, 38.29, 46.0071, 53.7397, 61.3865, 68.805, 75.8258, 82.2711, 87.9844, 92.8529, 96.8155, 99.8643, 102.028, 35.9937, 43.176, 50.2818, 57.2337, 63.9254, 70.2353, 76.0376, 81.2191, 85.6913, 89.4, 92.3228, 94.4654, 33.187, 39.9262, 46.5212, 52.9138, 59.0256, 64.7678, 70.0483, 74.7823, 78.9005, 82.3546, 85.1174, 87.1791, 29.862, 36.2455, 42.4288, 48.3704, 54.0151, 59.2961, 64.1447, 68.4981, 72.2994, 75.5056, 78.0877, 80.026, 25.968, 32.0826, 37.9443, 43.528, 48.7949, 53.6988, 58.1883, 62.213, 65.727, 68.6926, 71.0776, 72.8562, 21.3831, 27.3301, 32.9611, 38.2731, 43.2425, 47.8394, 52.0265, 55.7642, 59.013, 61.7391, 63.9072, 65.4834]
        #self.alltheta2 = [98.3489, 102.907, 108.447, 114.913, 122.154, 129.913, 137.867, 145.697, 153.164, 160.143, 166.607, 172.598, 103.191, 107.318, 112.241, 117.903, 124.183, 130.905, 137.857, 144.826, 151.637, 158.176, 164.393, 170.288, 107.861, 111.595, 115.998, 121.015, 126.551, 132.478, 138.651, 144.919, 151.156, 157.261, 163.186, 168.905, 112.445, 115.821, 119.776, 124.259, 129.198, 134.494, 140.041, 145.734, 151.471, 157.176, 162.8, 168.317, 117.026, 120.068, 123.63, 127.662, 132.103, 136.883, 141.918, 147.126, 152.434, 157.78, 163.118, 168.428, 121.682, 124.41, 127.616, 131.254, 135.276, 139.62, 144.223, 149.022, 153.96, 158.988, 164.068, 169.181, 126.512, 128.929, 131.802, 135.088, 138.741, 142.709, 146.943, 151.393, 156.013, 160.764, 165.619, 170.561, 131.649, 133.741, 136.288, 139.242, 142.562, 146.198, 150.111, 154.257, 158.605, 163.122, 167.788, 172.598]
        
        # Center Well Lookup Map, testing with limit switch
        self.alltheta1 = [41.847, 50.9548, 60.4327, 70.1221, 79.7451, 88.9228, 97.2435, 104.363, 110.091, 114.403, 117.402, 119.248, 40.3897, 48.7287, 57.2355, 65.7761, 74.147, 82.0952, 89.3558, 95.7013, 100.987, 105.16, 108.25, 110.336, 38.3797, 46.0772, 53.8088, 61.4667, 68.8999, 75.9338, 82.3843, 88.0958, 92.9545, 96.9054, 99.9449, 102.11, 35.8858, 43.0509, 50.1569, 57.1199, 63.8277, 70.1519, 75.9641, 81.1486, 85.6175, 89.3198, 92.2376, 94.3817, 32.9391, 39.667, 46.267, 52.6743, 58.8056, 64.565, 69.8593, 74.6, 78.7194, 82.1695, 84.9268, 86.9873, 29.538, 35.9128, 42.1049, 48.0649, 53.7315, 59.0333, 63.8991, 68.2615, 72.0646, 75.2671, 77.8416, 79.7733, 25.6389, 31.7444, 37.6141, 43.2165, 48.5061, 53.432, 57.9383, 61.9724, 65.4878, 68.4474, 70.8214, 72.587, 21.1475, 27.0706, 32.7018, 38.0259, 43.015, 47.6303, 51.8318, 55.5757, 58.824, 61.5399, 63.6908, 65.2475]
        self.alltheta2 = [95.8505, 100.622, 106.368, 113.023, 120.424, 128.3, 136.311, 144.135, 151.538, 158.404, 164.724, 170.549, 100.434, 104.736, 109.826, 115.635, 122.039, 128.852, 135.854, 142.829, 149.607, 156.078, 162.196, 167.968, 104.857, 108.745, 113.293, 118.438, 124.083, 130.096, 136.325, 142.615, 148.842, 154.908, 160.765, 166.399, 109.2, 112.722, 116.812, 121.42, 126.465, 131.849, 137.463, 143.194, 148.943, 154.636, 160.222, 165.68, 113.543, 116.729, 120.426, 124.583, 129.138, 134.014, 139.128, 144.393, 149.735, 155.092, 160.419, 165.696, 117.954, 120.83, 124.178, 127.95, 132.093, 136.546, 141.244, 146.119, 151.112, 156.173, 161.268, 166.375, 122.519, 125.099, 128.126, 131.557, 135.346, 139.438, 143.784, 148.327, 153.025, 157.834, 162.726, 167.686, 127.349, 129.628, 132.348, 135.466, 138.938, 142.717, 146.759, 151.022, 155.468, 160.066, 164.795, 169.648]
        
        # P1 Well Lookup Map
        
        # P2 Well Lookup Map
        
        # P3 Well Lookup Map
        
        # P4 Well Lookup Map
        
        
        self.wellids = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8', 'a9', 'a10', 'a11', 'a12', 'b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7', 'b8', 'b9', 'b10', 'b11', 'b12', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'd1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7', 'd8', 'd9', 'd10', 'd11', 'd12', 'e1', 'e2', 'e3', 'e4', 'e5', 'e6', 'e7', 'e8', 'e9', 'e10', 'e11', 'e12', 'f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'f8', 'f9', 'f10', 'f11', 'f12', 'g1', 'g2', 'g3', 'g4', 'g5', 'g6', 'g7', 'g8', 'g9', 'g10', 'g11', 'g12', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'h7', 'h8', 'h9', 'h10', 'h11', 'h12']
        
        # Hardware attributes, Motor 1 is the top motor, Motor 2 is the bottom motor
        
        #Stepper Motor 2 Setup, Bottom Motor
        
        coils = (
            DigitalInOut(board.GP18),  # A1
            DigitalInOut(board.GP19),  # A2
            DigitalInOut(board.GP21),  # B1
            DigitalInOut(board.GP20),  # B2
        )
        for coil in coils:
            coil.direction = Direction.OUTPUT
         
        self.motor2 = stepper.StepperMotor(
            coils[0], coils[1], coils[2], coils[3], microsteps=None
        )
        # Stepper Motor 1 Setup, Top Motor

        coils = (
            DigitalInOut(board.GP6),  # A1
            DigitalInOut(board.GP7),  # A2
            DigitalInOut(board.GP8),  # B1
            DigitalInOut(board.GP9),  # B2
        )
        for coil in coils:
            coil.direction = Direction.OUTPUT

        self.motor1 = stepper.StepperMotor(
            coils[0], coils[1], coils[2], coils[3], microsteps=None
        )
        
        # Front Limit Switch, when activated, lsfront.value = false
        
        self.lsfront = DigitalInOut(board.GP2)
        self.lsfront.direction = Direction.INPUT
        self.lsfront.pull = Pull.UP
        
        # Rear Limit Switch, when activated, lsrear.value = false
        
        self.lsrear = DigitalInOut(board.GP3)
        self.lsrear.direction = Direction.INPUT
        self.lsrear.pull = Pull.UP
        
        # Pump 1
        self.pump1 = DigitalInOut(board.GP0)
        self.pump1.direction = Direction.OUTPUT
        self.pump1.value = False
        
        # Pump 2
        self.pump2 = DigitalInOut(board.GP1)
        self.pump2.direction = Direction.OUTPUT
        self.pump2.value = False
        
        # Pump 3
        self.pump3 = DigitalInOut(board.GP14)
        self.pump3.direction = Direction.OUTPUT
        self.pump3.value = False
        
        # Pump 4
        self.pump4 = DigitalInOut(board.GP15)
        self.pump4.direction = Direction.OUTPUT
        self.pump4.value = False
    
    # Sidekick Functions
    
    # Basic movement function. Moves the steppers to a new angular position, then updates current angular position.
    
    def advangleboth(self,newangle1,newangle2):
        #steps_one = int(abs(newangle1-self.current[0])/.45)
        #final_one = self.current[0] + (int((newangle1-self.current[0])/.45))*.45
        #steps_two = int(abs(newangle2-self.current[1])/.45)
        #final_two = self.current[1] + (int((newangle2-self.current[1])/.45))*.45
        
        steps_one = round(abs(newangle1-self.current[0])/.45)
        final_one = self.current[0] + (round((newangle1-self.current[0])/.45))*.45
        steps_two = round(abs(newangle2-self.current[1])/.45)
        final_two = self.current[1] + (round((newangle2-self.current[1])/.45))*.45
        
        if steps_one <= steps_two:
            for x in range(steps_one):
                if self.current[0] <= newangle1:
                    self.motor1.onestep(style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
                else:
                    self.motor1.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
                if self.current[1] <= newangle2:
                    self.motor2.onestep(style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
                else:
                    self.motor2.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
            for x in range(steps_two-steps_one):
                if self.current[1] <= newangle2:
                    self.motor2.onestep(style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.004)
                else:
                    self.motor2.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.004)
            self.current[0] = final_one
            self.current[1] = final_two
        
        if steps_two < steps_one:
            for x in range(steps_two):
                if self.current[0] <= newangle1:
                    self.motor1.onestep(style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
                else:
                    self.motor1.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
                if self.current[1] <= newangle2:
                    self.motor2.onestep(style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
                else:
                    self.motor2.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.002)
            for x in range(steps_one-steps_two):
                if self.current[0] <= newangle1:
                    self.motor1.onestep(style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.004)
                else:
                    self.motor1.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE) #style=stepper.INTERLEAVE
                    time.sleep(.004)
            
            self.current[0] = final_one
            self.current[1] = final_two
    
    # Plate Cycling, allows the user to cycle through the plating process. Good for spotting misalligned plates.
    
    def platecycle(self):
        
        platecycle = input("Would you like to cycle through the plating process? 'yes' or 'no'  ")

        if platecycle == "yes":
            for i in range(len(self.wellids)):
                thetas = kf.angle_lookup(self.wellids[i],self.wellids,self.alltheta1,self.alltheta2)
                self.advangleboth(thetas[0], thetas[1])
                print(self.wellids[i], "| ideal thetas:", thetas, "actual thetas:", self.current)
                time.sleep(.4)
                #self.pumpcycle()

            # Return to Starting Position
            self.advangleboth(self.home[0], self.home[1])
    
    # Pump 1 Dispense Cycle test
    
    def p1dispensecycle(self,desiredamount):
        
        for i in range(len(self.wellids)):
            thetas = kf.angle_lookup(self.wellids[i],self.wellids,self.alltheta1,self.alltheta2)
            self.movetothetas_p1(thetas)
            print(self.wellids[i], "| ideal thetas:", thetas, "actual thetas:", self.current)
            self.dispense1(desiredamount)

        # Return to Starting Position
        self.advangleboth(self.home[0], self.home[1])
        
    # Pump 2 Dispense Cycle test
    
    def p2dispensecycle(self,desiredamount):
        
        for i in range(len(self.wellids)):
            thetas = kf.angle_lookup(self.wellids[i],self.wellids,self.alltheta1,self.alltheta2)
            self.movetothetas_p2(thetas)
            print(self.wellids[i], "| ideal thetas:", thetas, "actual thetas:", self.current)
            self.dispense2(desiredamount)

        # Return to Starting Position
        self.advangleboth(self.home[0], self.home[1])
    
    # Pump 3 Dispense Cycle test
    
    def p3dispensecycle(self,desiredamount):
        
        for i in range(len(self.wellids)):
            thetas = kf.angle_lookup(self.wellids[i],self.wellids,self.alltheta1,self.alltheta2)
            self.movetothetas_p3(thetas)
            print(self.wellids[i], "| ideal thetas:", thetas, "actual thetas:", self.current)
            self.dispense3(desiredamount)

        # Return to Starting Position
        self.advangleboth(self.home[0], self.home[1])
    
    # Pump 4 Dispense Cycle test
    
    def p4dispensecycle(self,desiredamount):
        
        for i in range(len(self.wellids)):
            thetas = kf.angle_lookup(self.wellids[i],self.wellids,self.alltheta1,self.alltheta2)
            self.movetothetas_p4(thetas)
            print(self.wellids[i], "| ideal thetas:", thetas, "actual thetas:", self.current)
            self.dispense4(desiredamount)

        # Return to Starting Position
        self.advangleboth(self.home[0], self.home[1])
            
    # Pump Cycling, places each pump over the centerpoint
    
    def pumpcycle(self):
        
        print(self.current)
        
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,self.current[0],self.current[1])
        centerthetas = [self.current[0],self.current[1]]
        
        print(centerthetas)
        
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N1",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        time.sleep(.8)
        
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N2",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        time.sleep(.8)
        
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N3",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        time.sleep(.8)
        
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N4",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        time.sleep(.8)
        
        print(self.current)
        self.advangleboth(centerthetas[0], centerthetas[1])
        print(self.current)
    
    # Simple move to well function. Moves center of effector to target well.
    
    def movetowell(self, target_wellid):
        thetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
        self.advangleboth(thetas[0], thetas[1])
    
    
    ##%% PUMP MOVEMENT METHODS
    
    
    # Moves Pump 1 to target well
    
    def movetowell_p1(self, target_wellid):
        wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N1",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
    # Moves Pump 1 to target thetas
    
    def movetothetas_p1(self, targetthetas):
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N1",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
    
    # Moves Pump 2 to target well
    
    def movetowell_p2(self, target_wellid):
        wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N2",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
    # Moves Pump 2 to target thetas
    
    def movetothetas_p2(self, targetthetas):
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N2",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
     # Moves Pump 3 to target well
    
    def movetowell_p3(self, target_wellid):
        wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N3",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
    # Moves Pump 3 to target thetas
    
    def movetothetas_p3(self, targetthetas):
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N3",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
    # Moves Pump 4 to target well
    
    def movetowell_p4(self, target_wellid):
        wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N4",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
    # Moves Pump 4 to target thetas
    
    def movetothetas_p4(self, targetthetas):
        center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
        thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N4",center,self.origin)
        self.advangleboth(thetas[0], thetas[1])
        
    # Home function, returns effector to set home position
    
    def return_home(self):
        self.advangleboth(self.home[0], self.home[1])
        
    # Moves selected pump to purge location
    
    def movetopurge(self,pumpid):
        
        #purgecoord = kf.forward_kinematics(self.L1,self.L2,self.L3,self.purge[0],self.purge[1])
        
        if pumpid == "p1":
            self.movetothetas_p1(self.purge)
        if pumpid == "p2":
            self.movetothetas_p2(self.purge)
        if pumpid == "p3":
            self.movetothetas_p3(self.purge)
        if pumpid == "p4":
            self.movetothetas_p4(self.purge)
        
    ##%% DISPENSE FUNCTIONS
    
    # Dispenses the smallest amount of liquid from pump 1 (10 microliter aliquots)
    def dispense1(self,desiredamount):
        #desiredamount = float(input("enter the amount of liquid you want to dispense in microliters  "))
        actualamount = round(desiredamount/10)*10
        cycles = actualamount/10
        print("dispensing", actualamount)
        
        for i in range(cycles):
            self.pump1.value = True
            time.sleep(.1)
            print("energize")
            self.pump1.value = False
            time.sleep(.1)
            print("de-energize")
            print(i)
            
    # Dispenses the smallest amount of liquid from pump 2 (10 microliter aliquots)
    def dispense2(self,desiredamount):
        #desiredamount = float(input("enter the amount of liquid you want to dispense in microliters  "))
        actualamount = round(desiredamount/10)*10
        cycles = actualamount/10
        print("dispensing", actualamount)
        
        for i in range(cycles):
            self.pump2.value = True
            time.sleep(.1)
            print("energize")
            self.pump2.value = False
            time.sleep(.1)
            print("de-energize")
            print(i)
            
    # Dispenses the smallest amount of liquid from pump 3 (10 microliter aliquots)
    def dispense3(self,desiredamount):
        #desiredamount = float(input("enter the amount of liquid you want to dispense in microliters  "))
        actualamount = round(desiredamount/10)*10
        cycles = actualamount/10
        print("dispensing", actualamount)
        
        for i in range(cycles):
            self.pump3.value = True
            time.sleep(.1)
            print("energize")
            self.pump3.value = False
            time.sleep(.1)
            print("de-energize")
            print(i)
            
    # Dispenses the smallest amount of liquid from pump 4 (10 microliter aliquots)
    def dispense4(self,desiredamount):
        #desiredamount = float(input("enter the amount of liquid you want to dispense in microliters  "))
        actualamount = round(desiredamount/10)*10
        cycles = actualamount/10
        print("dispensing", actualamount)
        
        for i in range(cycles):
            self.pump4.value = True
            time.sleep(.1)
            print("energize")
            self.pump4.value = False
            time.sleep(.1)
            print("de-energize")
            print(i)
        
    #Dispense Cycle TEST
    def dispensecycle(self,desiredamount):
        
        for i in range(len(self.wellids)):
            thetas = kf.angle_lookup(self.wellids[i],self.wellids,self.alltheta1,self.alltheta2)
            self.advangleboth(thetas[0], thetas[1])
            print(self.wellids[i], "| ideal thetas:", thetas, "actual thetas:", self.current)
            time.sleep(.4)
            self.dispense1(desiredamount)

        # Return to Starting Position
        self.advangleboth(self.home[0], self.home[1])

    
    # Finds the endpoints, run first whenever waking the machine!
    
    def initialize(self):
        self.motor2.release()
        
        # Calibrates Motor 1, (Upper Motor)
        
        while self.lsfront.value != False:
            self.motor1.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE)
            time.sleep(.01)
        if self.lsfront.value == False:
            print("Front limit reached")
            self.current[0] = 0
        
        # Calibrates Motor 2, (Lower Motor)
        
        while self.lsrear.value != False:
            self.motor1.onestep(style=stepper.INTERLEAVE)
            self.current[0] = self.current[0] + .45
            self.motor2.onestep(direction=stepper.FORWARD, style=stepper.INTERLEAVE)
            time.sleep(.01)
        if self.lsrear.value == False:
                print("Rear limit reached")
                self.current[1] = 180
        
        self.return_home()
    
    # Releases both motors
    
    def release(self):
        self.motor1.release()
        self.motor2.release()
    
    # Allows the user to move the effector freely, displays current position
    
    def freemove(self):
        nearestwell = input("enter closest well  ")
        thetas =  kf.angle_lookup(nearestwell,self.wellids,self.alltheta1,self.alltheta2)
        self.advangleboth(thetas[0], thetas[1])
        while True:
            direction = input("enter direction with w,a,s,d, the hit enter. If finished, enter 'calibrate'  ")
            if direction == "s":
                newangles = kf.down(self.current[0],self.current[1])[0:2]
                self.advangleboth(newangles[0], newangles[1])
            if direction == "w":
                newangles = kf.up(self.current[0],self.current[1])[0:2]
                self.advangleboth(newangles[0], newangles[1])
            if direction == "a":
                newangles = kf.left(self.current[0],self.current[1])[0:2]
                self.advangleboth(newangles[0], newangles[1])
            if direction == "d":
                newangles = kf.right(self.current[0],self.current[1])[0:2]
                self.advangleboth(newangles[0], newangles[1])
            if direction == "calibrate":
                position = kf.forward_kinematics(self.L1,self.L2,self.L3,self.current[0],self.current[1])
                print("Calibration complete! Corrected angles are", self.current, "Corrected location is", position)
                break

    # Regenerate the plate map. Use if plating for the first time, or if using new plates.
    def remap(self):
        while True:
            calibrate = input("Calibrate? Type 'yes' to continue, or 'no' to stop  ")
            if calibrate == "no":
                break
            if calibrate == "yes":
                
                
                calcorners = ['a1','a12','h1','h12']
                calcorner_pos = []
                
                for x in range(4):
                    thetas =  kf.angle_lookup(calcorners[x],self.wellids,self.alltheta1,self.alltheta2)
                    if x == 0:
                        print("Center over the top left well")
                    if x == 1:
                        print("Center over the top right well")
                    if x == 2:
                        print("Center over the bottom left well")
                    if x == 3:
                        print("Center over the bottom right well")
                    
                    self.advangleboth(thetas[0], thetas[1])
                    
                    while True:
                        direction = input("enter direction with w,a,s,d, the hit enter. If finished, enter 'calibrate'  ")
                        if direction == "s":
                            newangles = kf.down(self.current[0],self.current[1])[0:2]
                            self.advangleboth(newangles[0], newangles[1])
                        if direction == "w":
                            newangles = kf.up(self.current[0],self.current[1])[0:2]
                            self.advangleboth(newangles[0], newangles[1])
                        if direction == "a":
                            newangles = kf.left(self.current[0],self.current[1])[0:2]
                            self.advangleboth(newangles[0], newangles[1])
                        if direction == "d":
                            newangles = kf.right(self.current[0],self.current[1])[0:2]
                            self.advangleboth(newangles[0], newangles[1])
                        if direction == "calibrate":
                            position = kf.forward_kinematics(self.L1,self.L2,self.L3,self.current[0],self.current[1])
                            print("Calibration complete! Corrected angles are", self.current, "Corrected location is", position)
                            calcorner_pos.append(position)
                            break
                results = plategen.remap_plate(calcorner_pos[0],calcorner_pos[1],calcorner_pos[2],calcorner_pos[3],self.L1,self.L2,self.L3,self.origin)
                print(results[0])
                print(results[1])
                print(results[2])
    
    # Reads a CSV file and turns it into commands.
    
    def read_instructions(self, filename):
        test = open(filename)
        stuff = test.read()
        stuff = stuff.replace("\r", "").split("\n")
        stuff = stuff[1:len(stuff)]

        cmd = []

        for i in range(len(stuff)):
            cmd.append(stuff[i].split(","))
        
        cmd = cmd[0:len(cmd)-1]

        return cmd
    
    # Takes a CSV file of instructions, and plates them.
    
    def execute_protocol(self, filename):
        commands = self.read_instructions(filename)
        #print(commands)
        for i in range(len(commands)):
            targetwell = commands[i][0]
            pumpid = commands[i][1]
            desiredamount = float(commands[i][2])
            #print(targetwell)
            #print(pumpid)
            #print(desiredamount)
            
            if pumpid == "p1":
                self.movetowell_p1(targetwell)
                self.dispense1(desiredamount)
            if pumpid == "p2":
                self.movetowell_p2(targetwell)
                self.dispense2(desiredamount)
            if pumpid == "p3":
                self.movetowell_p3(targetwell)
                self.dispense3(desiredamount)
            if pumpid == "p4":
                self.movetowell_p4(targetwell)
                self.dispense4(desiredamount)

                
alpha = Sidekick(7,3,10,0.5,[0,0],[90,178],[119.484,171.701])
alpha.initialize()
print(alpha.current)

alpha.movetopurge("p1")
alpha.dispense1(1000)
alpha.movetopurge("p2")
alpha.dispense2(1000)
alpha.movetopurge("p3")
alpha.dispense3(1000)
alpha.movetopurge("p4")
alpha.dispense4(1000)

alpha.execute_protocol("flag_demo_cheat.csv")

alpha.return_home()


print(alpha.current)
