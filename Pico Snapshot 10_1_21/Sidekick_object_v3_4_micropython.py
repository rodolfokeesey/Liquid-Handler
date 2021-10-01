from machine import Pin
import time
import kinematicsfunctions as kf
import plategen

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
        self.purge = [45.7808, 89.6624]
        self.stepsize = .1125 # 8 microsteps at 0.9 degree motor, 
        self.stepdelay = .0015
        
        # Well Lookup Map, the default 96 well map is loaded here, but should be recalibrated with the remap method
        #self.wellids = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8', 'a9', 'a10', 'a11', 'a12', 'b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7', 'b8', 'b9', 'b10', 'b11', 'b12', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'd1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7', 'd8', 'd9', 'd10', 'd11', 'd12', 'e1', 'e2', 'e3', 'e4', 'e5', 'e6', 'e7', 'e8', 'e9', 'e10', 'e11', 'e12', 'f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'f8', 'f9', 'f10', 'f11', 'f12', 'g1', 'g2', 'g3', 'g4', 'g5', 'g6', 'g7', 'g8', 'g9', 'g10', 'g11', 'g12', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'h7', 'h8', 'h9', 'h10', 'h11', 'h12']
        #self.alltheta1 = [41.1831, 50.2859, 59.7319, 69.3711, 78.9405, 88.0794, 96.3887, 103.529, 109.302, 113.668, 116.711, 118.583, 40.0429, 48.3965, 56.8974, 65.4186, 73.7662, 81.6966, 88.9522, 95.3083, 100.615, 104.813, 107.921, 110.013, 38.29, 46.0071, 53.7397, 61.3865, 68.805, 75.8258, 82.2711, 87.9844, 92.8529, 96.8155, 99.8643, 102.028, 35.9937, 43.176, 50.2818, 57.2337, 63.9254, 70.2353, 76.0376, 81.2191, 85.6913, 89.4, 92.3228, 94.4654, 33.187, 39.9262, 46.5212, 52.9138, 59.0256, 64.7678, 70.0483, 74.7823, 78.9005, 82.3546, 85.1174, 87.1791, 29.862, 36.2455, 42.4288, 48.3704, 54.0151, 59.2961, 64.1447, 68.4981, 72.2994, 75.5056, 78.0877, 80.026, 25.968, 32.0826, 37.9443, 43.528, 48.7949, 53.6988, 58.1883, 62.213, 65.727, 68.6926, 71.0776, 72.8562, 21.3831, 27.3301, 32.9611, 38.2731, 43.2425, 47.8394, 52.0265, 55.7642, 59.013, 61.7391, 63.9072, 65.4834]
        #self.alltheta2 = [98.3489, 102.907, 108.447, 114.913, 122.154, 129.913, 137.867, 145.697, 153.164, 160.143, 166.607, 172.598, 103.191, 107.318, 112.241, 117.903, 124.183, 130.905, 137.857, 144.826, 151.637, 158.176, 164.393, 170.288, 107.861, 111.595, 115.998, 121.015, 126.551, 132.478, 138.651, 144.919, 151.156, 157.261, 163.186, 168.905, 112.445, 115.821, 119.776, 124.259, 129.198, 134.494, 140.041, 145.734, 151.471, 157.176, 162.8, 168.317, 117.026, 120.068, 123.63, 127.662, 132.103, 136.883, 141.918, 147.126, 152.434, 157.78, 163.118, 168.428, 121.682, 124.41, 127.616, 131.254, 135.276, 139.62, 144.223, 149.022, 153.96, 158.988, 164.068, 169.181, 126.512, 128.929, 131.802, 135.088, 138.741, 142.709, 146.943, 151.393, 156.013, 160.764, 165.619, 170.561, 131.649, 133.741, 136.288, 139.242, 142.562, 146.198, 150.111, 154.257, 158.605, 163.122, 167.788, 172.598]
        
        # Center Well Lookup Map, testing with limit switch
        self.alltheta1 = [40.0435, 48.0684, 56.2205, 64.3838, 72.3843, 80.0053, 87.0193, 93.2226, 98.4754, 102.713, 105.942, 108.219, 38.0725, 45.4792, 52.8934, 60.2208, 67.3341, 74.0815, 80.3059, 85.8678, 90.6614, 94.628, 97.7512, 100.05, 35.5539, 42.4508, 49.2695, 55.9385, 62.3627, 68.4331, 74.038, 79.0764, 83.4663, 87.1561, 90.1212, 92.3629, 32.5215, 39.0016, 45.3409, 51.4851, 57.364, 62.8964, 68.0029, 72.6051, 76.6399, 80.0634, 82.8464, 84.9793, 28.9626, 35.1147, 41.0723, 46.798, 52.24, 57.3397, 62.0361, 66.27, 69.9916, 73.1599, 75.7481, 77.7352, 24.8176, 30.7296, 36.3955, 41.7919, 46.8866, 51.6348, 55.9905, 59.9091, 63.3482, 66.2726, 68.6521, 70.4613, 19.9475, 25.7227, 31.1896, 36.3439, 41.168, 45.6326, 49.7044, 53.3476, 56.5269, 59.2087, 61.3612, 62.9506, 14.0569, 19.8465, 25.2336, 30.2431, 34.8767, 39.1227, 42.9588, 46.3573, 49.2869, 51.7129, 53.5981, 54.8941]
        self.alltheta2 = [102.037, 106.117, 110.924, 116.4, 122.439, 128.88, 135.539, 142.225, 148.782, 155.098, 161.119, 166.836, 106.736, 110.42, 114.715, 119.566, 124.892, 130.577, 136.491, 142.504, 148.495, 154.38, 160.101, 165.633, 111.355, 114.681, 118.538, 122.877, 127.633, 132.721, 138.044, 143.507, 149.02, 154.514, 159.938, 165.268, 115.977, 118.974, 122.448, 126.357, 130.645, 135.243, 140.084, 145.091, 150.198, 155.347, 160.498, 165.625, 120.691, 123.376, 126.505, 130.04, 133.929, 138.12, 142.557, 147.181, 151.942, 156.793, 161.7, 166.644, 125.599, 127.979, 130.787, 133.982, 137.522, 141.362, 145.454, 149.755, 154.219, 158.816, 163.515, 168.303, 130.849, 132.906, 135.394, 138.271, 141.495, 145.023, 148.817, 152.839, 157.054, 161.438, 165.972, 170.648, 136.686, 138.361, 140.501, 143.056, 145.975, 149.214, 152.742, 156.524, 160.535, 164.758, 169.181, 173.811]
        
        # P1 Well Lookup Map
        
        # P2 Well Lookup Map
        
        # P3 Well Lookup Map
        
        # P4 Well Lookup Map
        
        
        self.wellids = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6', 'a7', 'a8', 'a9', 'a10', 'a11', 'a12', 'b1', 'b2', 'b3', 'b4', 'b5', 'b6', 'b7', 'b8', 'b9', 'b10', 'b11', 'b12', 'c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7', 'c8', 'c9', 'c10', 'c11', 'c12', 'd1', 'd2', 'd3', 'd4', 'd5', 'd6', 'd7', 'd8', 'd9', 'd10', 'd11', 'd12', 'e1', 'e2', 'e3', 'e4', 'e5', 'e6', 'e7', 'e8', 'e9', 'e10', 'e11', 'e12', 'f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7', 'f8', 'f9', 'f10', 'f11', 'f12', 'g1', 'g2', 'g3', 'g4', 'g5', 'g6', 'g7', 'g8', 'g9', 'g10', 'g11', 'g12', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'h7', 'h8', 'h9', 'h10', 'h11', 'h12']
        
        ########## Hardware attributes, Motor 1 is the top motor, Motor 2 is the bottom motor
        
        ##### Stepper Motor 2 Setup, Bottom Motor
        
        # 1.8 degree steppers, order from top: red,blue,green,black
        
        # Step Pin
        self.motor2 = Pin(10, Pin.OUT)
        self.motor2.value(0)

        # Direction Pin

        self.motor2_d = Pin(9, Pin.OUT)
        self.motor2_d.value(0)

        # Mode Pins

        self.motor2_m0 = Pin(15, Pin.OUT)
        self.motor2_m1 = Pin(14, Pin.OUT)

        # Sleep Pin

        self.motor2_sleep = Pin(16, Pin.OUT)
        self.motor2_sleep.value(0)

        # Setting Mode to 1/8 step, so 0.45 degrees per step on a 1.8 degree stepper

        self.motor2_m0.value(0)
        self.motor2_m1.value(0)
        
        
        ##### Stepper Motor 1 Setup, Top Motor
        # 1.8 degree steppers, order from top: blue,red,green,black

        # Step Pin
        self.motor1 = Pin(1, Pin.OUT)
        self.motor1.value(0)

        # Direction Pin

        self.motor1_d = Pin(0, Pin.OUT)
        self.motor1_d.value(0)

        # Mode Pins

        self.motor1_m0 = Pin(6, Pin.OUT)

        self.motor1_m1 = Pin(5, Pin.OUT)

        # Sleep Pin

        self.motor1_sleep = Pin(7, Pin.OUT)
        self.motor1_sleep.value(0)

        # Setting Mode to 1/8 step, so 0.45 degrees per step

        self.motor1_m0.value(0)
        self.motor1_m1.value(0)
        
        ##### Limit Switch Setup #####
        
        # Front Limit Switch, when activated, lsfront.value = false
        
        self.lsfront = Pin(18, Pin.IN, Pin.PULL_UP)
     
        # Rear Limit Switch, when activated, lsrear.value = false
        
        self.lsrear = Pin(19, Pin.IN, Pin.PULL_UP)
        
        ##### Purge Button #####
        
        self.purgebutton = Pin(20, Pin.IN, Pin.PULL_UP)

        
        ##### Pump Setup #####
        
        # Pump 1
        self.pump1 = Pin(27, Pin.OUT)
        self.pump1.value(0)
        
        # Pump 2
        self.pump2 = Pin(26, Pin.OUT)
        self.pump2.value(0)
        
        # Pump 3
        self.pump3 = Pin(22, Pin.OUT)
        self.pump3.value(0)
        
        # Pump 4
        self.pump4 = Pin(21, Pin.OUT)
        self.pump4.value(0)
    
    # Sidekick Functions
    
    # One step command for the steppers.
    
    def motor1_onestep(self,direction):
        self.motor1_d.value(direction) # False is CCW, True is CW
        self.motor1.value(0)
        time.sleep(self.stepdelay)
        self.motor1.value(1)
        time.sleep(self.stepdelay)
        
    def motor2_onestep(self,direction): # 
        self.motor2_d.value(direction) # False is CCW, True is CW
        self.motor2.value(0)
        time.sleep(self.stepdelay)
        self.motor2.value(1)
        time.sleep(self.stepdelay)
    
    # Basic movement function. Moves the steppers to a new angular position, then updates current angular position.
    
    #step size at 32 microsteps, 0.9 degree stepper is .028125
    
    def advangleboth(self,newangle1,newangle2):
        
        steps_one = round(abs(newangle1-self.current[0])/self.stepsize)
        final_one = self.current[0] + (round((newangle1-self.current[0])/self.stepsize))*self.stepsize
        steps_two = round(abs(newangle2-self.current[1])/self.stepsize)
        final_two = self.current[1] + (round((newangle2-self.current[1])/self.stepsize))*self.stepsize
        
        if steps_one <= steps_two:
            for x in range(steps_one):
                if self.current[0] <= newangle1:
                    self.motor1_onestep(0) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
                else:
                    self.motor1_onestep(1) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
                if self.current[1] <= newangle2:
                    self.motor2_onestep(0) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
                else:
                    self.motor2_onestep(1) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
            for x in range(steps_two-steps_one):
                if self.current[1] <= newangle2:
                    self.motor2_onestep(0) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay*2)
                else:
                    self.motor2_onestep(1) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay*2)
            self.current[0] = final_one
            self.current[1] = final_two
        
        if steps_two < steps_one:
            for x in range(steps_two):
                if self.current[0] <= newangle1:
                    self.motor1_onestep(0) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
                else:
                    self.motor1_onestep(1) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
                if self.current[1] <= newangle2:
                    self.motor2_onestep(0) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
                else:
                    self.motor2_onestep(1) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay)
            for x in range(steps_one-steps_two):
                if self.current[0] <= newangle1:
                    self.motor1_onestep(0) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay*2)
                else:
                    self.motor1_onestep(1) #style=stepper.INTERLEAVE
                    time.sleep(self.stepdelay*2)
            
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
    
    def manualpurge(self,pumpid):
        
        if pumpid == "p1":
            self.movetothetas_p1(self.purge)
            while True:
                if self.purgebutton.value() == 0:
                    self.dispense1(10)
                    print(self.purgebutton.value())
                    time.sleep(.01)
                    if self.purgebutton.value() == 1:
                        break
        if pumpid == "p2":
            self.movetothetas_p2(self.purge)
            while True:
                if self.purgebutton.value() == 0:
                    self.dispense2(10)
                    print(self.purgebutton.value())
                    time.sleep(.01)
                    if self.purgebutton.value() == 1:
                        break
        if pumpid == "p3":
            self.movetothetas_p3(self.purge)
            while True:
                if self.purgebutton.value() == 0:
                    self.dispense3(10)
                    print(self.purgebutton.value())
                    time.sleep(.01)
                    if self.purgebutton.value() == 1:
                        break
        if pumpid == "p4":
            self.movetothetas_p4(self.purge)
            while True:
                if self.purgebutton.value() == 0:
                    self.dispense4(10)
                    print(self.purgebutton.value())
                    time.sleep(.01)
                    if self.purgebutton.value() == 1:
                        break
    
        
    
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
        self.motor2_sleep.value(1)
        # Calibrates Motor 1, (Upper Motor)
        
        while self.lsfront.value() != 0:
            self.motor1_onestep(1)
            time.sleep(self.stepdelay*3)
        if self.lsfront.value() == 0:
            print("Front limit reached")
            self.current[0] = 0
        
        # Calibrates Motor 2, (Lower Motor)
        self.motor2_sleep.value(0)
        
        while self.lsrear.value() != 0:
            self.motor1_onestep(0)
            self.current[0] = self.current[0] + self.stepsize
            self.motor2_onestep(0)
            time.sleep(self.stepdelay*3)
        if self.lsrear.value() == 0:
                print("Rear limit reached")
                self.current[1] = 180
        
        self.return_home()
    
    # Releases both motors
    
    def release(self):
        self.motor1_sleep.value(1)
        self.motor2_sleep.value(1)
    
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


#for x in range(10):
#    alpha.motor2_onestep(True)

#for x in range(10):
#    alpha.motor1_onestep(True)

alpha.initialize()
#print(alpha.current)

#alpha.movetopurge("p1")
#alpha.dispense1(100)
#alpha.movetopurge("p2")
#alpha.dispense2(100)
#alpha.movetopurge("p3")
#alpha.dispense3(100)
#alpha.movetopurge("p4")
#alpha.dispense4(100)

#alpha.execute_protocol("flag_demo_cheat.csv")

#alpha.return_home()


#print(alpha.current)
