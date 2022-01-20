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
        self.home = [90,178]  # there is a `Home` parameter not used in init
        self.origin = Origin
        self.current = Effector_Location #Angular location of stepper
        self.purge = self.loadpurge() #[45.7808, 89.6624]
        self.stepsize = .1125 # 8 microsteps at 0.9 degree motor, 
        self.stepdelay = .0010
        

        # Center Well Lookup Map, saved as platemap1.txt
        
        self.plateinfo = self.loadplate()
        self.alltheta1 = self.plateinfo[0]
        self.alltheta2 = self.plateinfo[1]
        self.wellids = self.plateinfo[2]
        
        
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
    
    # Plate loading function
    
    @staticmethod
    def loadplate():
        """Take the save text file of well coordinates and read it into memory."""
        
        platefile = open("platemap1.txt","r")
        emptyplate = platefile.read()
        emptyplate = emptyplate.split("\n")
        platelength = int((len(emptyplate) - 4)/3)
        sectionlength = int((len(emptyplate) - 1)/3)
        theta_one = emptyplate[1:sectionlength]
        theta_one = [float(coords) for coords in theta_one]
        theta_two = emptyplate[sectionlength+1:sectionlength*2]
        theta_two = [float(coords) for coords in theta_two]
        well_ids = emptyplate[(sectionlength*2)+1:len(emptyplate)-1]
        platefile.close()
        return[theta_one,theta_two,well_ids]
    
    # Purge location loading
    
    @staticmethod
    def loadpurge():
        """Take the save text file of purge coordinates and read it into memory."""
        
        purgefile = open("purge1.txt","r")
        purgeloc = purgefile.read()
        purgeloc = purgeloc.split("\n")
        purgeloc = [float(coords) for coords in purgeloc]
        purgefile.close()
        return purgeloc
        
    
    # One step command for the steppers.
    
    def motor1_onestep(self,direction):
        """Command stepper motor 1 to move one step, False is CCW, True is CW"""
        
        self.motor1_d.value(direction) # False is CCW, True is CW
        self.motor1.value(0)
        time.sleep(self.stepdelay)
        self.motor1.value(1)
        time.sleep(self.stepdelay)
        
    def motor2_onestep(self,direction):
        """Command stepper motor 1 to move one step, False is CCW, True is CW"""
        
        self.motor2_d.value(direction) # False is CCW, True is CW
        self.motor2.value(0)
        time.sleep(self.stepdelay)
        self.motor2.value(1)
        time.sleep(self.stepdelay)
    
    # Basic movement function. Moves the steppers to a new angular position, then updates current angular position.
    
    #step size at 32 microsteps, 0.9 degree stepper is .028125
    
    def advangleboth(self,newangle1,newangle2):
        """Move the steppers to a new angular position, update current angular position."""

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
        """Allow the user to cycle through the plating process."""
        
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
    
    # Pump Cycling, places each pump over the centerpoint, mostly for troubleshooting/reliability testing
    
    def pumpcycle(self):
        """Places each pump over the armature centerpoint."""
        
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
    

    ##%% PUMP MOVEMENT METHODS
    
    # Simple move to well function. Moves indicated effector to target well.
    
    def movetowell(self, effector, target_wellid):
        """Move indicated effector to target well."""
        
        validwell = [x for x in self.wellids if x == target_wellid]
        if not validwell:
            print("The target well is not in the current plate layout")
            return
        
        if effector == "center":
            thetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p1":
            wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N1",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p2":
            wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N2",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p3":
            wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N3",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p4":
            wellthetas = kf.angle_lookup(target_wellid,self.wellids,self.alltheta1,self.alltheta2)
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,wellthetas[0],wellthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N4",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        else:
            print("Indicated pump not recognized")
    
    # Simple move to theta function. Moves indicated effector to target well.
    
    def movetothetas(self, effector, targetthetas):
        """Move indicated effector to target well."""
        
        if effector == "center":
            self.advangleboth(targetthetas[0], targetthetas[1])
        elif effector == "p1":
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N1",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p2":
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N2",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p3":
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N3",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        elif effector == "p4":
            center = kf.forward_kinematics(self.L1,self.L2,self.L3,targetthetas[0],targetthetas[1])
            thetas = kf.inverse_kinematics_multi(self.L1,self.L2,self.L3,self.Ln,"N4",center,self.origin)
            self.advangleboth(thetas[0], thetas[1])
        else:
            print("Indicated pump not recognized")
            
        
    # Home function, returns effector to set home position
    
    def return_home(self):
        """Return effector to set home position"""
        
        self.advangleboth(self.home[0], self.home[1])
        
    # Moves selected pump to purge location
    
    def movetopurge(self,pumpid):
        """Move selected pump to purge location"""
        
        self.movetothetas(pumpid,self.purge)

    # Allows the user to select a pump, and purge the lines by holding down the purge button.
    def manualpurge(self):
        """Allow the user to select a pump, and purge the lines by holding down the purge button."""
        
        while True:
            ool = 1
            pumpid = input("Which pump would you like to purge? \n Type 'p1', 'p2', 'p3', or 'p4' and hit enter.   ")
            
            if pumpid == "p1" or pumpid == "p2" or pumpid == "p3" or pumpid == "p4":
                self.movetothetas(pumpid,self.purge)
                timer = 0
                outerloop = 1
                while outerloop == 1:
                    outerloop = 1
                    if self.purgebutton.value() == 0:
                        timer = 0
                        self.dispense(pumpid,10)
                        print(self.purgebutton.value())
                        time.sleep(.01)
                    if self.purgebutton.value() == 1:
                        time.sleep(.1)
                        timer=timer+1
                    if self.purgebutton.value() == 1 and timer >= 40:
                        stop = input("Type stop if you want stop. Type anything else if you'd like to continue purging this pump.  ")
                        if stop == "stop":
                            timer = 0
                            outerloop = 0
                        else:
                            timer = 0
            cont = input("Would you like to home another pump? \n Type 'yes' or 'no'   ")
            if cont == "yes":
                ool = 1
            if cont != "yes":
                break
        
        
    
    ##%% DISPENSE FUNCTIONS
    
    # Dispenses the commanded amount of liquid from the indicated pump (10 microliter aliquots)
    def dispense(self,pump,desiredamount):
        """Dispense the commanded amount of liquid from the indicated pump (10 microliter aliquots)."""
        
        actualamount = round(desiredamount/10)*10
        cycles = round(actualamount/10)
        if cycles != 0:
            print("dispensing", actualamount)
        
        if pump == "p1":
            for i in range(cycles):
                self.pump1.value(1)
                time.sleep(.1)
                #print("energize")
                self.pump1.value(0)
                time.sleep(.1)
                #print("de-energize")
                #print(i)
        elif pump == "p2":
            for i in range(cycles):
                self.pump2.value(1)
                time.sleep(.1)
                #print("energize")
                self.pump2.value(0)
                time.sleep(.1)
                #print("de-energize")
                #print(i)
        elif pump == "p3":
            for i in range(cycles):
                self.pump3.value(1)
                time.sleep(.1)
                #print("energize")
                self.pump3.value(0)
                time.sleep(.1)
                #print("de-energize")
                #print(i)
        elif pump == "p4":
            for i in range(cycles):
                self.pump4.value(1)
                time.sleep(.1)
                #print("energize")
                self.pump4.value(0)
                time.sleep(.1)
                #print("de-energize")
                #print(i)
        else:
            print("Indicated pump is not recognized")

    # Finds the endpoints, run first whenever waking the machine!
    
    def initialize(self):
        """Find the endpoints of armature travel."""
        
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
        
    # Hardware Check, checks if everything is working properly, used to validate wiring during build.
    
    def hardwarecheck(self):
        """Hardware Check, check if everything is working properly, use to validate wiring during build."""
    
        self.motor2_sleep.value(1)
        # Calibrates Motor 1, (Upper Motor)
        print("Testing top motor. Armature movement should be clockwise, moving to front of Sidekick")
        while self.lsfront.value() != 0:
            self.motor1_onestep(1)
            time.sleep(self.stepdelay*3)
        if self.lsfront.value() == 0:
            print("Front limit switch triggered")
            self.current[0] = 0
        
        # Calibrates Motor 2, (Lower Motor)
        self.motor2_sleep.value(0)
        
        print("Testing lower motor. Armature movement should be counterclockwise, moving to rear of Sidekick")
        while self.lsrear.value() != 0:
            self.motor1_onestep(0)
            self.current[0] = self.current[0] + self.stepsize
            self.motor2_onestep(0)
            time.sleep(self.stepdelay*3)
        if self.lsrear.value() == 0:
                print("Rear limit switch triggered")
                self.current[1] = 180
        
        self.return_home()
        
        print("testing pump 1 (closest to front). You should hear clicking as the pump energizes.")
        
        self.dispense("p1", 100)
        print("testing pump 2.")
        self.dispense("p2", 100)
        print("testing pump 3.")
        self.dispense("p3", 100)
        print("testing pump 4.")
        self.dispense("p4", 100)
        
        print("testing purge button. Click the purge button, then release it. \nThe Sidekick will output whether it detects that the button is held or released.")
        
        time.sleep(5)
        
        ot = 1
        btest = 1
        while ot == 1:
            if self.purgebutton.value() == 0:
                while btest == 1:
                    if self.purgebutton.value() == 0:
                        print("purge button pressed")
                        time.sleep(.1)
                    if self.purgebutton.value() == 1:
                        print("purge button released")
                        btest =0
                        ot = 0
                     

        print("Hardware check complete. If any components did not behave as described, please troubleshoot wiring")
        
    
    # Releases both motors
    
    def release(self):
        """Release both motors"""

        self.motor1_sleep.value(1)
        self.motor2_sleep.value(1)
    
    # Wakes both motors
    
    def wake(self):
        """Wake both motors"""
        
        self.motor1_sleep.value(0)
        self.motor2_sleep.value(0)
        self.initialize()

    
    # Allows the user to move the effector freely, then prints position.
    
    def freemove(self):
        """Allow the user to move the effector freely, then print position."""
        
        nearestwell = input("enter closest well  ")
        thetas =  kf.angle_lookup(nearestwell,self.wellids,self.alltheta1,self.alltheta2)
        self.advangleboth(thetas[0], thetas[1])
        while True:
            direction = input("enter direction with w,a,s,d, the hit enter. If finished, enter 'finished'  ")
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
            if direction == "finished":
                position = kf.forward_kinematics(self.L1,self.L2,self.L3,self.current[0],self.current[1])
                print("Freemove ended. Current angles are", self.current, "Current location is", position)
                break
            
    def purgeset(self):
        """Set the location of the purge vessel"""

        nearestwell = input("enter closest well to purge location  ")
        thetas =  kf.angle_lookup(nearestwell,self.wellids,self.alltheta1,self.alltheta2)
        self.advangleboth(thetas[0], thetas[1])
        while True:
            direction = input("enter direction with w,a,s,d, the hit enter.\n If centered over the purge location, enter 'finished'  ")
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
            if direction == "finished":
                position = kf.forward_kinematics(self.L1,self.L2,self.L3,self.current[0],self.current[1])
                print("Freemove ended. Current angles are", self.current, "Current location is", position)
                break
        file = open("purge1.txt", "w")
        file.write(str(self.current[0]) + "\n")
        file.write(str(self.current[1]))
        file.close()
        self.return_home()

    # Regenerates the plate map. Use if plating for the first time, or if using new plates.
    def remap(self):
        """Regenerate the plate map. Use if plating for the first time, or if using new plates."""
        
        
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
                file = open("platemap1.txt", "w")
                file.write("theta_one \n")
                for coord in results[0]:
                    file.write(str(coord) + "\n")
                file.write("theta_two \n")
                for coord in results[1]:
                    file.write(str(coord) + "\n")
                file.write("well_ids \n")
                for coord in results[2]:
                    file.write(coord + "\n")
                file.close()
                self.plateinfo = self.loadplate()
                self.alltheta1 = self.plateinfo[0]
                self.alltheta2 = self.plateinfo[1]
                self.wellids = self.plateinfo[2]
                self.return_home()
    
    # Reads a CSV file and turns it into commands.
    
    @staticmethod
    def read_instructions(filename):
        """Read a CSV file and turn it into commands."""
        
        test = open(filename)
        stuff = test.read()
        stuff = stuff.replace("\r", "").split("\n")
        stuff = stuff[1:]

        cmd = [stuff[i].split(",") for i in range(len(stuff) - 1)]

        #print(cmd)

        return cmd
    
    # Takes a CSV file of instructions, and plates them.
    
    def execute_protocol(self):
        """Take a CSV file of instructions, and plate them."""

        commands = self.read_instructions("saved_protocol.csv")
        print(commands)
        for i in range(len(commands)):
            pumpid = commands[i][0]
            targetwell = commands[i][1]
            desiredamount = float(commands[i][2])
            #print(targetwell)
            #print(pumpid)
            #print(desiredamount)
            
            self.movetowell(pumpid,targetwell)
            self.dispense(pumpid,desiredamount)


                
#alpha = Sidekick(7,3,10,0.5,[0,0],[90,178],[119.484,171.701]) # This is the template of the default Sidekick.

