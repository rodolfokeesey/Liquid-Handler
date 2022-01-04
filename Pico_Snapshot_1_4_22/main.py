from Sidekick_object_v3_4_micropython import Sidekick
from parser import com_parser


# Defines an instance of the Sidekick
alpha = Sidekick(7,3,10,0.5,[0,0],[90,178],[119.484,171.701])

# On startup, homes position
alpha.initialize()
#alpha.release()


# Main loop, awaiting command


while True:
    # Get User Input
    command = input("awaiting command\n")
    command = command.lower()
    #alpha.record_command(command)
    
    #### Calibration/Troubleshooting functions
    # Initializing/Homing: Input Initialize
    if command == ("initialize"):
        alpha.initialize()
    # Hardware Check: Input Hardware check
    elif command == ("hardware check"):
        alpha.hardwarecheck()
    # Free Move: Input Free Move -> Function walks you through it
    elif command == ("free move"):
        alpha.freemove()
    # Sleep
    elif command == ("sleep"):
        alpha.release()
    elif command == ("wake"):
        alpha.wake()
    elif command == ("return home"):
        alpha.return_home()
    elif command == ("manual purge"):
        alpha.manualpurge()
    elif command == ("remap"):
        alpha.remap()
    elif command == ("remap v2"):
        alpha.remap_v2()
    elif command == ("set purge"):
        alpha.purgeset()
    #### Parses basic functions from instructions
    else:
        parsed = com_parser(command)
        pumpid = parsed[0]
        action = parsed[1]
        volume = parsed[2]
        wellid = parsed[3]

            
            
        if pumpid != []:
            if action != []:
                if action == "dispense":
                    if volume != []:
                        if wellid != []:
                            if wellid != "purge":
                                alpha.movetowell(pumpid,wellid)
                                alpha.dispense(pumpid,volume)
                            if wellid == "purge":
                                alpha.movetopurge(pumpid)
                                alpha.dispense(pumpid,volume)
                        if wellid == []:
                            print("No target well indicated. \n Ex. well a8")
                    if volume == []:
                        print("No volume indicated. \n Ex. 200 microliters")
                if action == "move":
                    if wellid != []:
                        if wellid != "purge":
                            alpha.movetowell(pumpid,wellid)
                        else:
                            alpha.movetopurge(pumpid)
                    if wellid == []:
                        print("No target well indicated. \n Ex. 'well a8' or 'purge'")
            if action == []:
                print('No action found. Please type "dispense" or "move" to indicate \n what action you would like the pump to take.')
        if pumpid == []:
            print("Please indicate which pump you want to move. \n Ex. pump 4")

    
    
    # Command List
    
    # Basic Functions
    # Dispense: Input pump, Input Amount, Input Target
    # Move to well: Input pump, Input Target
    # Move to angle: Input pump, Input Target
    
    # First, indicate pump, then give the rest of the commands.
    # So a line would look like, from pump 1, dispense 100 microliters into well a1
    
    #First parse: Assume one line is one command
    # Sub parse: Cut line into words
    # Sub parse: Identify the pump. Look for pump string and an ID after it.
    # Sub parse: Determine the command, Look for move or dispense
    # Sub parse: First look for dispense.
    # Sub parse: If dispense, look for amount, then target.
    # Sub parse: If move, look for target after it.
        # If target is well, use move to well
        # If target is
    # Sub parse: If missing an input, or referencing an impossible value, say command not understood, report issue
    
    
    
    # Setting up Well/Purge
    # Plate Remapping: Input Plate Remapping -> Function walks you through it
    # Purge Remapping: Input Purge Remapping -> Function walks you through it
    
    
    # Parse String based on spaces/new lines
    
    # Get Commands In String
    # Execute Commands In Order
    # Return to main loop to recieve more commands
