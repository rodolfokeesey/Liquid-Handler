from Sidekick_object import Sidekick
from parser import com_parser, gcode_parser


# Defines an instance of the Sidekick
alpha = Sidekick(7,3,10,0.5,[0,0],[90,178],[119.484,171.701])

# On startup, homes position
alpha.initialize()


# Dictionary of available commands
comdict = {
        "initialize" : alpha.initialize,
        "hardware check" : alpha.hardwarecheck,
        "xy position" : alpha.print_current_xy,
        "angular position" : alpha.print_angular_position,
        "free move" : alpha.freemove,
        "sleep" : alpha.release,
        "wake" : alpha.wake,
        "return home" : alpha.return_home,
        "manual purge" : alpha.manualpurge,
        "remap" : alpha.remap,
        "set purge" : alpha.purgeset,
        "execute saved protocol" : alpha.execute_protocol,
        "g28" : alpha.return_home,  # G-CODE equivalent comamnds
        "g29" : alpha.remap,
        "m17" : alpha.wake,
        "m18" : alpha.release,
        "m24" : alpha.execute_protocol
        }

# Main loop, awaiting command

while True:
    # Get User Input
    command = input("awaiting command\n")
    command = command.lower()
    
    if command in comdict:  # handle simple commands without arguments
        comdict[command]()
    elif command == "":     # handle blank line
        continue
    else:
        if command[0]=="p":   # interpret it as our own simplified command type
            pumpid, wellid, volume = com_parser(command)
            alpha.movetowell(pumpid,wellid)
            alpha.dispense(pumpid,volume)
        elif command[0]=="g":  # interpret it as gcode
            x, y, volumes = gcode_parser(command)
            
            #print([x,y])
            # if x or y (or both) are not specified by G-Code, then use the current value
            currentXY = alpha.current_xy()
           
            if x is None:  
                x = currentXY[0]
            if y is None:
                y = currentXY[1]

            # move to desired position
            alpha.movetoXY("center", x, y)

            # perform dispense for each pump in order 
            # NOTE: This is probably not the desired operation, as it will dispense from these pumps with the 
            #       center defined as above, rather than with the given pump output in that location.
            
            if volumes is not None:
                for (i,v) in enumerate(volumes):
                    pumpid = "p"+str(i+1)
                    alpha.dispense(pumpid, v)
        

            