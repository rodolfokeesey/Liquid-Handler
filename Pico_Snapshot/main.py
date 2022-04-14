from Sidekick_object import Sidekick
from parser import com_parser


# Defines an instance of the Sidekick
alpha = Sidekick(7,3,10,0.5,[0,0],[90,178],[119.484,171.701])

# On startup, homes position
alpha.initialize()
#alpha.release()


# Dictionary of available commands
comdict = {
        "initialize" : alpha.initialize,
        "hardware check" : alpha.hardwarecheck,
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
        "m18" : alpha.freemove,
        "m23" : alpha.execute_protocol
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
            pumpid, x, y, volumes = gcode_parser(command)
            
            if x is not None and y is not None:
                ### TODO - ROD—functions for moving to specific xy location
                alpha.movetoXY("center", x, y)

            if volumes is not None:
                for (i,v) in enumerate(volumes):
                    pumpid = "p"+str(i+1)
                    alpha.dispense(pumpid, v)
        

            