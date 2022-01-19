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
        "execute saved protocol" : alpha.execute_protocol
        }

# Main loop, awaiting command

while True:
    # Get User Input
    command = input("awaiting command\n")
    command = command.lower()
    
    if command in comdict:
        comdict[command]()
    else:
        parsed = com_parser(command)
        pumpid = parsed[0]
        wellid = parsed[1]
        volume = parsed[2]
        
        alpha.movetowell(pumpid,wellid)
        alpha.dispense(pumpid,volume)
        

            