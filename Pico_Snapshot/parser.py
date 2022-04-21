
def gcode_parser(command):
    """Parse Gcode commands"""
    x = None
    y = None
    volumes = None

    tokens = command.split()
    tokens.reverse()

    #print(tokens)
    cmd = tokens.pop()

    if cmd != "g0":
        print("error:  gcode command ", cmd, "is not supported.  Only G0 based moves are supported.")
        return [x, y, volumes]
    
    while (tokens):
        cmd = tokens.pop()

        if cmd == "x": # handle space between X and input
            x = float(tokens.pop())/10.
        elif cmd[0] == "x": #handle no space after x
            x = float(cmd[1:])/10.  # gcode uses units of millimeters, convert to centimeters
        elif cmd == "y":
            y = float(tokens.pop())/10.
        elif cmd[0] == "y":
            y = float(cmd[1:])/10.
        elif cmd == "e":
            volumes = [float(v) for v in tokens.pop().split(";")] # assume uL units
        elif cmd[0] == "e":
            volumes = [float(v) for v in cmd[1:].split(";")]

    return [x, y, volumes]


def com_parser (command):
    """Parse the movement or dispense command string."""
    
    tokens = command.split()

    # Indicated Pump
    pumpid = tokens[0]
    
    # Target Well
    try:
        wellid = tokens[1]
    except:
        wellid = []
    
    # Target Volume
    if len(tokens) == 3:
        try:
            volume = float(tokens[2])
        except:
            print("Volume (microliters) not recognized, please enter in a numerical value")
            volume = 0
    else:
        volume = 0
    
    return [pumpid,wellid,volume]
