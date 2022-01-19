def com_parser (command):
    """Parse the movement or dispense command string."""
    
    comqueue = command.split()
    
    # Indicated Pump
    pumpid = comqueue[0]
    
    # Target Well
    try:
        wellid = comqueue[1]
    except:
        wellid = []
    
    # Target Volume
    if len(comqueue) == 3:
        try:
            volume = float(comqueue[2])
        except:
            print("Volume (microliters) not recognized, please enter in a numerical value")
            volume = 0
    else:
        volume = 0
    
    return [pumpid,wellid,volume]
