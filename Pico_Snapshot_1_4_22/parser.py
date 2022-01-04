def com_parser (command):
    
    comqueue = command.split()
    
    # Parses Targeted Pump
    pp = comqueue[0]
    if pp != []:
        if (pp.find('p') != -1):
            pumpid = pp
        else:
            pumpid = []
    if pp == []:
        pumpid = []
    
    
    #Parses Targeted Action, and dispense volume
    ta = comqueue[1]
    
    if str.isdigit(ta) == True:
        action = "dispense"
        volume = int(ta)
        wellid = comqueue[2]
        
    else:
        action = "move"
        volume = []
        wellid = comqueue[1]
    
    return [pumpid,action,volume,wellid]