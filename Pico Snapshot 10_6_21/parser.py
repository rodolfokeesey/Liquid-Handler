def com_parser (command):
    
    comqueue = command.split()
    
    # Parses Targeted Pump
    pp = [x for x in comqueue if x == "pump"]
    if pp != []:
        pindex = comqueue.index("pump")
        pumpnum = pindex + 1
        if str.isdigit(comqueue[pumpnum]) == True:
            pumpid = "p" + str(int(comqueue[pumpnum]))
        else:
            pumpid = []
    if pp == []:
        pumpid = []
    
    
    #Parses Targeted Action, Dispense first
    ta = [x for x in comqueue if x == "dispense"]
    if ta != []:
        dispindex = comqueue.index("dispense")
        action = "dispense"
        
        # Parse how much to dispense
        volumeind = dispindex + 1
        if str.isdigit(comqueue[volumeind]) == True:
            volume = int(comqueue[volumeind])
        if str.isdigit(comqueue[volumeind]) != True:
            volume = []
    #If the action is not dispense, it must be move
    if ta == []: 
        ta = [x for x in comqueue if x == "move"]
        if ta != []:
            moveindex = comqueue.index("move")
            action = "move"
            volume = []
        if ta == []:
            action = []
            volume = []
    #Parses target well or location
    twl = [x for x in comqueue if x == "well"]
    if twl != []:
        wellindex = comqueue.index("well")
        wellid = comqueue[wellindex+1]
    if twl == []:
        twl = [x for x in comqueue if x == "purge"]
        if twl != []:
            wellid = "purge"
        if twl == []:
            wellid = []
        
    
    
    return [pumpid,action,volume,wellid]
