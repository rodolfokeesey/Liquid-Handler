def read_instructions(filename)
    test = open(filename)
    stuff = test.read()
    stuff = stuff.replace("\r", "").split("\n")
    stuff = stuff[2:len(stuff)]

    cmd = []

    for i in range(len(stuff)):
        print(i)
        cmd.append(stuff[i].split(","))

    return[cmd]

