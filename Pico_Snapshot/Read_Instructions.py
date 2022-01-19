def read_instructions(filename):
    test = open(filename)
    stuff = test.read()
    stuff = stuff.replace("\r", "").split("\n")
    stuff = stuff[2:]

    cmd = []

    for thing in stuff:
        print(thing)
        cmd.append(thing.split(","))

    return [cmd]

