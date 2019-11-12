Listens on UDP 7890 for commands

Command format:
0: command type
    0: new frame
    1: continuation
    2: drop buffer
1: format
    0: raw
    1: run-length encoded
2-3: unused
4-5: pixel count, little endian
    for new frame, this should be the total number of pixels
    for continuations, this should be the starting pixel
