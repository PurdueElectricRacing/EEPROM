headerInfo = []
headerCount = 0
filePath = "./putty.log"

with open(filePath, 'rb') as fid:
    headerCount = ord(fid.read(1))
    print("Header count: "+str(headerCount))

    for i in range(0, headerCount):
        headerEntry = []
        name = fid.read(3).decode('UTF-8')
        #print("Name: " + name)
        version = ord(fid.read(1))
        overWriteProtect = version >> 7
        version = version & 0b01111111

        #print("Value: " + str(version))
        #print("Overwrite: " + str(overWriteProtect))
        size = ord(fid.read(1))
        size = size | (ord(fid.read(1)) << 8)
        #print("Size: " + str(size))
        address = ord(fid.read(1))
        address = address | (ord(fid.read(1)) << 8)
        #print("Address: " + str(address))
        headerEntry.append(name)
        headerEntry.append(version)
        headerEntry.append(overWriteProtect)
        headerEntry.append(size)
        headerEntry.append(address)
        headerInfo.append(headerEntry)

    for header in headerInfo:
        fid.seek(header[4])
        header.append(str(list(fid.read(header[3]))))
        print("Name: {} Version: {} OW: {} Size: {} Addr: {} Value: {}".format(*header))

a = input("enter to quit")