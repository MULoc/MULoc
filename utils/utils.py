def byte2txt(byteData,path,name,anchor_num):

    byteData = byteData[100:]

    index = byteData.find(b'\x07\x06\x05\x04\x03')
    byteData = byteData[index+5:]

    curLen = 0
    maxLen = len(byteData)
    last = 0
    strData = ''
    last_byte = 0

    while curLen < maxLen - 45:

        last = len(strData)
        last_byte = curLen
        for i in range(anchor_num):

            
            for j in range(anchor_num-1):

                real = int.from_bytes(byteData[curLen:curLen+2], byteorder='little')
                curLen += 2
                imag = int.from_bytes(byteData[curLen:curLen+2], byteorder='little')
                curLen += 2

                phase_correction = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
                curLen += 1

                # rssi = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
                # curLen += 1

                rxPC = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
                curLen += 1

                maxGC = int.from_bytes(byteData[curLen:curLen+2], byteorder='big')
                curLen += 2

                timestamp = int.from_bytes(byteData[curLen:curLen+5], byteorder='little')
                curLen += 5

                # strData += format(real, '04x')+','+format(imag, '04x')+','+format(timestamp, '010x')+','
                # strData += f"{format(real, '04x')},{format(imag, '04x')},{format(phase_correction, '02x')},{format(rssi, '02x')},{format(timestamp, '010x')},"
                strData += f"{format(real, '04x')},{format(imag, '04x')},{format(phase_correction, '02x')},{format(rxPC, '02x')},{format(maxGC, '04x')},{format(timestamp, '010x')},"
                
                if j < i:
                    strData += str(j)+','
                else:
                    strData += str(j+1)+','
            
            idx = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
            curLen += 1

            strData += format(idx, '02x') + '\n'
        # print(strData[last:])
        # print(byteData[last_byte:curLen].hex(',',1))

        last = len(strData)
        last_byte = curLen
        for i in range(anchor_num):

            real = int.from_bytes(byteData[curLen:curLen+2], byteorder='little')
            curLen += 2
            imag = int.from_bytes(byteData[curLen:curLen+2], byteorder='little')
            curLen += 2

            phase_correction = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
            curLen += 1

            # rssi = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
            # curLen += 1

            rxPC = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
            curLen += 1

            maxGC = int.from_bytes(byteData[curLen:curLen+2], byteorder='big')
            curLen += 2

            timestamp = int.from_bytes(byteData[curLen:curLen+5], byteorder='big')
            curLen += 5

            ci = int.from_bytes(byteData[curLen:curLen+4], byteorder='big')
            curLen += 4

            # strData += format(real, '04x')+','+format(imag, '04x')+','+format(phase_correction, '02x')+','+format(timestamp, '010x')+','+format(ci, '08x')+','+str(i)+','
            # strData += format(real, '04x')+','+format(imag, '04x')+','+format(timestamp, '010x')+','+format(ci, '08x')+','+str(i)+','
            # strData += f"{format(real, '04x')},{format(imag, '04x')},{format(phase_correction, '02x')},{format(rssi, '02x')},{format(timestamp, '010x')},{format(ci, '08x')},{i},"
            strData += f"{format(real, '04x')},{format(imag, '04x')},{format(phase_correction, '02x')},{format(rxPC, '02x')},{format(maxGC, '04x')},{format(timestamp, '010x')},{format(ci, '08x')},{i},"

        idx = int.from_bytes(byteData[curLen:curLen+1], byteorder='big')
        curLen += 1
        strData += format(idx, '02x') + '\n'

        # print(strData[last:])
        curLen += 5
                    
    file = open(path+name+'_str'+'.txt', 'w+')
    file.write(strData)
    file.close() 
        
def seperate(path, name, tag_num, anchor_num):

    # Read all original data from tag file
    file = open(path + name+'_str'+'.txt', 'r')
    lines = file.readlines()
    file.close()

    filenames = []
    for i in range(anchor_num):
        filenames.append(path+name+'_anchor'+str(i+1)+'.txt')

    filenames.append(path+name+'_tag1'+'.txt')

    files = []
    for filename in filenames:
        files.append(open(filename, 'w+'))
    
    cnt = 0
    seq_num = ''
    line_temp = []
    line_len = []

    for line in lines:
        # Delete the final '\n' read from the serial
        line = line.strip('\n')
        
        # New message in the same round
        if seq_num == line[-1]:
            cnt += 1
            line_temp.append(line)
            
        # End of the round
        else:

            # If there are tag_num + anchor_num messages in total
            if cnt == anchor_num + 1:

                # Write these messages into files
                for i in range(anchor_num + 1):
                    files[i].writelines(line_temp[i]+'\n')

            seq_num = line[-1]
            cnt = 1
            line_temp = [line]

    for file in files:
        file.close()
