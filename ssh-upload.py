import os
import sys
with open('output_file.txt', 'w') as file:
    remote = sys.argv[1]
    remotepath = sys.argv[2]
    uf2 = sys.argv[3]
    # replace the .bin with .uf2
    uf2 = uf2.replace('.bin', '.uf2')
    elf = uf2.replace('.uf2', '.elf')
    # verify the file exists
    if os.path.isfile(uf2):
        print('\033[32mFile exists!\033[0m')  # green color
    else:
        print('\033[31mFile does not exist!\033[0m')  # red color

    # ssh copy the file to the remote device
    # first check we can connect to the remote device
    response = os.system('ping -c 1 ' + remote)
    if response == 0:
        print('\033[32mDevice is up!\033[0m')
        # copy the file to the remote device
        os.system('scp ' + uf2 + ' pi@' + remote + ':' + remotepath)
        os.system('scp ' + elf + ' pi@' + remote + ':' + remotepath)
        print('\033[32mFile copied!\033[0m')
        # Then run flash.sh on the remote device
        os.system('ssh pi@' + remote + ' "cd ' + remotepath + '; ./flash.sh ' + uf2 + '"')
        print('\033[32mFile flashed!\033[0m')
    else:
        print('\033[31mDevice is down!\033[0m')
        sys.exit()
    

    
