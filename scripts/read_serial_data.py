import serial
from time import sleep
import csv
import time

COM = '/dev/cu.usbmodem141101'
filepath = "data/" 
toprow = ["Wavelength", "Temperature", "TargetControlVoltage", "ControlVoltage", "ADCVoltage"]
measurements = []

def addData(serText, printFlag):
    measurement = serText.split("$", 5)[1:]
    if len(measurement) != 5:
        return TypeError

    if printFlag == True:
        print(serText, end="\r\n", flush=True)
    
    measurements.append(measurement)

def clearData():
    measurements.clear()


def measure(filename):
    BAUD = 250000
    ser = serial.Serial(COM, BAUD, timeout = .1)
    print('Waiting for device');
    sleep(0.1)
    print(ser.name)
    new_measurement_flag = False

    while True:
        if ser.is_open == False:
            ser.open()
        ser.flushInput()
        ser.flush()
        val = str(ser.readline().decode().strip('\r\n'))
        if "DATA:$1850$" in val:
            new_measurement_flag = True
            clearData()
            addData(val, False)
            val = ""
        while new_measurement_flag:
            val = str(ser.readline().decode().strip('\r\n'))
            if "DATA:$1850$" in val:
                new_measurement_flag = False
                ser.close()
                with open(filepath + filename + ".csv", 'w') as writeFile:
                    writer = csv.writer(writeFile)
                    writer.writerow(toprow)
                    writer.writerows(measurements)
                writeFile.close()
                
                return
            addData(val, False)


def main():
    cmd = 'n'
    scan_again = 'n'
    while 1:
        if scan_again == 'n' :
            plastic_type = input('Input plastic TYPE (PET, HDPE, PVC, LDPE, PP, PS, other): ')
            plastic_color = input('Input plastic COLOR (green, white, blue, clear, other): ')
            cmd = input('Please remove any objects for reference measurement. Ready? (y/n)? ')
        elif scan_again == 'y' :
            cmd = 'y'
        else:
            scan_again = 'n'
            cmd = 'n'
        
        filename = ''
        if cmd == 'y':
            filename = time.strftime("%Y%m%d") + "-" + time.strftime("%H%M%S") + "-" + plastic_type + "-" + plastic_color
            measure(filename + '-ref')
        
        cmd = input('Now place the ' + plastic_color + ' ' + plastic_type + '. Ready? (y/n)? ')
        if cmd == 'y':
            measure(filename)
        
        cmd = 'n'
        scan_again = input('Do you want to redo the measurement (y/n)? ')
        

if __name__ == "__main__":
    main()