import csv
import numpy as np
from os import walk

import matplotlib.pyplot as plt
from sklearn.preprocessing import minmax_scale
from scipy import signal

pathToData = 'data/'

def main():
    (_, _, filenames) = next(walk(pathToData))
    
    plt.title('plastics')
    for filename_ref in filenames:
        refMeasurements = []
        measurements = []
        file_title = ''
        #first fill the measurements array
        if filename_ref.split("-", 4)[4:] == ['ref.csv'] :
            with open(pathToData + filename_ref, 'r') as readFile:
                readerMeas = csv.reader(readFile)
                refMeasurements = list(readerMeas)[1:]
            readFile.close() 
            for filename in filenames:
                if filename.split("-", 4)[1] == filename_ref.split("-", 4)[1]:
                    if filename.split("-", 4)[4:] != ['ref.csv'] :
                        file_title = (filename.split("-", 4)[2]+'_'+ filename.split("-", 4)[3]).split(".",1)[0]
                        with open(pathToData + filename, 'r') as readFile:
                            readerMeasRef = csv.reader(readFile)
                            measurements = list(readerMeasRef)[1:]
                        readFile.close()
            
        if len(measurements) > 0 and len(refMeasurements) > 0:
            # then grab only the first and last element and store it in a numpy array
            measurements = np.array(measurements)
            measurements = np.delete(measurements, (1,2,3), axis=1)

            refMeasurements = np.array(refMeasurements)
            refMeasurements = np.delete(refMeasurements, (1,2,3), axis=1)
            
            # grab the wavelenth array
            x = measurements.astype(np.float)[:, 0]

            # subtract reference measurement
            meas = measurements.astype(np.float)[:, 1] - refMeasurements.astype(np.float)[:, 1]
            # apply minmax
            meas_norm = minmax_scale(meas, feature_range=(0,1))
            
            # filter the measured data
            b, a = signal.butter(8, 0.075)
            measFilt = signal.filtfilt(b, a, meas_norm, padlen=150)
            
            # now plot it
            color = 'grey'
            if "PP" in file_title:
                color = 'blue'
                plt.plot(x, measFilt, color=color, label=file_title)
            if "HDPE" in file_title:
                color = 'green'
                plt.plot(x, measFilt, color=color, label=file_title)
            if "PS" in file_title:
                color = 'orange'
                plt.plot(x, measFilt, color=color, label=file_title)
            if "PET" in file_title:
                color = 'red'
                plt.plot(x, measFilt, color=color, label=file_title)
            if "other" in file_title:
                color = 'grey'
                plt.plot(x, measFilt, color=color, label=file_title)
            
            plt.ylabel('reflection intensity')
            plt.xlabel('wavelength[nm]')
            
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()