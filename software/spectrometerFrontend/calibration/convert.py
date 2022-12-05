from os import truncate
import pandas as pd
import math

df1 = pd.read_excel (r'REFERENCE DATA SHEET.xlsx', sheet_name='Normal temp compensation const', header=10)
df1CropRow = df1.truncate(before=1, after=1, axis="rows")
normalTempCompConst = df1CropRow.filter(['a','b','c','g'], axis="columns")
print("**************************************")
print("Reading Data From Excel Sheet")
print("**************************************")
print (normalTempCompConst)



df2 = pd.read_excel (r'REFERENCE DATA SHEET.xlsx', sheet_name='Temp compensation const', header=10)
dfCropRow = df2.truncate(before=1, after=1, axis="rows")
tempCompConst = dfCropRow.filter(['p','q','r','s','t','u','v','w'], axis="columns")
print (tempCompConst)

print("**************************************")
print("Getting Serial No. and Management Code")
print("**************************************")


df1 = pd.read_excel(r'REFERENCE DATA SHEET.xlsx', sheet_name='Normal temp compensation const')
df1 = df1.rename(str.lower, axis="columns")
df1CropRow = df1.truncate(before=1, after=1, axis="rows")
managementCode = df1.iat[5,3]	
serialNo = df1.iat[11,1]
print("Management Code: ", str(managementCode))
print("Serial No.: ", serialNo)

# retreiving variables from pandas dataframes

a_const = normalTempCompConst.at[1,'a']
b_const = normalTempCompConst.at[1,'b']
c_const = normalTempCompConst.at[1,'c']
g_const = normalTempCompConst.at[1,'g']

p_const = tempCompConst.at[1,'p']
q_const = tempCompConst.at[1,'q']
r_const = tempCompConst.at[1,'r']
s_const = tempCompConst.at[1,'s']
t_const = tempCompConst.at[1,'t']
u_const = tempCompConst.at[1,'u']
v_const = tempCompConst.at[1,'v']
w_const = tempCompConst.at[1,'w']

A_const = 1.326430696
Z_const = -5.54932E-07

T0_const = 25.0 #degC

tempRange_const = [5.0, 15.0, 25.0, 35.0, 45.0]

B = [None] * 3
B[0] = -a_const
B[1] = -p_const
B[2] = -t_const
C = [None] * 3
C[0] = 3 * a_const * g_const + b_const
C[1] = 3 * p_const * g_const + q_const
C[2] = 3 * t_const * g_const + u_const
D = [None] * 3
D[0] = - (3 * a_const * pow(g_const, 2) + 2 * b_const * g_const + c_const)
D[1] = - (3 * p_const * pow(g_const, 2) + 2 * q_const * g_const + r_const)
D[2] = - (3 * t_const * pow(g_const, 2) + 2 * u_const * g_const + v_const)
E = [None] * 3
E[0] = a_const * pow(g_const, 3)  + b_const * pow(g_const, 2) + c_const * g_const
E[1] = p_const * pow(g_const, 3)  + q_const * pow(g_const, 2) + r_const * g_const + s_const
E[2] = t_const * pow(g_const, 3)  + u_const * pow(g_const, 2) + v_const * g_const + w_const

a = [None] * 3
b = [None] * 3
c = [None] * 3
d = [None] * 3
e = [None] * 3
f = [None] * 3

for n in range(0,3):
    a[n] = pow(A_const, 5) *    B[n]
    b[n] = pow(A_const, 4) *    ( 5 * B[n] *    Z_const +           C[n]                                                                ) 
    c[n] = pow(A_const, 3) *    ( 10 * B[n] *   pow(Z_const, 2) +   4 * C[n] * Z_const +            D[n]                                ) 
    d[n] = pow(A_const, 2) *    ( 10 * B[n] *   pow(Z_const, 3) +   6 * C[n] * pow(Z_const, 2) +    3 * D[n] * Z_const +    E[n]        ) 
    e[n] = A_const * Z_const *  ( 5 * B[n] *    pow(Z_const, 3) +   4 * C[n] * pow(Z_const, 2) +    3 * D[n] * Z_const +    2 * E[n]    ) 
    f[n] = pow(Z_const, 2) *    ( B[n] *        pow(Z_const, 3) +   C[n] * pow(Z_const, 2) +        D[n] * Z_const +        E[n]        ) 

minWavelength = 1550
maxWavelength = 1850
lookupTable = []
for wavelength in range(minWavelength, maxWavelength + 1):
    wavelength = wavelength * 1e-9
    tempTable = []
    for temp in tempRange_const:
        controlVoltageSquared = 0.0
        for n in range(0,3):
            controlVoltageSquared += ( a[n] * pow(wavelength, 5) + b[n] * pow(wavelength, 4) + c[n] * pow(wavelength, 3) + d[n] * pow(wavelength, 2) + e[n] * wavelength + f[n] ) * pow(temp - T0_const, n)
        controlVoltage = math.sqrt(controlVoltageSquared)
        tempTable.append(int(controlVoltage * 1000))
    lookupTable.append(tempTable)

# begin writing to file
filename = "FPI-" + serialNo + ".h"
f = open(filename, "w")
f.write("/*************************************************************\n")
f.write("   Copyright (c) 2021 Armin Straller.   \n")
f.write("   Auto generated file with information provided by HAMAMATSU \n")
f.write("   Serial No.: " + serialNo + "\n")
f.write("   Management Code: " + str(managementCode) + "\n")
f.write("*************************************************************/\n")
f.write("\n")

f.write("/* Define to prevent recursive inclusion -------------------------------------*/\n")
f.write("#ifndef __FPI_H\n")
f.write("#define __FPI_H\n")
f.write("\n")
#f.write("/* Includes ------------------------------------------------------------------*/\n")
#f.write("#include \"main.h\"\n")
#f.write("\n")

f.write("/* Defines ------------------------------------------------------------------*/\n")
f.write("#define WAVELENGTH_MIN " + str(minWavelength) + " // unit: [nm] \n")
f.write("#define WAVELENGTH_MAX " + str(maxWavelength) + " // unit: [nm] \n")
f.write("#define VFPI_MIN " + str(int(lookupTable[len(lookupTable)-1][0])) + " // unit: [mV]\n")
f.write("#define VFPI_MAX " + str(int(lookupTable[0][len(tempRange_const)-1])) + " // unit: [mV]\n")
f.write("#define WAVELENGTH_LOOKUP_LEN " + str(len(lookupTable)) + "\n")
f.write("#define TEMP_LOOKUP_LEN " + str(len(tempRange_const)) + " // unit: [mV]\n")

f.write("\n")
f.write("// Serial Number of the MEMS-FPI Sensor \n")
f.write("static const char serialNumber[] = \"" + serialNo + "\";\n")
f.write("\n")
f.write("// Serial Number of the MEMS-FPI Sensor \n")
f.write("static const char managementCode[] = \"" + str(managementCode) + "\";\n")
f.write("\n")
f.write("// Lookup Table for Reference Temperatures \n")
f.write("// from 5 degC to 45 degC \n")
f.write("static const uint8_t tempLookup[TEMP_LOOKUP_LEN] =\n")
f.write("{ ")
for temp in tempRange_const:
    f.write(str(temp))
    if not temp == tempRange_const[-1]:
        f.write(", ")
f.write(" };\n")
f.write("\n")
f.write("// Lookup Table for Control Voltages \n")
f.write("// x-Direction from 1550 nm to 1850 nm \n")
f.write("// y-Direction from 5 degC to 45 degC \n")
f.write("static const uint16_t fpiLookup[WAVELENGTH_LOOKUP_LEN][TEMP_LOOKUP_LEN] =\n")
f.write("{\n")

for i in range(0, len(lookupTable)):
    f.write("   { ") 
    for j in range(0, len(tempRange_const)):
        f.write(str(lookupTable[i][j]))
        if not j == len(tempRange_const)-1:
            f.write(", ")
    f.write(" }")
    if not i == len(lookupTable)-1:
        f.write(",")
    f.write("\n")
f.write("};\n")
f.write("\n")
f.write("#endif /* __FPI_H */")
f.close()