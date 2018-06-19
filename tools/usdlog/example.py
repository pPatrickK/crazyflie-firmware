# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re

# decode binary log data
logData = cff.decode("log00")

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'

# number of columns and rows for suplot
plotCols = 1;
plotRows = 1;

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k
    print(k)



# get plot config from user
plotGyro = 0
if re.search('gyro', keys):
    inStr = input("plot gyro data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotGyro = 1
        plotRows += 1

plotAccel = 0
if re.search('acc', keys):
    inStr = input("plot accel data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotAccel = 1
        plotRows += 1

plotMag = 0
if re.search('mag', keys):
    inStr = input("plot magnetometer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotMag = 1
        plt.xlabel('RTOS Ticks')
        plotRows += 1

plotBaro = 0
if re.search('baro', keys):
    inStr = input("plot barometer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotBaro = 1
        plotRows += 1

plotCtrl = 0
if re.search('ctrltarget', keys):
    inStr = input("plot control data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotCtrl = 1
        plotRows += 1

plotStab = 0
if re.search('stabilizer', keys):
    inStr = input("plot stabilizer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotStab = 1
        plotRows += 1

plotRadio = 0
if re.search('radio', keys):
    inStr = input("plot radio data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotRadio = 1
        plotRows += 1

plotVicon = 0
if re.search('vicon', keys):
    inStr = input("plot vicon data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotVicon = 1
        plotRows += 1

plotViconVSCtrl = 0
if re.search('vicon', keys):
    if re.search('ctrltarget', keys):
        inStr = input("plot vicon vs ctrl data (angles) ? ([Y]es / [n]o): ")
        if ((re.search('^[Yy]', inStr)) or (inStr == '')):
            plotViconVSCtrl = 1
            plotRows += 1

plotPosEstiAltitude = 0
if re.search('posEstimatorAlt', keys):
    inStr = input("plot Pose Estimator Altitude data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotPosEstiAltitude = 1
        plotRows += 1


# current plot for simple subplot usage
plotCurrent = 0;

# new figure
plt.figure(0)

if plotGyro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['gyro.x'], '-', label='X')
    plt.plot(logData['tick'], logData['gyro.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['gyro.z'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Gyroscope [°/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotAccel:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['acc.x'], '-', label='X')
    plt.plot(logData['tick'], logData['acc.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['acc.z'], '-', label='Z')
    plt.ylabel('Accelerometer [g]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)


if plotMag:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['mag.x'], '-', label='X')
    plt.plot(logData['tick'], logData['mag.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['mag.z'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Magnetometer')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotBaro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['baro.pressure'], '-')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Pressure [hPa]')

    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['baro.temp'], '-')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Temperature [degC]')

if plotCtrl:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['ctrltarget.roll'], '-', label='roll')
    plt.plot(logData['tick'], logData['ctrltarget.pitch'], '-', label='pitch')
    plt.plot(logData['tick'], logData['ctrltarget.yaw'], '-', label='yaw')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Control')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotStab:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stabilizer.roll'], '-', label='roll')
    plt.plot(logData['tick'], logData['stabilizer.pitch'], '-', label='pitch')
    plt.plot(logData['tick'], logData['stabilizer.yaw'], '-', label='yaw')
    plt.plot(logData['tick'], logData['stabilizer.thrust'], '-', label='thrust')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Stabilizer')
    plt.legend(loc=9, ncol=4, borderaxespad=0.)

if plotRadio:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['radio.rssi'], '-', label='rssi')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('RSSI')
    plt.legend(loc=9, ncol=4, borderaxespad=0.)

if plotVicon:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['vicon.x'], '-', label='X')
    plt.plot(logData['tick'], logData['vicon.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['vicon.z'], '-', label='Z')
    plt.plot(logData['tick'], logData['vicon.v_x'], '-', label='Vel X')
    plt.plot(logData['tick'], logData['vicon.v_y'], '-', label='Vel Y')
    plt.plot(logData['tick'], logData['vicon.v_z'], '-', label='Vel Z')
    plt.plot(logData['tick'], logData['vicon.roll'], '-', label='roll')
    plt.plot(logData['tick'], logData['vicon.pitch'], '-', label='pitch')
    plt.plot(logData['tick'], logData['vicon.yaw'], '-', label='yaw')
    plt.plot(logData['tick'], logData['vicon.dt'], '-', label='dT')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Vicon Data')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)


if plotViconVSCtrl:
    plt.plot(logData['tick'], logData['ctrltarget.roll'], '-', label='CTRL roll')
    plt.plot(logData['tick'], logData['ctrltarget.pitch'], '-', label='CTRL pitch')
    plt.plot(logData['tick'], logData['ctrltarget.yaw'], '-', label='CTRL yaw')
    plt.plot(logData['tick'], logData['vicon.roll'], '-', label='Vicon roll')
    plt.plot(logData['tick'], logData['vicon.pitch'], '-', label='Vicon pitch')
    plt.plot(logData['tick'], logData['vicon.yaw'], '-', label='Vicon yaw')
    plt.plot(logData['tick'], logData['vicon.dt'], '-', label='dT')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Vicon vs CTRL Data')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotPosEstiAltitude:
    plt.plot(logData['tick'], logData['posEstimatorAlt.estimatedZ'], '-', label='Esti Z')
    plt.plot(logData['tick'], logData['posEstimatorAlt.velocityZ'], '-', label='Vel Z')
    plt.plot(logData['tick'], logData['posEstimatorAlt.estVZ'], '-', label='Esti Vel Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Pose Estimator Data')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

plt.show()
