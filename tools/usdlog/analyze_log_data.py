# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re
import getopt
import os
import sys

def usage():
    print ('Options:')
    print ('None / default: "log00.txt" should be in the script directory')
    print ('-f /--file_name/--dir_name=  <Path/with/file/name without File Extension or Path/to/Dir >')
    print ('-k /--key_group=  <Key group name(s) as comma separated list>')
    print ('-a /--all  (If true, all available data is plotted)')
    # print ('-v /--vicon=      (If true, vicon data is plotted if available)')
    # print ('-s /--save_name=  <Name for the saved plots> (If added, all used plots are saved to disk with save_name)')
    print ('Example:        python3 analyze_log_data.py -f log06')
    print ('With full path: python3 analyze_log_data.py -f path/to/log_name')

# "main" function that will be run if the script is run from command-line
def main():
    try:
        opts, _ = getopt.getopt(sys.argv[1:], 'f:k:v:s:a', ['file_name=','key_group=','vicon=','save_name=','all'])
        # In case there are no command-line options, maybe the script is run by
        # exec(open("./path/to/script.py").read(), globals())
        # If so, let's search for the options in the global variables.
        if len(opts) == 0:
            if 'param_file_name' in globals():
                opts.append(('-f', param_file_name))
            if 'param_all' in globals():
                opts.append(('-a', ''))
            # TODO I did not even try to finish this ...
        print(opts)
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    plot_all = False
    key_to_plot_alone = None
    for opt, arg in opts:
        if opt in ('-f', '--file_name', '--dir_name'):
            log_name = arg
        elif opt in ('-a', '--all'):
            plot_all = True
        elif opt in ('-k', '--key_group'):
            key_to_plot_alone = arg
    plot_log_data(log_name, plot_all, key_to_plot_alone)

# plot functions
def include_log(key_group,group_label,sub_key_list,sub_key_label_list,plot_y_label,plot_title):
    # See plot_log_data(): We use globals to omit too many parameters (just for
    # convenience)
    global multi_log
    global plot_all
    global key_to_plot_alone
    global keys
    global logData

    inStr = ''
    plotsPerFigure = 4
    if re.search(key_group, keys):
        if not plot_all and not key_to_plot_alone:
            inStr = input('plot ' + str(group_label) + ' data? ([Y]es / [n]o): ')
        if ((re.search('^[Yy]', inStr)) or (inStr == '') or plot_all or key_to_plot_alone):
            for sub_key_input in sub_key_list:
                bSubKeyMissing = False
                if (key_group + '.' + sub_key_input) in logData:
                    print(key_group + '.' + sub_key_input + ' vorhanden')
                else:
                    print(key_group + '.' + sub_key_input + ' nicht vorhanden')
                    bSubKeyMissing = True
            if bSubKeyMissing:
                return
            if multi_log:
                # fig Name
                fig_name = ''
                for label in sub_key_label_list:
                    fig_name = fig_name + '| ' + str(label) + ' '
                (nFigures, nlogsLastFig) = divmod(len(log_list),plotsPerFigure)
                cFigures = 1
                for c, (logDataMulti, log_name_nulti) in enumerate(zip(logDataList, log_list)):
                    if (c % 4) == 0:
                        plt.figure(str(group_label + ' ' + fig_name + ' ' + str(cFigures)))
                        cFigures += 1
                    (nSubplot, divSubPlots) = divmod(c,plotsPerFigure)
                    plt.subplot(plotsPerFigure,1,divSubPlots+1)
                    if c == 0: plt.title(plot_title)
                    for (sub_key, sub_key_label) in zip(sub_key_list,sub_key_label_list):
                        plt.plot(logDataMulti['tick'], logDataMulti[key_group + '.' + sub_key], '-', label=(sub_key_label + ' ' + str(log_name_nulti)))
                    plt.xlabel('RTOS Ticks')
                    plt.ylabel(plot_y_label)
                    plt.legend(loc=0, ncol=3, borderaxespad=0.)
            else:
                # fig Name
                fig_name = ''
                for label in sub_key_label_list:
                    fig_name = fig_name + str(label)
                plt.figure(str(group_label + ' ' + fig_name))
                for (sub_key, sub_key_label) in zip(sub_key_list,sub_key_label_list):
                    plt.plot(logData['tick'], logData[key_group + '.' + sub_key], '-', label=sub_key_label)
                plt.xlabel('RTOS Ticks')
                plt.ylabel(plot_y_label)
                plt.title(plot_title)
                plt.legend(loc=0, ncol=3, borderaxespad=0.)
    return

def plot_log_data(log_name = 'log00', m_plot_all = False, m_key_to_plot_alone = None):
    # We use globals here so that we do not have to pass all these arguments
    # to the include_log function
    global multi_log
    global plot_all
    global key_to_plot_alone
    global keys
    global logData

    plot_all = m_plot_all
    key_to_plot_alone = m_key_to_plot_alone

    if os.path.isdir(log_name):
        log_list = os.listdir(arg)
        log_dir = arg
        multi_log = True
    elif os.path.isfile(log_name):
        multi_log = False
    else:
        print("File/Directory does not exist.")
        sys.exit(2)

    if multi_log:
        logDataList = []
        for log_file in log_list:
            logData = cff.decode(os.path.join(log_dir,log_file))
            logDataList.append(logData)
            printed = []
            for data in logDataList:
                keys = ""
                for k, v in data.items():
                    keys += k
                    if k not in printed:
                        print(k)
                    printed.append(k)
    else:
        # decode binary log data
        logData = cff.decode(log_name)
        # let's see which keys exists in current data set
        keys = ""
        for k, v in logData.items():
            keys += k
            print(k)

    # TODO In case there is no command-line option at all, the following line crashes
    # because key_to_plot_alone is not set to None but completely undefined.
    if key_to_plot_alone != None:
        keys = key_to_plot_alone
        print(keys)


    # set window background to white
    plt.rcParams['figure.facecolor'] = 'w'
    plt.close('all')


    # usage of "include new data to log plots" functions
    # include_log(logging group name [string],name to ask in console [string],logging group keys [list of strings],
    #   logging group key legend labels [list of strings],ylabel [string],title [string]):
    #
    # IF ONLY ONE LOGGING GROUP KEY USE NON CURLY BRACKETS AROUND THE STRING INPUT !!!
    # Example: (['aZimu'])

    # TODO This does not work as expected: I wanted to have pyplot non-interactive
    # so that the figures do no appear yet when include_log() is run. But they
    # appeared anyways (when I tested last time).
    plt.ioff()

    # Vicon
    include_log('vicon','Vicon Position',('x','y','z'),('X','Y','Z'),'Position [m]','Vicon Position')
    #include_log('vicon','Vicon Velocity',('v_x','v_y','v_z'),('vX','vY','vZ'),'Velocity [m/s]','Vicon Velocity')
    #include_log('vicon','Vicon System Latency',(['dt']),(['dT']),'Time since last Pkg [ms]','Vicon Package Arrival')
    #include_log('vicon','Start Flag',(['startFlag']),(['startFlag']),'Time since last Pkg [ms]','startFlag')
    #include_log('vicon','FuckYou',(['fuckYou']),(['fuckYou']),'Time since last Pkg [ms]','fuckYou')
    include_log('vicon','Vicon Angles',('pitch','roll'),('pitch','roll'),'pitch and roll [Deg]','Actual Angles')
    # #  Control Data
    # include_log('ctrltarget','CTRL Target',('roll','pitch','yaw'),('Roll','Pitch','Yaw'),'Object Rot Angle [°]','Control Target Data')
    # include_log('ctrltarget','CTRL Target Data',('emergencyStop','upsideDown'),('Emergency Stop ','Upside Down '),'Triggered? [bool]','Control Target FLAGS')
    # include_log('ctrltarget','CTRL Target Data',(['aZimu']),(['Acc Z from IMU']),'Z Acc [m/s²]','Control Target Z Acc')
    # # IMU Data
    # include_log('gyro','Gyroscope',('x','y','z'),('X','Y','Z'),'Position [m]','Gyroscope Rotation')
    # include_log('acc','Acceleration IMU',('x','y','z'),('a_X','a_Y','a_Z'),'Acceleration [m/s²]','Acceleration from IMU')
    # include_log('mag','Magnetometer IMU',('x','y','z'),('g_X','g_Y','g_Z'),'Magnetometer [?]','Magnetometer from IMU')
    # include_log('baro','Pressure IMU',('pressure'),('Pressure'),'Pressure [bar?]','Pressure from IMU')
    # # Stabilizer
    # include_log('stabilizer','Stabilizer',('roll','pitch','yaw'),('Roll','Pitch','Yaw'),'Object Rot Angle [°]','Stabilizer Values')
    # include_log('stabilizer','Stabilizer Thrust',(['thrust']),(['Thrust']),'Thrust [?]','Stabilizer only Thrust')
    # # Radio
    # include_log('radio','Radio Data',(['rssi']),(['RSSI']),'Signal Strength [?]','Radio Signal Strength')
    # # Motor data
    # include_log('motor','Motor Values',('m1','m2','m3','m4'),('Motor 1','Motor 2','Motor 3','Motor 4'),'Motor Thrust [?]','Motor Thrusts')
    # include_log('pwm','Motor PWM',('m1_pwm','m2_pwm','m3_pwm','m4_pwm'),('PWM 1','PWM 2','PWM 3','PWM 4'),'PWM Magnitude [uint16]','PWM Signal')
    # # Battery Values
    # include_log('pm','Power Management Battery',(['vbat']),(['Voltage Batt']),'Voltage [V]','Battery Voltage')
    # # Pose Estimator
    # include_log('posEstimatorAlt','Pose Estimator',('estimatedZ','velocityZ','estVZ'),('Esti Z','Vel Z','Esti Vel Z'),'Pose Estimator Data [?]','Pose Estimator')
    # # Situation Awareness Flags
    # include_log('sitAw','Situation Awareness',('ARDetected','TuDetected','FFAccWZDetected'),('At rest','tumbled','freefall'),'Enabled [bool]','Situation Awareness FLAGS')
    # # Mellinger Controler Data
    # include_log('ctrlMel','Controler Mellinger',('spR','spP','spY'),('spR','spP','spY'),'Setpoint Angular Rates [?]','Mellinger Setpoint Rates')
    # include_log('ctrlMel','Controller Mellinger',('i_err_x','i_err_y','i_err_z'),('errorX','errorY','errorZ'),'?','Mellinger Position Error')
    include_log('ctrlMel','Controller Mellinger',('pitchd','rolld'),('pitchd','rolld'),'pitch and roll [Deg]','Desired Angles')
    include_log('ctrlMel','Controller Mellinger',('Mx','My'),('Mx','My'),'moments','By Mellinger calculated moments')
    include_log('ctrlMel','Controller Mellinger',(['cThrust']),(['current thrust']),'current thrust','current thrust from Mellinger Controller')
    # Estimator Kalman Data
    #include_log('estKal','Estimator Kalman',('pitch_e','roll_e'),('pitch estimated','roll estimated'),'pitch and roll [Deg]','Estimated Angles')
    # Estimator KalmanUSC Data
    include_log('kalmanUSC','Estimator Kalman USC',('pitch_eUSC','roll_eUSC'),('pitch estimated','roll estimated'),'pitch and roll [DEG]','Estimated Angles (USC)')
    include_log('motor','MOTORVAL unclipped',('logM1','logM2','logM3','logM4'),('M1','M2','M3','M4'),'motorval unclipped','Motorvalues logged')

    # If this is run as a script from the command-line, it should be non-interactive
    # because otherwise, the figures are closed as soon as the program is finished
    # which is just after the plots have been drawn.
    # If the function is run from an interactive python session, plotting should
    # also be interactive so that we can interact with the figures afterwards.
    if __name__ == "__main__":
        plt.ioff()
    else:
        plt.ion()
    plt.show()

# Adjust all axes in all figures so that they match the current axes in the
# current figure, i.e. make everything the same in x-direction!
def adjust_xlim():
    xlims = plt.gcf().gca().get_xlim()

    for fignum in plt.get_fignums():
        for ax in plt.figure(fignum).get_axes():
            ax.set_xlim(xlims)

if __name__ == "__main__":
    main()
