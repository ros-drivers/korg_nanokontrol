#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
# Author: Allison Thackston

import roslib
roslib.load_manifest('korg_nanokontrol')
import rospy
import rospkg
import rosparam

import pygame
import pygame.midi

import sys

from sensor_msgs.msg import Joy


def main():

    # connect to
    pygame.midi.init()
    devices = pygame.midi.get_count()
    if devices < 1:
        print "No MIDI devices detected"
        exit(-1)
    print "Found %d MIDI devices" % devices

    if len(sys.argv) > 1:
        input_dev = int(sys.argv[1])
    else:
        input_dev = pygame.midi.get_default_input_id()
        if input_dev == -1:
            print "No default MIDI input device"
            exit(-1)
    print "Using input device %d" % input_dev

    controller = pygame.midi.Input(input_dev)

    # start node
    rospy.init_node('kontrol')
    pub = rospy.Publisher('joy', Joy, latch=True)

    # load in default parameters if not set
    if not rospy.has_param('modes'):
        rospack = rospkg.RosPack()
        paramlist=rosparam.load_file(rospack.get_path('korg_nanokontrol')+'/config/nano_kontrol_config.yaml')
        for params, ns in paramlist:
            rosparam.upload_params(ns,params)

    modes = rospy.get_param('modes')

    m = Joy()
    m.axes = [0] * 18
    m.buttons = [0] * 25
    mode = None

    p = False

    while not rospy.is_shutdown():
        m.header.stamp = rospy.Time.now()
        # count the number of events that are coalesced together
        c = 0
        while controller.poll():
            c += 1
            data = controller.read(1)
            # print data
            # loop through events received
            for event in data:
                control = event[0]
                timestamp = event[1]

                # look for continuous controller commands
                if (control[0] & 0xF0) == 176:
                    control_id = control[1] | ((control[0] & 0x0F) << 8)

                    # guess initial mode based on command
                    if mode is None:
                        candidate = None
                        for index, mode in enumerate(modes):
                            if control_id in mode['control_axis']:
                                if candidate is not None:
                                    candidate = None
                                    break
                                candidate = index

                            if control_id in mode['control_buttons']:
                                if candidate is not None:
                                    candidate = None
                                    break
                                candidate = index
                        mode = candidate
                        if mode is None:
                            print 'skipped because mode is yet unknown'
                            continue

                    control_axis = modes[mode]['control_axis']
                    control_buttons = modes[mode]['control_buttons']

                    if control_id in control_axis:
                        control_val = float(control[2] - 63) / 63.0
                        if control_val < -1.0:
                            control_val = -1.0
                        if control_val > 1.0:
                            control_val = 1.0

                        axis = control_axis[control_id]
                        m.axes[axis] = control_val
                        p = True

                    if control_id in control_buttons:
                        button = control_buttons.index(control_id)
                        if control[2] != 0:
                            m.buttons[button] = 1
                        else:
                            m.buttons[button] = 0
                        p = True

                # look for mode commands
                elif control[0] == 79:
                    mode = control[1]
                    m.buttons[24] = mode
                    p = True

        if p:
            pub.publish(m)
            p = False

        rospy.sleep(0.1)  # 10Hz maximum input


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
