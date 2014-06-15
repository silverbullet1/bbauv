#!/usr/bin/env python

import signal
import curses
from curses import wrapper

import rospy
import actionlib
from bbauv_msgs.msg import ControllerAction, ControllerGoal, compass_data
from bbauv_msgs.srv import set_controller

def normAngle(angle):
    while angle < 0:
        angle += 360
    return angle % 360

class Control:
    def __init__(self):
        # Forward, Sidemove, isRelative, Heading, Depth
        self.setpoints = [0.0, 0.0, 1, 0.0, 0.0]

        self.curHeading = 0.0
        self.comSub = rospy.Subscriber("/euler", compass_data, self.compassCb)

        self.setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)
        self.motionClient = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)

        try:
            rospy.loginfo("Waiting for LocomotionServer...")
            self.motionClient.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("LocomotionServer timeout!")

    def sendMovement(self, f, s, h, d):
        goal = ControllerGoal(forward_setpoint=f, heading_setpoint=h,
                              sidemove_setpoint=s, depth_setpoint=d)
        self.motionClient.send_goal(goal)

    def goalCb(self):
        pass

    def compassCb(self, data):
        self.curHeading = data.yaw

    def enable(self):
        self.setServer(forward=True, sidemove=True, heading=True, depth=True,
                       pitch=True, roll=True, topside=False, navigation=False)

    def startGoal(self):
        if self.setpoints[2] == 1:
            self.sendMovement(self.setpoints[0], self.setpoints[1],
                              normAngle(self.curHeading + self.setpoints[3]),
                              self.setpoints[4])
        else:
            self.sendMovement(self.setpoints[0], self.setpoints[1],
                              self.setpoints[3], self.setpoints[4])

    def endGoal(self):
        self.motionClient.cancel_all_goals()

    def disable(self):
        self.setServer(*([False]*10))

def get_param(screen, prompt_string, posY):
     screen.addstr(posY, 3, prompt_string, curses.A_HORIZONTAL)
     screen.addstr(" ")
     screen.refresh()
     input = screen.getstr()
     return input

def get_setpoints(screen, controller):
    promptStrs = map(lambda s: s + " (current = {}):",
                     ["Forward", "Sidemove", "Is Relative", "Heading", "Depth"])
    setpoints = controller.setpoints
    screen.clear()
    screen.border(0)
    curses.echo()
    screen.addstr(2, 2, "Dear Sir, please enter where you want to go\n",
                  curses.color_pair(1) | curses.A_UNDERLINE)
    for prompt in enumerate(promptStrs):
        data = get_param(screen,
                         prompt[1].format(setpoints[prompt[0]]),
                         prompt[0]+4)
        if not data.strip(): continue
        if (prompt[0] == 2):
            try:
                data = int(data)
                controller.setpoints[prompt[0]] = data
            except ValueError:
                return False
        else:
            try:
                data = float(data)
                controller.setpoints[prompt[0]] = data
            except ValueError:
                return False

    return True

def main(stdscr):
    controller = Control()

    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_WHITE)

    c = 0
    statusLine = "Status:"
    while (c != ord('5')):
        stdscr.clear()
        stdscr.border(0)
        stdscr.addstr(2, 2, "Hello my good sir! What do you want to do?\n",
                      curses.A_BOLD)
        stdscr.addstr(4, 4, "1 - Send Movement")
        stdscr.addstr(5, 4, "2 - End Goal")
        stdscr.addstr(6, 4, "3 - Disable")
        stdscr.addstr(7, 4, "4 - Enable")
        stdscr.addstr(8, 4, "5 - Exit")

        stdscr.addstr(10, 2, statusLine,
                      curses.color_pair(1) | curses.A_UNDERLINE)

        stdscr.refresh()
        c = stdscr.getch()

        if c == ord('1'):
            if get_setpoints(stdscr, controller):
                controller.enable()
                controller.startGoal()
                statusLine = "Status: Goal started"
            else:
                statusLine = "Status: Goal invalid"
        elif c == ord('2'):
            controller.endGoal()
            statusLine = "Status: Goal ended"
        elif c == ord('3'):
            controller.disable()
            statusLine = "Status: Disabled"
        elif c == ord('4'):
            controller.enable()

if __name__ == "__main__":
    rospy.init_node("control_cli")
    wrapper(main)
