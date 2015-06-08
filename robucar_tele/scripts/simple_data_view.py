#!/usr/bin/python


import rospy
import ttk
from Tkinter import *
import tkFont
from robucar_driver.msg import SimpleRobotData


class Pr(object):

    """docstring for Pr"""

    def __init__(self, r):
        super(Pr, self).__init__()
        self.v = 0
        self.fa = 0
        self.ra = 0
        self.Labels = None
        self.root = r
        self.font = tkFont.Font(family='Helvetica', size=21, weight='bold')

    def update(self, msg):
        print msg
        self.v = msg.speed_average
        self.fa = msg.angle_forward
        self.ra = msg.angle_rear

    def task(self):
        if self.Labels is not None:
            for l in self.Labels:
                l.pack_forget()

        self.Labels = []
        msg1 = "speed :  " + str(self.v)
        msg2 = "front angle :  " + str(self.fa)
        msg3 = "rear angle :  " + str(self.ra)
        bg = "#16a085"
        self.Labels.append(ttk.Label(self.root, text=msg1, font=self.font,
                                     background="#efefef", foreground=bg))
        self.Labels.append(ttk.Label(self.root, text=msg2, font=self.font,
                                     background="#efefef", foreground=bg))
        self.Labels.append(ttk.Label(self.root, text=msg3, font=self.font,
                                     background="#efefef", foreground=bg))

        for l in self.Labels:
            l.pack(ipadx=10, ipady=10)

        for widget in self.root.pack_slaves():
            widget.pack_configure(fill=BOTH, expand=True)

        self.root.after(500, self.task)


if __name__ == "__main__":
    root = Tk()
    pr = Pr(root)
    root.after(200, pr.task)
    rospy.init_node('data_simple_gui', anonymous=True)

    rospy.Subscriber('myrobot_data', SimpleRobotData, pr.update, queue_size=1)

    root.mainloop()
