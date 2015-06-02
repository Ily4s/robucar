#!/usr/bin/python


import rospy
import ttk
from Tkinter import *
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

    def update(self, msg):
        self.v = msg.speed_average
        self.fa = msg.angle_forward
        self.ra = msg.angle_rear

    def task(self):
        if self.Labels is not None:
            for l in self.Labels:
                l.pack_forget()

        self.Labels = []
        msg1 = str(self.v)
        msg2 = str(self.fa)
        msg3 = str(self.ra)
        bg = "#16a085"
        self.Labels.append(ttk.Label(self.root, text=msg1,
                                     background=bg, foreground="#efefef"))
        self.Labels.append(ttk.Label(self.root, text=msg2,
                                     background=bg, foreground="#efefef"))
        self.Labels.append(ttk.Label(self.root, text=msg3,
                                     background=bg, foreground="#efefef"))

        for l in self.Labels:
            l.pack(ipadx=10, ipady=10)

        for widget in self.root.pack_slaves():
            widget.pack_configure(fill=BOTH, expand=True)

        self.root.after(500, self.task)


if __name__ == "__main__":
    root = Tk()
    pr = Pr(root)
    root.after(500, pr.task)
    rospy.init_node('data_simple_gui', anonymous=True)

    rospy.Subscriber('myrobot_data', SimpleRobotData, pr.update, queue_size=1)

    root.mainloop()
