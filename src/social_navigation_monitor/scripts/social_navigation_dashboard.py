#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tam thoi chi hien thi 2 do thi SII va SGI.
"""

import os
import queue
import select
import sys
from collections import deque
from datetime import datetime

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from social_navigation_monitor.msg import SII, SGI

LABEL_FS = 12
TITLE_FS = 14
TITLE_WEIGHT = "bold"
TICK_FS = 10
FIGSIZE = (14, 5)
ANIM_INTERVAL_MS = 120

sii_queue = queue.Queue()
sgi_queue = queue.Queue()
social_start_time = None


def _apply_axis_font(ax):
    ax.title.set_fontsize(TITLE_FS)
    ax.title.set_fontweight(TITLE_WEIGHT)
    ax.xaxis.label.set_fontsize(LABEL_FS)
    ax.yaxis.label.set_fontsize(LABEL_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)


def get_message_time_sec(msg):
    if hasattr(msg, "header") and msg.header and msg.header.stamp:
        stamp_sec = msg.header.stamp.to_sec()
        if stamp_sec > 0:
            return stamp_sec
    return rospy.Time.now().to_sec()


def sii_callback(msg):
    global social_start_time
    msg_time_sec = get_message_time_sec(msg)
    if social_start_time is None:
        social_start_time = msg_time_sec
    t = max(0.0, msg_time_sec - social_start_time)
    sii_queue.put((t, float(msg.sii)))


def sgi_callback(msg):
    global social_start_time
    msg_time_sec = get_message_time_sec(msg)
    if social_start_time is None:
        social_start_time = msg_time_sec
    t = max(0.0, msg_time_sec - social_start_time)
    sgi_queue.put((t, float(msg.sgi)))


def main():
    rospy.init_node("social_navigation_dashboard", anonymous=True)

    max_points = int(rospy.get_param("~max_plot_points", 6000))

    sii_data = deque(maxlen=max_points)
    sgi_data = deque(maxlen=max_points)

    mpl.rcParams["figure.autolayout"] = False
    fig, axes = plt.subplots(1, 2, figsize=FIGSIZE, constrained_layout=True)
    fig.canvas.manager.set_window_title("Social Dashboard (SII/SGI)")
    ax_sii, ax_sgi = axes

    ax_sii.set_xlabel("Time (s)")
    ax_sii.set_ylabel("SII")
    ax_sii.set_title("Social Individual Index (SII)")
    ax_sii.grid(True, alpha=0.3)
    ax_sii.set_ylim(0, 1)
    sii_line, = ax_sii.plot([], [], "b-", label="SII", linewidth=1.5)
    ax_sii.axhline(y=0.14, color="cyan", linestyle="--", linewidth=1, label="Tc 0.14")
    ax_sii.axhline(y=0.54, color="red", linestyle="--", linewidth=1, label="Tp 0.54")
    ax_sii.legend(loc="upper right", fontsize=TICK_FS)
    _apply_axis_font(ax_sii)

    ax_sgi.set_xlabel("Time (s)")
    ax_sgi.set_ylabel("SGI")
    ax_sgi.set_title("Social Group Index (SGI)")
    ax_sgi.grid(True, alpha=0.3)
    ax_sgi.set_ylim(0, 1)
    sgi_line, = ax_sgi.plot([], [], "r-", label="SGI", linewidth=1.5)
    ax_sgi.axhline(y=0.14, color="cyan", linestyle="--", linewidth=1, label="Tg 0.14")
    ax_sgi.legend(loc="upper right", fontsize=TICK_FS)
    _apply_axis_font(ax_sgi)

    rospy.Subscriber("/sii", SII, sii_callback, queue_size=50)
    rospy.Subscriber("/sgi", SGI, sgi_callback, queue_size=50)

    def update_plot(_frame):
        while not sii_queue.empty():
            sii_data.append(sii_queue.get())
        while not sgi_queue.empty():
            sgi_data.append(sgi_queue.get())

        if sii_data:
            tx, ty = zip(*sii_data)
            sii_line.set_data(tx, ty)
            t0, t1 = min(tx), max(tx)
            ax_sii.set_xlim(t0, t1 if t1 > t0 else t0 + 1e-6)

        if sgi_data:
            tx, ty = zip(*sgi_data)
            sgi_line.set_data(tx, ty)
            t0, t1 = min(tx), max(tx)
            ax_sgi.set_xlim(t0, t1 if t1 > t0 else t0 + 1e-6)

    _ani = FuncAnimation(fig, update_plot, interval=ANIM_INTERVAL_MS, cache_frame_data=False)
    plt.show(block=True)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
