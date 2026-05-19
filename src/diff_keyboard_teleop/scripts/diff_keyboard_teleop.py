#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teleop bàn phím cho robot vi sai (diff): publish geometry_msgs/Twist.

- Chỉ dùng linear.x (tiến/lùi) và angular.z (quay); linear.y luôn 0 (diff không strafe).
- Giống x_omni4wd_keyboard_teleop: pynput bắt phím toàn hệ thống, làm mượt bão hòa gia tốc.

Cài pynput: pip3 install --user pynput
"""

from __future__ import annotations

import math

import rospy
from geometry_msgs.msg import Twist

try:
    from pynput.keyboard import Key, Listener
except ImportError as exc:
    raise RuntimeError("Cần pynput: pip3 install --user pynput") from exc


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _saturate_toward(current: float, target: float, max_step: float) -> float:
    err = target - current
    if abs(err) <= max_step:
        return target
    return current + math.copysign(max_step, err)


class DiffKeyboardTeleop:
    def __init__(self):
        rospy.init_node("diff_keyboard_teleop", anonymous=False)

        self._cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self._rate_hz = float(rospy.get_param("~publish_rate", 50.0))

        self._max_vx = float(rospy.get_param("~max_vx", 0.5))
        self._max_wz = float(rospy.get_param("~max_wz", 1.0))

        self._accel_linear = float(rospy.get_param("~accel_linear", 1.5))
        self._accel_angular = float(rospy.get_param("~accel_angular", 2.5))

        self._vx_forward = float(rospy.get_param("~vx_forward", self._max_vx))
        self._vx_backward = float(rospy.get_param("~vx_backward", self._max_vx))
        self._wz_ccw = float(rospy.get_param("~wz_ccw", self._max_wz))
        self._wz_cw = float(rospy.get_param("~wz_cw", self._max_wz))

        self._use_arrows = rospy.get_param("~use_arrow_keys", True)
        self._invert_angular_z = bool(rospy.get_param("~invert_angular_z", False))

        self._pub = rospy.Publisher(self._cmd_topic, Twist, queue_size=10)

        self._tgt_vx = 0.0
        self._tgt_wz = 0.0
        self._cur_vx = 0.0
        self._cur_wz = 0.0

        self._keys_vx = {"w": False, "s": False}
        self._keys_wz = {"a": False, "d": False}
        self._arrow_vx = {Key.up: False, Key.down: False}
        self._arrow_wz = {Key.left: False, Key.right: False}

        self._running = True
        self._listener = Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

        rospy.loginfo(
            "diff_keyboard_teleop: topic=%s rate=%.1f Hz | max vx=%.2f m/s wz=%.2f rad/s",
            self._cmd_topic,
            self._rate_hz,
            self._max_vx,
            self._max_wz,
        )
        rospy.loginfo(
            "Phím: W/S tiến/lùi, A/D quay CCW/CW, Space dừng, Esc thoát"
            + ("; mũi tên: lên/xuống vx, trái/phải wz" if self._use_arrows else "")
        )

    def _recompute_targets(self):
        w, s = self._keys_vx["w"], self._keys_vx["s"]
        if self._use_arrows:
            w = w or self._arrow_vx[Key.up]
            s = s or self._arrow_vx[Key.down]
        if w and not s:
            self._tgt_vx = self._vx_forward
        elif s and not w:
            self._tgt_vx = -self._vx_backward
        else:
            self._tgt_vx = 0.0

        a, d = self._keys_wz["a"], self._keys_wz["d"]
        if self._use_arrows:
            a = a or self._arrow_wz[Key.left]
            d = d or self._arrow_wz[Key.right]
        if a and not d:
            self._tgt_wz = self._wz_ccw
        elif d and not a:
            self._tgt_wz = -self._wz_cw
        else:
            self._tgt_wz = 0.0

        self._tgt_vx = _clamp(self._tgt_vx, -abs(self._max_vx), abs(self._max_vx))
        self._tgt_wz = _clamp(self._tgt_wz, -abs(self._max_wz), abs(self._max_wz))

    def _on_press(self, key):
        if key == Key.esc:
            self._running = False
            return

        if key == Key.space:
            self._stop_all_keys()
            self._tgt_vx = self._tgt_wz = 0.0
            return

        ch = getattr(key, "char", None)
        if ch is not None:
            c = ch.lower()
            if c in self._keys_vx:
                self._keys_vx[c] = True
            elif c in self._keys_wz:
                self._keys_wz[c] = True
            self._recompute_targets()
            return

        if self._use_arrows:
            if key in self._arrow_vx:
                self._arrow_vx[key] = True
            elif key in self._arrow_wz:
                self._arrow_wz[key] = True
            self._recompute_targets()

    def _on_release(self, key):
        ch = getattr(key, "char", None)
        if ch is not None:
            c = ch.lower()
            if c in self._keys_vx:
                self._keys_vx[c] = False
            elif c in self._keys_wz:
                self._keys_wz[c] = False
            self._recompute_targets()
            return

        if self._use_arrows:
            if key in self._arrow_vx:
                self._arrow_vx[key] = False
            elif key in self._arrow_wz:
                self._arrow_wz[key] = False
            self._recompute_targets()

    def _stop_all_keys(self):
        for k in self._keys_vx:
            self._keys_vx[k] = False
        for k in self._keys_wz:
            self._keys_wz[k] = False
        for k in self._arrow_vx:
            self._arrow_vx[k] = False
        for k in self._arrow_wz:
            self._arrow_wz[k] = False

    def _smooth_step(self, dt: float):
        max_dv = self._accel_linear * dt
        max_dw = self._accel_angular * dt
        self._cur_vx = _saturate_toward(self._cur_vx, self._tgt_vx, max_dv)
        self._cur_wz = _saturate_toward(self._cur_wz, self._tgt_wz, max_dw)

        self._cur_vx = _clamp(self._cur_vx, -abs(self._max_vx), abs(self._max_vx))
        self._cur_wz = _clamp(self._cur_wz, -abs(self._max_wz), abs(self._max_wz))

    def _publish(self):
        msg = Twist()
        msg.linear.x = self._cur_vx
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        wz = self._cur_wz
        if self._invert_angular_z:
            wz = -wz
        msg.angular.z = wz
        self._pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self._rate_hz)
        try:
            while self._running and not rospy.is_shutdown():
                dt = 1.0 / self._rate_hz
                self._smooth_step(dt)
                self._publish()
                rate.sleep()
        finally:
            self._listener.stop()
            stop = Twist()
            for _ in range(5):
                self._pub.publish(stop)
                rospy.sleep(0.05)


def main():
    teleop = DiffKeyboardTeleop()
    teleop.spin()
    return 0


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
