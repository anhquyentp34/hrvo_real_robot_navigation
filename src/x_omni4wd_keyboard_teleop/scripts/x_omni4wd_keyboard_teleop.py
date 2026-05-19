#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teleop bàn phím cho x_omni4wd (robot thật hoặc giả lập): publish geometry_msgs/Twist.

- Giống rqt_omni_robot_steering: đủ vx (linear.x), vy (linear.y), wz (angular.z), REP-103.
- Giống better_teleop: pynput bắt phím toàn hệ thống (không cần focus terminal).
- Khác better_teleop thuần: làm mượt bằng bão hòa gia tốc trên mỗi trục (tránh giật khi nhấn/thả phím).

Cài pynput: pip3 install --user pynput
"""

from __future__ import annotations

import math

import rospy
from geometry_msgs.msg import Twist

try:
    from pynput.keyboard import Key, Listener
except ImportError as exc:
    raise RuntimeError(
        "Cần pynput: pip3 install --user pynput"
    ) from exc


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _saturate_toward(current: float, target: float, max_step: float) -> float:
    err = target - current
    if abs(err) <= max_step:
        return target
    return current + math.copysign(max_step, err)


class OmniKeyboardTeleop:
    """Theo cách better_teleop: mục tiêu vận tốc từ trạng thái phím; publish vòng lặp chính."""

    def __init__(self):
        rospy.init_node("x_omni4wd_keyboard_teleop", anonymous=False)

        self._cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self._rate_hz = float(rospy.get_param("~publish_rate", 50.0))

        self._max_vx = float(rospy.get_param("~max_vx", 0.5))
        self._max_vy = float(rospy.get_param("~max_vy", 0.5))
        self._max_wz = float(rospy.get_param("~max_wz", 1.0))

        self._accel_linear = float(rospy.get_param("~accel_linear", 1.5))  # (m/s^2)
        self._accel_angular = float(rospy.get_param("~accel_angular", 2.5))  # (rad/s^2)

        # Tốc độ CRACK khi giữ phím (giống “rate” trong better_teleop / rqt bước slider).
        self._vx_forward = float(rospy.get_param("~vx_forward", self._max_vx))
        self._vx_backward = float(rospy.get_param("~vx_backward", self._max_vx))
        self._vy_left = float(rospy.get_param("~vy_left", self._max_vy))
        self._vy_right = float(rospy.get_param("~vy_right", self._max_vy))
        self._wz_ccw = float(rospy.get_param("~wz_ccw", self._max_wz))
        self._wz_cw = float(rospy.get_param("~wz_cw", self._max_wz))

        self._use_arrows = rospy.get_param("~use_arrow_keys", True)
        self._invert_angular_z = bool(rospy.get_param("~invert_angular_z", False))

        self._pub = rospy.Publisher(self._cmd_topic, Twist, queue_size=10)

        # Mục tiêu [vx, vy, wz] — cập nhật từ bàn phím
        self._tgt_vx = 0.0
        self._tgt_vy = 0.0
        self._tgt_wz = 0.0

        self._cur_vx = 0.0
        self._cur_vy = 0.0
        self._cur_wz = 0.0

        self._keys_vx = {"w": False, "s": False}
        self._keys_vy = {"q": False, "e": False}
        self._keys_wz = {"a": False, "d": False}
        self._arrow_vx = {Key.up: False, Key.down: False}
        self._arrow_wz = {Key.left: False, Key.right: False}

        self._running = True
        self._listener = Listener(on_press=self._on_press, on_release=self._on_release)
        self._listener.start()

        rospy.loginfo(
            "x_omni4wd_keyboard_teleop: topic=%s rate=%.1f Hz | max vx,vy=(%.2f,%.2f) m/s wz=%.2f rad/s",
            self._cmd_topic,
            self._rate_hz,
            self._max_vx,
            self._max_vy,
            self._max_wz,
        )
        rospy.loginfo(
            "Phím: W/S tiến/lùi, Q/E trái/phải (strafe), A/D quay CCW/CW, Space dừng, Esc thoát"
            + ("; mũi tên: lên/xuống vx, trái/phải wz" if self._use_arrows else "")
        )

    def _recompute_targets(self):
        """Gộp phím đồng thời: mỗi trục ưu tiên phím ‘dương’ nếu cả hai cùng giữ (tránh kẹt)."""
        # vx
        w, s = self._keys_vx["w"], self._keys_vx["s"]
        if self._use_arrows:
            up = self._arrow_vx[Key.up]
            down = self._arrow_vx[Key.down]
            w = w or up
            s = s or down
        if w and not s:
            self._tgt_vx = self._vx_forward
        elif s and not w:
            self._tgt_vx = -self._vx_backward
        else:
            self._tgt_vx = 0.0

        # vy (REP-103: +y trái) — Q strafe trái (+vy), E strafe phải (-vy) theo rqt_omni_robot_steering
        q, e = self._keys_vy["q"], self._keys_vy["e"]
        if q and not e:
            self._tgt_vy = self._vy_left
        elif e and not q:
            self._tgt_vy = -self._vy_right
        else:
            self._tgt_vy = 0.0

        # wz
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
        self._tgt_vy = _clamp(self._tgt_vy, -abs(self._max_vy), abs(self._max_vy))
        self._tgt_wz = _clamp(self._tgt_wz, -abs(self._max_wz), abs(self._max_wz))

    def _on_press(self, key):
        if key == Key.esc:
            self._running = False
            return

        if key == Key.space:
            self._stop_all_keys()
            self._tgt_vx = self._tgt_vy = self._tgt_wz = 0.0
            return

        ch = getattr(key, "char", None)
        if ch is not None:
            c = ch.lower()
            if c in self._keys_vx:
                self._keys_vx[c] = True
            elif c in self._keys_vy:
                self._keys_vy[c] = True
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
            elif c in self._keys_vy:
                self._keys_vy[c] = False
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
        for k in self._keys_vy:
            self._keys_vy[k] = False
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
        self._cur_vy = _saturate_toward(self._cur_vy, self._tgt_vy, max_dv)
        self._cur_wz = _saturate_toward(self._cur_wz, self._tgt_wz, max_dw)

        self._cur_vx = _clamp(self._cur_vx, -abs(self._max_vx), abs(self._max_vx))
        self._cur_vy = _clamp(self._cur_vy, -abs(self._max_vy), abs(self._max_vy))
        self._cur_wz = _clamp(self._cur_wz, -abs(self._max_wz), abs(self._max_wz))

    def _publish(self):
        msg = Twist()
        msg.linear.x = self._cur_vx
        msg.linear.y = self._cur_vy
        msg.linear.z = 0.0
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
            # Lệnh dừng an toàn
            stop = Twist()
            for _ in range(5):
                self._pub.publish(stop)
                rospy.sleep(0.05)


def main():
    teleop = OmniKeyboardTeleop()
    teleop.spin()
    return 0


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
