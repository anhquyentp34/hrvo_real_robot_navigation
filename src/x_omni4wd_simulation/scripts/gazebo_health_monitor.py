#!/usr/bin/env python3
"""Theo dõi gzserver/gzclient và model robot; in cảnh báo khi Gazebo out."""
import subprocess
import rospy
from gazebo_msgs.srv import GetWorldProperties


def pgrep(name):
    try:
        out = subprocess.check_output(["pgrep", "-c", name], stderr=subprocess.DEVNULL)
        return int(out.strip())
    except subprocess.CalledProcessError:
        return 0


def gazebo_service_alive():
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=0.5)
        return True
    except rospy.ROSException:
        return False


def main():
    rospy.init_node("gazebo_health_monitor", anonymous=True)
    model = rospy.get_param("~robot_model", "x_omni4wd")
    period = float(rospy.get_param("~period", 2.0))
    warned_gz = False
    warned_model = False

    get_world = None
    rospy.loginfo("Gazebo health monitor: model=%s", model)

    while not rospy.is_shutdown():
        n_server = pgrep("gzserver")
        n_client = pgrep("gzclient")
        gz_ok = gazebo_service_alive() or n_server > 0

        if not gz_ok:
            if not warned_gz:
                rospy.logerr(
                    "Gazebo physics không phản hồi (gzserver có thể segfault). "
                    "Chạy: scripts/relaunch_omni_hrvo.sh"
                )
                warned_gz = True
        else:
            warned_gz = False

        if rospy.get_param("/use_sim_time", False) and n_server > 0:
            try:
                if get_world is None:
                    rospy.wait_for_service("/gazebo/get_world_properties", timeout=1.0)
                    get_world = rospy.ServiceProxy(
                        "/gazebo/get_world_properties", GetWorldProperties
                    )
                props = get_world()
                if model not in props.model_names:
                    if not warned_model:
                        rospy.logwarn(
                            "Model '%s' không có trong Gazebo (có %d model). Spawn thất bại hoặc đã văng.",
                            model,
                            len(props.model_names),
                        )
                        warned_model = True
                else:
                    warned_model = False
            except Exception:
                pass

        if n_client == 0 and rospy.get_param("~expect_gui", True):
            rospy.logwarn_throttle(30.0, "gzclient không chạy — cửa sổ Gazebo GUI đã tắt (exit 134 thường gặp). gzserver vẫn có thể chạy.")

        rospy.sleep(period)


if __name__ == "__main__":
    main()
