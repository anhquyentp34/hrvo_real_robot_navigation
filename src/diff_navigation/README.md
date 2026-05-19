# diff_navigation

Goi navigation cho `diff_bot` (move_base + costmap + planner configs).

## Noi dung

- Launch move_base cho cac mode:
  - `move_base_TEB.launch`
  - `move_base_TEB1_velodyne.launch`
  - `move_base_hrvo.launch`
- Config costmap/planner trong `config/`.
- Launch tong hop `final*.launch` de chay localization + move_base + RViz.

## Chay nhanh (vi du)

```bash
source ~/hrvo_real_robot_navigation/devel/setup.bash
roslaunch diff_navigation final1_velodyne.launch
```

## Ket hop voi simulation moi

`diff_simulation/diff_bot_simulation.launch` goi truc tiep launch trong goi nay qua tham so:

- `local_planner:=teb`
- `local_planner:=hrvo`

(che do `dwa` dung launch cua `robot_gazebo` de giu tuong thich.)
