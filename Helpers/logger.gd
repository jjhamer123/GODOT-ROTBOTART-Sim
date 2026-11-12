extends Node
class_name BaseLinkLogger
## Logs where the robot's end (for Robot Art: the pen tip) is, plus every joint
## value, to the console and to a CSV file you can load straight into MATLAB.
##
## Positions are in ROS axes relative to the robot's base (x, y = paper plane,
## z = up, metres), so they compare directly with the URDF / MATLAB maths.

@export var godot_robot: GodotRobot
## Optional: log every joint's value too (rad, or m for sliding joints).
@export var joint_controller: RobotJointController

@export_group("What to track")
## URDF link to track. Blank = "pen" if the robot has one, otherwise the deepest
## leaf link (the end-effector for an arm). Set it for grippers/branching arms.
@export var end_link_name: String = ""
## Point on that link to track, in the link's Godot axes (URDF z = Godot y).
## Robot Art's pen tip is (0, -0.13, 0); it is filled in automatically for "pen".
@export var tip_offset: Vector3 = Vector3.ZERO

@export_group("Output")
## Seconds between CSV rows (0.02 = 50 rows a second, plenty for a drawing).
@export var log_interval_sec: float = 0.02
## Seconds between console prints (0 = never print).
@export var print_interval_sec: float = 1.0
@export var write_to_file: bool = true
## %s is replaced by the date and time. user:// is ~/.local/share/godot/app_userdata/<project>/
@export var csv_path: String = "user://logs/robot_log_%s.csv"

const PEN_TIP := Vector3(0, -0.13, 0)

var _end_body: Node3D
var _offset := Vector3.ZERO
var _file: FileAccess
var _joint_names: Array[String] = []
var _t := 0.0
var _log_timer := 0.0
var _print_timer := 0.0


func _ready() -> void:
	reinitialize()


func reinitialize() -> void:
	_close_file()
	_end_body = null
	if not godot_robot:
		return   # ArmRig assigns the robot and calls reinitialize() once it has loaded it
	if godot_robot.links.is_empty() and godot_robot.has_method("rebuild_caches"):
		godot_robot.rebuild_caches()

	var link_name := end_link_name
	if link_name.is_empty():
		link_name = "pen" if godot_robot.links.has("pen") else _find_deepest_real_leaf_link()
	_end_body = godot_robot.links.get(link_name) as Node3D
	if not _end_body:
		push_warning("BaseLinkLogger: link '%s' not found - not logging" % link_name)
		return
	_offset = tip_offset
	if link_name == "pen" and tip_offset == Vector3.ZERO:
		_offset = PEN_TIP

	_joint_names.clear()
	if joint_controller:
		var fixed := {}
		for j in godot_robot.urdf.joints:
			if j.type == "fixed":
				fixed[j.name] = true
		for j in joint_controller.get_joint_names():
			if not fixed.has(j):   # fixed joints never move: leave them out
				_joint_names.append(j)
	_t = 0.0
	_log_timer = log_interval_sec   # write a row straight away
	if write_to_file:
		_open_file(link_name)
	print("BaseLinkLogger: tracking '%s'%s, %d joints" % [link_name,
			" tip" if _offset != Vector3.ZERO else "", _joint_names.size()])


func _physics_process(delta: float) -> void:
	if not _end_body:
		return
	# the controller may finish setting up after us: pick its joints up late
	if joint_controller and _joint_names.is_empty() and not joint_controller.get_joint_names().is_empty():
		reinitialize()
		return
	_t += delta
	_log_timer += delta
	_print_timer += delta
	var do_log := _file != null and _log_timer >= log_interval_sec
	var do_print := print_interval_sec > 0.0 and _print_timer >= print_interval_sec
	if not (do_log or do_print):
		return

	var p := get_tip_position()
	var rpy := get_tip_rpy()
	if do_log:
		_log_timer = 0.0
		var row := PackedStringArray([ "%.4f" % _t,
			"%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
			"%.5f" % rpy.x, "%.5f" % rpy.y, "%.5f" % rpy.z ])
		for j in _joint_names:
			row.append("%.6f" % joint_controller.get_joint_angle(j))
		_file.store_line(",".join(row))
		_file.flush()
	if do_print:
		_print_timer = 0.0
		print("END  t=%.2fs  pos=(%.1f, %.1f, %.1f) mm  rpy=(%.1f, %.1f, %.1f) deg" % [
			_t, p.x * 1000, p.y * 1000, p.z * 1000,
			rad_to_deg(rpy.x), rad_to_deg(rpy.y), rad_to_deg(rpy.z)])


## Tracked point relative to the robot base, in ROS axes (m).
func get_tip_position() -> Vector3:
	var g: Vector3 = godot_robot.global_transform.affine_inverse() * (_end_body.global_transform * _offset)
	return Vector3(g.x, -g.z, g.y)   # Godot (x, y up, z) -> ROS (x, y, z up)


## Roll, pitch, yaw of the tracked link relative to the robot base, ROS convention (rad).
func get_tip_rpy() -> Vector3:
	var gb: Basis = (godot_robot.global_transform.basis.inverse() * _end_body.global_transform.basis).orthonormalized()
	# change of axes Godot -> ROS: R_ros = M^T * R_godot * M
	var m := Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))
	var r := m.transposed() * gb * m
	# r[col][row]; ROS rpy is R = Rz(yaw) * Ry(pitch) * Rx(roll)
	var roll := atan2(r.y.z, r.z.z)
	var pitch := asin(clamp(-r.x.z, -1.0, 1.0))
	var yaw := atan2(r.x.y, r.x.x)
	return Vector3(roll, pitch, yaw)


func _open_file(link_name: String) -> void:
	var path := csv_path % Time.get_datetime_string_from_system().replace(":", "-") if csv_path.contains("%s") else csv_path
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	_file = FileAccess.open(path, FileAccess.WRITE)
	if _file == null:
		push_warning("BaseLinkLogger: can't write %s" % path)
		return
	var header := PackedStringArray(["t_s", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"])
	header.append_array(_joint_names)
	_file.store_line("# robot=%s link=%s offset=%s (ROS axes, metres, relative to robot base)" % [godot_robot.name, link_name, _offset])
	_file.store_line(",".join(header))
	print("BaseLinkLogger: writing %s" % ProjectSettings.globalize_path(path))


func _close_file() -> void:
	if _file:
		_file.close()
		_file = null


func _exit_tree() -> void:
	_close_file()


# ---------------------------------------------------------------- end link

func _find_deepest_real_leaf_link() -> String:
	if not godot_robot or not godot_robot.urdf:
		return ""
	var real_links: Dictionary = godot_robot.links
	var parent_of: Dictionary = {}
	var has_real_child: Dictionary = {}
	for joint in godot_robot.urdf.joints:
		if real_links.has(joint.parent) and real_links.has(joint.child):
			parent_of[joint.child] = joint.parent
			has_real_child[joint.parent] = true
	var deepest_name := ""
	var deepest_depth := -1
	var depth_cache: Dictionary = {}
	for link_name in real_links.keys():
		if has_real_child.has(link_name):
			continue
		var depth := _real_link_depth(link_name, parent_of, depth_cache)
		if depth > deepest_depth:
			deepest_depth = depth
			deepest_name = link_name
	return deepest_name


func _real_link_depth(link_name: String, parent_of: Dictionary, cache: Dictionary) -> int:
	if cache.has(link_name):
		return cache[link_name]
	if not parent_of.has(link_name):
		cache[link_name] = 0
		return 0
	var depth: int = 1 + _real_link_depth(parent_of[link_name], parent_of, cache)
	cache[link_name] = depth
	return depth
