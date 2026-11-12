extends CanvasLayer
class_name SimOverlay
## On-screen status + joint table.
## Robot Art robots (joint1..N / clampB1..N) get one row per layer and the pen
## position; any other robot (e.g. the Mover6) gets one row per moving joint.
## The table rebuilds itself when the robot is reloaded with different joints.

#have these point at the ROS2 bridge and the arm controller
@export var joint_controller: RobotJointController
@export var ros_bridge: RosArmBridge

#control visability
@export var toggle_key: Key = KEY_TAB
@export var overlay_visibility_key: Key = KEY_H
## Seconds between table refreshes (values are smoothed over this time).
@export var refresh_sec: float = 0.1

const PEN_TIP := Vector3(0, -0.13, 0)   # pen tip in the pen link (Godot axes)

var _panel: PanelContainer
var _title_label: Label
var _status_label: Label
var _mode_label: Label
var _pen_label: Label

var _table_panel: PanelContainer
var _table_title: Label
var _grid: GridContainer

var _joint_names: Array[String] = []          # names the table was built for
var _layers := 0                              # > 0: Robot Art layout
var _cells: Dictionary[String, Label] = {}    # "<joint>" -> value label, "<joint>/v" -> speed label
var _prev: Dictionary[String, float] = {}
var _timer := 0.0


func _ready() -> void:
	layer = 10  # draw above the 3D viewport
	_build_status_panel()
	_build_table_skeleton()


# ---------------------------------------------------------------- panels

func _build_status_panel() -> void:
	_panel = PanelContainer.new()
	_panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	_panel.position = Vector2(16, 16)
	_panel.custom_minimum_size = Vector2(260, 0)
	add_child(_panel)

	var margin := _margin()
	_panel.add_child(margin)
	var vbox := VBoxContainer.new()
	margin.add_child(vbox)

	_title_label = Label.new()
	_title_label.add_theme_font_size_override("font_size", 18)
	vbox.add_child(_title_label)
	_status_label = Label.new()
	vbox.add_child(_status_label)
	_mode_label = Label.new()
	vbox.add_child(_mode_label)
	_pen_label = Label.new()
	_pen_label.visible = false
	vbox.add_child(_pen_label)

	vbox.add_child(HSeparator.new())
	var controls_header := Label.new()
	controls_header.text = "Controls"
	controls_header.add_theme_font_size_override("font_size", 13)
	vbox.add_child(controls_header)
	var controls := [
		"[%s]  Toggle Simulation / Shadow" % OS.get_keycode_string(toggle_key),
		"[%s]  Show / hide overlay" % OS.get_keycode_string(overlay_visibility_key),
		"Click viewport  Capture mouse look",
		"[Esc]  Release mouse",
		"[W A S D]  Move camera",
		"[Space / E]  Move up",
		"[Ctrl / Q]  Move down",
		"[Shift]  Sprint",
	]
	for line in controls:
		var hint := Label.new()
		hint.text = line
		hint.add_theme_font_size_override("font_size", 11)
		hint.modulate = Color(1, 1, 1, 0.65)
		vbox.add_child(hint)


func _build_table_skeleton() -> void:
	_table_panel = PanelContainer.new()
	_table_panel.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
	_table_panel.anchor_top = 1.0
	_table_panel.anchor_bottom = 1.0
	_table_panel.grow_vertical = Control.GROW_DIRECTION_BEGIN
	_table_panel.position = Vector2(16, -16)
	add_child(_table_panel)

	var margin := _margin()
	_table_panel.add_child(margin)
	var vbox := VBoxContainer.new()
	margin.add_child(vbox)
	_table_title = Label.new()
	_table_title.text = "Joint States"
	_table_title.add_theme_font_size_override("font_size", 14)
	vbox.add_child(_table_title)
	vbox.add_child(HSeparator.new())
	_grid = GridContainer.new()
	_grid.add_theme_constant_override("h_separation", 18)
	_grid.add_theme_constant_override("v_separation", 4)
	vbox.add_child(_grid)


func _margin() -> MarginContainer:
	var m := MarginContainer.new()
	for side in ["left", "right"]:
		m.add_theme_constant_override("margin_" + side, 12)
	for side in ["top", "bottom"]:
		m.add_theme_constant_override("margin_" + side, 10)
	return m


# ---------------------------------------------------------------- table

## Moving joints only (fixed joints never change), in the controller's order.
func _moving_joints() -> Array[String]:
	var fixed := {}
	var robot = joint_controller.godot_robot
	if robot and robot.urdf:
		for j in robot.urdf.joints:
			if j.type == "fixed":
				fixed[j.name] = true
	var out: Array[String] = []
	for n in joint_controller.get_joint_names():
		if not fixed.has(n):
			out.append(n)
	return out


func _rebuild_table(names: Array[String]) -> void:
	for c in _grid.get_children():
		c.queue_free()
	_cells.clear()
	_prev.clear()
	_joint_names = names
	_layers = 0
	while names.has("joint%d" % (_layers + 1)) and names.has("clampB%d" % (_layers + 1)):
		_layers += 1

	if _layers > 0:
		# Robot Art: one row per layer
		_table_title.text = "Layers (%d)" % _layers
		var head := ["Layer", "Arm (°)", "Speed (°/s)", "Radius (mm)", "Bearing (°)", "Bar slide (mm)"]
		_grid.columns = head.size()
		for h in head:
			_grid.add_child(_cell(h, true))
		for i in range(1, _layers + 1):
			_grid.add_child(_cell(str(i), true))
			_add_value("joint%d" % i)
			_add_value("joint%d" % i, true)
			_add_value("clampB%d" % i)
			_add_value("pivot%d" % i, false, i == 1)        # layer 1's bearing never turns
			_add_value("support%d" % i)
	else:
		# any other robot: one row per moving joint
		_table_title.text = "Joint States"
		_grid.columns = 3
		for h in ["Joint", "Value", "Speed"]:
			_grid.add_child(_cell(h, true))
		for n in names:
			_grid.add_child(_cell(n, true))
			_add_value(n)
			_add_value(n, true)
	for n in names:
		_prev[n] = joint_controller.get_joint_angle(n)


func _add_value(joint: String, speed := false, blank := false) -> void:
	var c := _cell("–" if blank or not _joint_names.has(joint) else "0.0")
	c.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	_grid.add_child(c)
	if not blank and _joint_names.has(joint):
		_cells[joint + ("/v" if speed else "")] = c


func _cell(text: String, header := false) -> Label:
	var label := Label.new()
	label.text = text
	if header:
		label.add_theme_font_size_override("font_size", 12)
		label.modulate = Color(1, 1, 1, 0.7)
	return label


## Value of a joint for display: mm for sliding joints, degrees otherwise.
func _show(joint: String, value: float, per_sec := false) -> String:
	if joint_controller.is_prismatic(joint):
		return "%.1f" % (value * 1000.0) + (" mm/s" if per_sec and _layers == 0 else (" mm" if _layers == 0 else ""))
	return "%.1f" % rad_to_deg(value) + (" °/s" if per_sec and _layers == 0 else ("°" if _layers == 0 else ""))


# ---------------------------------------------------------------- input / update

func _unhandled_key_input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed and not event.echo):
		return
	if event.keycode == toggle_key:
		if ros_bridge:
			ros_bridge.toggle_mode()
	elif event.keycode == overlay_visibility_key:
		visible = not visible


func _process(delta: float) -> void:
	if ros_bridge:
		_title_label.text = ros_bridge.robot_name
		_status_label.text = "● Connected" if ros_bridge.is_online() else "○ No signal"
		_status_label.modulate = Color(0.4, 1.0, 0.4) if ros_bridge.is_online() \
			else Color(1.0, 0.4, 0.4)
		var how := ""
		if joint_controller:
			how = "  (kinematic)" if joint_controller.kinematic_mode else "  (physics)"
		_mode_label.text = "Mode: " + ros_bridge.mode_name() + how
		_mode_label.modulate = Color(0.5, 0.8, 1.0) if ros_bridge.mode == RosArmBridge.Mode.SIMULATION \
			else Color(1.0, 0.8, 0.3)

	if joint_controller == null or joint_controller.get_joint_names().is_empty():
		return
	_timer += delta
	if _timer < refresh_sec:
		return
	var dt := _timer
	_timer = 0.0

	# robot reloaded with different joints (e.g. a new number of layers)?
	var names := _moving_joints()
	if names != _joint_names:
		_rebuild_table(names)
		return

	for n in _joint_names:
		var v := joint_controller.get_joint_angle(n)
		var p: float = _prev.get(n, v)
		var d := v - p if joint_controller.is_prismatic(n) else angle_difference(p, v)  # no jump at ±180°
		_prev[n] = v
		if _cells.has(n):
			_cells[n].text = _show(n, v)
		if _cells.has(n + "/v"):
			_cells[n + "/v"].text = _show(n, d / dt, true)

	_update_pen()


## Pen tip on the paper (Robot Art only), mm, ROS axes relative to the robot base.
func _update_pen() -> void:
	var robot = joint_controller.godot_robot
	var pen: Node3D = robot.links.get("pen") if robot else null
	_pen_label.visible = pen != null
	if pen:
		var g: Vector3 = robot.global_transform.affine_inverse() * (pen.global_transform * PEN_TIP)
		_pen_label.text = "Pen  x %.1f  y %.1f mm" % [g.x * 1000.0, -g.z * 1000.0]
