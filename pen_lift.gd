extends Node
class_name PenLift
## Pen up / pen down for the Robot Art sim, over ROS 2 (std_msgs/Bool).
##
##   /pen_down_demand   IN:  true = put the pen down, false = lift it   (MATLAB: setPenDown)
##   /pen_down_pos      OUT: where the pen actually is (true = down)
##
## The pen takes Move Time to go up or down (like a servo pen lift), so _pos lags
## the demand. While it is up the paper doesn't draw and the pen line is drawn lifted.
## The PenXYLogger writes both values as columns in its CSV.
##
## Add a Node anywhere in the scene and give it this script. Pen Paper and Pen Line
## are found automatically if you leave them empty.

@export var demand_topic: String = "/pen_down_demand"
@export var position_topic: String = "/pen_down_pos"
## Pen state when the sim starts (before MATLAB says anything).
@export var start_down: bool = true
## Seconds for the pen to go up or down (0 = instant).
@export var move_time_sec: float = 0.15
## How far the pen lifts (m); only changes how the pen line looks.
@export var lift_height: float = 0.01
## /pen_down_pos is sent whenever it changes and also this often (s), so late
## subscribers still get it.
@export var publish_interval_sec: float = 0.1
## Key to toggle the pen by hand (useful without ROS). NONE = off.
@export var toggle_key: Key = KEY_X
@export var pen_paper: PenPaper
@export var pen_line: PenLine

## What MATLAB asked for (true = down).
var pen_down_demand: bool
## Where the pen is (true = down). Changes Move Time after the demand.
var pen_down_pos: bool

var _ros: RefCounted = null
var _switch_timer := 0.0
var _pub_timer := 0.0
var _line_tip := Vector3.ZERO


func _ready() -> void:
	add_to_group("pen_lift")
	pen_down_demand = start_down
	pen_down_pos = start_down
	# BoolTopic only exists in the Godot-ROS build: without it this still works by key
	if ClassDB.class_exists("BoolTopic"):
		_ros = ClassDB.instantiate("BoolTopic")
		_ros.set_subscribe_topic(demand_topic)
		_ros.set_publish_topic(position_topic)
	else:
		push_warning("PenLift: no BoolTopic (not the ROS build) - pen only toggles with the key")
	await get_tree().process_frame   # let the rest of the scene get ready
	if pen_paper == null:
		pen_paper = _find(func(n): return n is PenPaper)
	if pen_line == null:
		pen_line = _find(func(n): return n is PenLine)
	if pen_line:
		_line_tip = pen_line.tip_offset
	_apply()
	_publish()


## Ask for the pen to go down (true) or up (false), same as a ROS message.
func set_demand(down: bool) -> void:
	if down != pen_down_demand:
		pen_down_demand = down
		_switch_timer = 0.0


func _process(delta: float) -> void:
	if _ros:
		_ros.spin_some()
		if _ros.has_new_data():
			set_demand(_ros.get_value())

	if pen_down_pos != pen_down_demand:
		_switch_timer += delta
		if _switch_timer >= move_time_sec:
			pen_down_pos = pen_down_demand
			_apply()
			_publish()

	_pub_timer += delta
	if _pub_timer >= publish_interval_sec:
		_publish()


func _unhandled_input(event: InputEvent) -> void:
	if toggle_key != KEY_NONE and event is InputEventKey and event.pressed \
			and not event.echo and event.keycode == toggle_key:
		set_demand(not pen_down_demand)


func _apply() -> void:
	if pen_paper:
		pen_paper.pen_down = pen_down_pos
	if pen_line:
		pen_line.tip_offset = _line_tip + (Vector3.ZERO if pen_down_pos else Vector3(0, lift_height, 0))


func _publish() -> void:
	_pub_timer = 0.0
	if _ros:
		_ros.publish(pen_down_pos)


func _find(test: Callable) -> Node:
	var stack: Array[Node] = [get_tree().root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if test.call(n):
			return n
		stack.append_array(n.get_children())
	return null
