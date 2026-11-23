extends Node
class_name PenXYLogger
## Writes the end effector's x, y (and the time) to a CSV: one row per tick.

## MATLAB:  d = readtable(".../pen_xy_....csv");  plot(d.x_m, d.y_m); axis equal

## Seconds between rows (0.02 = 50 rows a second; 0 = every physics tick).
@export var log_interval_sec: float = 0.02
## %s is replaced by the date and time.
@export var csv_path: String = "user://logs/pen_xy_%s.csv"
## Write millimetres instead of metres.
@export var millimetres: bool = false

var _logger: BaseLinkLogger
var _file: FileAccess
var _t := 0.0
var _timer := 0.0


func _ready() -> void:
	_logger = get_parent() as BaseLinkLogger
	if _logger == null:
		push_warning("PenXYLogger: put me under the logger node (BaseLinkLogger) - not logging")
		set_physics_process(false)


func _physics_process(delta: float) -> void:
	# the logger finds the pen once the robot has loaded (and again after a rebuild)
	if _logger._end_body == null or not is_instance_valid(_logger._end_body):
		return
	if _file == null:
		_open_file()
		if _file == null:
			set_physics_process(false)
			return
	else:
		_t += delta
		_timer += delta
		if _timer + 1e-9 < log_interval_sec:
			return
	_timer = 0.0
	var p: Vector3 = _logger.get_tip_position()
	var k := 1000.0 if millimetres else 1.0
	_file.store_line("%.4f,%.6f,%.6f" % [_t, p.x * k, p.y * k])
	_file.flush()   # so the rows are on disk even if the game is stopped from the editor


func _open_file() -> void:
	var path := csv_path % Time.get_datetime_string_from_system().replace(":", "-") if csv_path.contains("%s") else csv_path
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	_file = FileAccess.open(path, FileAccess.WRITE)
	if _file == null:
		push_warning("PenXYLogger: can't write %s" % path)
		return
	_file.store_line("t_s,x_mm,y_mm" if millimetres else "t_s,x_m,y_m")
	_t = 0.0
	_timer = 0.0
	print("PenXYLogger: writing ", ProjectSettings.globalize_path(path))


func _exit_tree() -> void:
	if _file:
		_file.close()
		_file = null
