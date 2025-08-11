extends RigidBody3D

@export var debug_print_interval = 0.5 # Print debug info every 0.5 seconds
var debug_timer = 0.0

@export_group("Altitude PID")
@export var target_altitude = 5.0
@export var altitude_kp = 40.0
@export var altitude_ki = 3.0
@export var altitude_kd = 25.0

@export_group("Angle Control PID (Roll)")
@export var max_tilt_angle_deg = 15.0
@export var angle_kp = 2.0   
@export var angle_ki = 0.8  
@export var angle_kd = 1.5    

# --- STATE VARIABLES ---
var altitude_integral = 0.0
var altitude_error_last = 0.0
var roll_integral = 0.0
var max_tilt_angle_rad: float

func _ready():
	max_tilt_angle_rad = deg_to_rad(max_tilt_angle_deg)

func _physics_process(delta):
	# --- 1. GET INPUT AND CURRENT STATE ---
	var altitude_input = Input.get_action_strength("ui_up") - Input.get_action_strength("ui_down")
	var roll_input = Input.get_action_strength("ui_right") - Input.get_action_strength("ui_left")
	
	# --- FIX 1: CORRECTLY MEASURE ROLL ANGLE ---
	# We measure how much the drone's right-vector (basis.x) is tilted up/down (its .y component).
	var current_roll_rad = -transform.basis.x.y
	
	# --- FIX 2: CORRECTLY MEASURE ROLL VELOCITY ---
	# Roll is rotation around the local Z-axis.
	var current_roll_velocity = angular_velocity.z 
	
	# --- 2. DETERMINE THE TARGETS ---
	target_altitude += altitude_input * 2.0 * delta
	var target_roll_rad = roll_input * max_tilt_angle_rad
	
	# --- 3. CALCULATE FORCES ---
	var upward_thrust = calculate_altitude_pid(delta)
	var roll_force_differential = calculate_roll_force(target_roll_rad, current_roll_rad, current_roll_velocity, delta)
	
	# --- 4. DISTRIBUTE FORCES TO MOTORS ---
	var base_motor_force = upward_thrust / 4.0
	
	var force_fl = base_motor_force + roll_force_differential
	var force_fr = base_motor_force - roll_force_differential
	var force_rl = base_motor_force + roll_force_differential
	var force_rr = base_motor_force - roll_force_differential
	
	# --- TIMED DEBUG PRINTING ---
	debug_timer += delta
	if debug_timer >= debug_print_interval:
		debug_timer = 0.0 # Reset the timer
		
		print("--- Frame State ---")
		print("Target Roll: %.3f | Current Roll: %.3f" % [target_roll_rad, current_roll_rad])
		
		# NEW PRINT STATEMENT FOR MOTOR FORCES
		print("Motor Forces | FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f" % [force_fl, force_fr, force_rl, force_rr])
		print("Base Thrust: %.2f, Roll Differential: %.2f" % [upward_thrust, roll_force_differential])
		print("--------------------")
	
	# --- 5. APPLY FORCES AT MOTOR POSITIONS ---
	# Note: Rolling left/right applies torque around the Z-axis.
	apply_force(transform.basis.y * force_fl, $MotorFrontLeft.position)
	apply_force(transform.basis.y * force_fr, $MotorFrontRight.position)
	apply_force(transform.basis.y * force_rl, $MotorRearLeft.position)
	apply_force(transform.basis.y * force_rr, $MotorRearRight.position)


func calculate_altitude_pid(delta: float) -> float:
	var error = target_altitude - global_position.y
	altitude_integral = clamp(altitude_integral + error * delta, -10, 10)
	var derivative = (error - altitude_error_last) / delta
	altitude_error_last = error
	
	var gravity_force = mass * ProjectSettings.get_setting("physics/3d/default_gravity")
	var pid_output = (altitude_kp * error) + (altitude_ki * altitude_integral) + (altitude_kd * derivative)
	return gravity_force + pid_output


func calculate_roll_force(target_angle: float, current_angle: float, current_velocity: float, delta: float) -> float:
	var error = target_angle - current_angle
	
	# Proportional
	var p_force = angle_kp * error
	
	# Integral
	roll_integral = clamp(roll_integral + error * delta, -5, 5)
	var i_force = angle_ki * roll_integral

	# Derivative (Damping)
	var d_force = angle_kd * current_velocity
	
	return p_force + i_force - d_force
