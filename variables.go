package main

import (
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
)

var BALL_MOVING_THRESHOULD_SPEED float32 = 1000

var NW_ROBOT_UPDATE_INTERFACE_NAME string = "nil"
var NW_VISION_REFEREE_INTERFACE_NAME string = "nil"
var NW_AI_IPADDR string = "127.0.0.1"
var NW_AI_PORT string = "30011"
var NW_AI_PORT_CONTROLLER string = "30012"
var NW_REF_MAX_DATAGRAM_SIZE int = 8192 * 2

var IMU_RESET_INTERVAL time.Duration = 5000 * time.Millisecond

var MAX_AVAILABLE_TIMEOUTS int = 5

// グローバル宣言
// 更新時のみ置き換えるようにする
var balldetect [16]bool
var visionwrapper [16]*pb_gen.SSL_WrapperPacket
var visiondetection [16]*pb_gen.SSL_DetectionFrame

var trackerwrapper [16]*pb_gen.TrackerWrapperPacket
var trackerdetection [16]*pb_gen.TrackedFrame

var geometrydata *pb_gen.SSL_GeometryData
var left_geo_goal_x float32
var left_geo_goal_y float32

var num_bluerobots int
var num_yellowrobots int

var bluerobots [16]*pb_gen.SSL_DetectionRobot
var yellowrobots [16]*pb_gen.SSL_DetectionRobot

var trackedblue [16]*pb_gen.TrackedRobot
var trackedyellow [16]*pb_gen.TrackedRobot

var ref_command *pb_gen.Referee
var ball *pb_gen.SSL_DetectionBall

var trackedball *pb_gen.TrackedBall

var only_use_tracker bool = false

var maxcameras int

var filtered_ball_x float32
var filtered_ball_y float32

var ball_slope_degree float32
var ball_intercept float32
var ball_speed float32
var ball_slope float32

var robot_slope [16]float32
var robot_intercept [16]float32
var robot_speed [16]float32
var robot_angular_velocity [16]float32

var battery_voltage [16]float32

var enemy_slope [16]float32
var enemy_intercept [16]float32
var enemy_speed [16]float32
var enemy_angular_velocity [16]float32
var distance_ball_robot [16]float32
var radian_ball_robot [16]float32

var framecounter int
var fps int
var secperframe float32
var isvisionrecv bool = false

var robot_online_count [16]uint8
var centercircleradius float32
var robot_ipaddr [16]string

var pre_command *pb_gen.Referee_Command
var now_command *pb_gen.Referee_Command
var last_command *pb_gen.Referee_Info_Command

var ourrobot_invisible_count [16]int
var ourrobot_is_visible [16]bool

var enemyrobot_invisible_count [16]int
var enemyrobot_is_visible [16]bool

var robot_difference_X [16]float32
var robot_difference_Y [16]float32
var robot_difference_Theta [16]float32

var enemy_difference_X [16]float32
var enemy_difference_Y [16]float32
var enemy_difference_Theta [16]float32

var ball_difference_X float32
var ball_difference_Y float32

var is_ball_moving bool

var filtered_robot_x [16]float32
var filtered_robot_y [16]float32

var ball_x_history []float32
var ball_y_history []float32

var imu_reset_time time.Time

var robot_online [16]bool
