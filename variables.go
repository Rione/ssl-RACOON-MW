package main

import (
	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
)

var BALL_MOVING_THRESHOULD_SPEED float32 = 1000

var NW_ROBOT_UPDATE_INTERFACE_NAME string = "nil"
var NW_VISION_REFEREE_INTERFACE_NAME string = "nil"

var NW_AI_IPADDR string = "127.0.0.1"      //DEPRECATED
var NW_AI_PORT string = "30011"            //DEPRECATED
var NW_AI_PORT_CONTROLLER string = "30012" //DEPRECATED
var NW_AI_PORT_GUI string = "30013"        //DEPRECATED

var NW_REF_MAX_DATAGRAM_SIZE int = 8192 * 2

var NW_OUT_MCAST_IPADDR string = "224.12.3.21"
var NW_OUT_MCAST_PORT string = "30006"

var MAX_AVAILABLE_TIMEOUTS int = 5

// グローバル宣言
// 更新時のみ置き換えるようにする
var is_detect_photo_sensor [16]bool
var is_detect_dribbler_sensor [16]bool
var is_new_dribbler [16]bool
var visionwrapper [16]*pb_gen.SSL_WrapperPacket
var visiondetection [16]*pb_gen.SSL_DetectionFrame

var geometrydata *pb_gen.SSL_GeometryData
var left_geo_goal_x float32
var left_geo_goal_y float32

var bluerobots [16]*pb_gen.SSL_DetectionRobot
var yellowrobots [16]*pb_gen.SSL_DetectionRobot

var ref_command *pb_gen.Referee
var ball *pb_gen.SSL_DetectionBall

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
var filtered_robot_theta [16]float32

var filtered_enemy_x [16]float32
var filtered_enemy_y [16]float32
var filtered_enemy_theta [16]float32

var robot_online [16]bool

var teamcolor_from_ref int = -1
var attack_direction_from_ref int = 0

var flag_ball bool = false

var is_ball_exit [16]bool
var ball_camera_X [16]float32
var ball_camera_Y [16]float32

type Adjustment struct {
	Min_Threshold         string  `json:"minThreshold"`
	Max_Threshold         string  `json:"maxThreshold"`
	Ball_Detect_Radius    int32   `json:"ballDetectRadius"`
	Circularity_Threshold float32 `json:"circularityThreshold"`
}

var adjustment [16]Adjustment
