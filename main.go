package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"math"
	"net"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"github.com/golang/protobuf/proto"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
)

//グローバル宣言
//更新時のみ置き換えるようにする
var balldetect [16]bool
var visionwrapper [16]*pb_gen.SSL_WrapperPacket
var visiondetection [16]*pb_gen.SSL_DetectionFrame
var geometrydata *pb_gen.SSL_GeometryData
var left_geo_goal_x float32
var left_geo_goal_y float32

var num_bluerobots int
var num_yellowrobots int

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

var enemy_slope [16]float32
var enemy_intercept [16]float32
var enemy_speed [16]float32
var enemy_angular_velocity [16]float32
var distance_ball_robot [16]float32
var radian_ball_robot [16]float32

var framecounter int
var fps float32
var secperframe float32
var isvisionrecv bool = false

// These numbers control how much smoothing the model does.
const observationNoise = 0.1 // entries for the diagonal of R_k
//const initialVariance = 0.01  // entries for the diagonal of P_0
const processVariance = 0.005 // entries for the diagonal of Q_k

func Calc_degree_normalize(rad float32) float32 {
	if rad > math.Pi {
		rad -= 2 * math.Pi
	} else if rad < -math.Pi {
		rad += 2 * math.Pi
	}
	return rad
}

func Calc_degree(x_1 float32, y_1 float32, x_2 float32, y_2 float32) float32 {
	var diff_x float64 = float64(x_1 - x_2)
	var diff_y float64 = float64(y_1 - y_2)
	var rad float32 = float32(math.Atan2(diff_y, diff_x))
	return rad
}

func Calc_distance(x_1 float32, y_1 float32, x_2 float32, y_2 float32) float32 {

	var diff_x float64 = float64(x_1 - x_2)
	var diff_y float64 = float64(y_1 - y_2)

	var dist float32 = float32(math.Sqrt((diff_x * diff_x) + (diff_y * diff_y)))

	return dist
}

func Update(chupdate chan bool) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: 40000,
	}

	serverConn, err := net.ListenMulticastUDP("udp", nil, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 1024)

	for {
		n, addr, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.Robot_Status{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)
		balldetect[packet.GetRobotId()] = packet.GetInfrared()
		log.Printf("State change signal recived from %s ", addr)
	}
}

var pre_command *pb_gen.Referee_Command
var now_command *pb_gen.Referee_Command
var last_command *pb_gen.Referee_Info_Command

func RefereeClient(chref chan bool) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.1"),
		Port: 10003,
	}

	log.Printf("Referee Client started.")
	serverConn, err := net.ListenMulticastUDP("udp", nil, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 2048)
	refcounter := 0

	for {
		n, addr, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.Referee{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)
		ref_command = packet

		now_command = packet.Command
		if pre_command == nil {
			pre_command = packet.Command
		}

		if now_command.String() != pre_command.String() {
			last_command = (*pb_gen.Referee_Info_Command)(pre_command)
		}
		pre_command = packet.Command
		if refcounter%600 == 0 {
			log.Printf("==== REFEREE OK KEEP RECEIVING FROM %s ====", addr)
			refcounter = 0
		}
		refcounter++
	}
}

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

func VisionReceive(chvision chan bool, port int, ourteam int, goalpos int, simmode bool, replay bool) {
	var pre_ball_X float32
	var pre_ball_Y float32
	var pre_robot_X [16]float32
	var pre_robot_Y [16]float32
	var pre_robot_Theta [16]float32

	var pre_enemy_X [16]float32
	var pre_enemy_Y [16]float32
	var pre_enemy_Theta [16]float32

	var t time.Time

	var modelBallX *models.SimpleModel

	modelBallX = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     1.0,
		ProcessVariance:     2,
		ObservationVariance: 2.0,
	})
	filterBallX := kalman.NewKalmanFilter(modelBallX)

	var modelBallY *models.SimpleModel

	modelBallY = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     1.0,
		ProcessVariance:     2,
		ObservationVariance: 2.0,
	})
	filterBallY := kalman.NewKalmanFilter(modelBallY)

	//f, _ := os.Create("./DEBUG2.txt")

	var pre_framecounter int = 0

	maxcameras = 0
	framecounter = 0
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: port,
	}
	log.Printf("Receiving Vision Multicast at Port %d", port)
	serverConn, _ := net.ListenMulticastUDP("udp", nil, serverAddr)
	defer serverConn.Close()

	buf := make([]byte, 2048)
	var reader *bufio.Reader
	var line []byte
	var str string
	var strarr []string
	var before_unix_time int
	var unixtime int

	if replay {
		log.Println("=====RECEIVING VIA DEBUG.TXT=====")
		f, _ := os.Open("DEBUG.txt")
		reader = bufio.NewReaderSize(f, 4096)
		defer f.Close()
	}

	for i := 0; i < 60; i++ {

		var n int
		var addr *net.UDPAddr
		var err error
		if replay {
			line, _, err = reader.ReadLine()
			CheckError(err)
			str = string(line)
			strarr = strings.Split(str, ",")
			if i == 0 {
				unixtime, _ = strconv.Atoi(strarr[0])
				before_unix_time, _ = strconv.Atoi(strarr[0])
			} else {
				before_unix_time = unixtime
				unixtime, _ = strconv.Atoi(strarr[0])
			}
			time.Sleep(time.Duration(unixtime-before_unix_time) * time.Millisecond)
		} else {
			n, addr, err = serverConn.ReadFromUDP(buf)
			CheckError(err)
		}

		packet := &pb_gen.SSL_WrapperPacket{}
		if replay {
			err = proto.UnmarshalText(strarr[1], packet)
		} else {
			err = proto.Unmarshal(buf[0:n], packet)
		}

		CheckError(err)

		if i == 0 {
			log.Printf("Vision signal reveived from %s", addr)
			log.Printf("Counting Max Cameras...")
		}

		if maxcameras < int(packet.Detection.GetCameraId())+1 {
			maxcameras = int(packet.Detection.GetCameraId()) + 1
		}

	}

	log.Printf("MAX CAMERAS: %d", maxcameras)
	log.Printf("Receive Loop and Send Start: Vision addr %s", serverAddr)

	for {
		framecounter++
		for i := 0; i < maxcameras; i++ {
			var n int
			var err error
			if replay {
				line, _, err = reader.ReadLine()
				CheckError(err)
				str = string(line)
				strarr = strings.Split(str, ",")

				before_unix_time = unixtime
				unixtime, _ = strconv.Atoi(strarr[0])
				time.Sleep(time.Duration(unixtime-before_unix_time) * time.Millisecond)
			} else {
				n, _, err = serverConn.ReadFromUDP(buf)
				CheckError(err)
			}

			packet := &pb_gen.SSL_WrapperPacket{}
			if replay {
				//log.Println(strarr[1])
				err = proto.UnmarshalText(strarr[1], packet)
			} else {
				err = proto.Unmarshal(buf[0:n], packet)
			}
			CheckError(err)

			visionwrapper[i] = packet
			visiondetection[i] = packet.Detection

			//log.Printf("Vision signal reveived from %s", addr)

			// Receive Geometry Data
			if packet.Geometry != nil { //Geometryパケットが送られていたら
				geometrydata = packet.Geometry
				var lgtlp1x, lgtlp1y, lgtlp2x float32
				var lgblp2y float32
				for _, line := range packet.Geometry.GetField().GetFieldLines() {
					if line.GetName() == "LeftGoalTopLine" {
						lgtlp1x = line.GetP1().GetX()
						lgtlp1y = line.GetP1().GetY()
					}
					if line.GetName() == "LeftGoalBottomLine" {
						lgblp2y = line.GetP2().GetY()
						lgtlp2x = line.GetP1().GetX()
					}
				}
				left_geo_goal_x = (lgtlp1x + lgtlp2x) * 0.5
				left_geo_goal_y = (lgtlp1y + lgblp2y) * 0.5

				//Invert
				if goalpos == -1 {
					left_geo_goal_x = left_geo_goal_x * -1
				}
			}

			var visible_in_vision_b [16]bool
			var visible_in_vision_y [16]bool
			num_yellowrobots = 0
			num_bluerobots = 0

			for i := 0; i < 16; i++ {
				visible_in_vision_b[i] = false
				visible_in_vision_y[i] = false
			}
			// Get Blue Robots
			for _, robot := range packet.Detection.GetRobotsBlue() {
				num_bluerobots++
				bluerobots[robot.GetRobotId()] = robot
				visible_in_vision_b[robot.GetRobotId()] = true
			}

			// Get Yellow Robots
			for _, robot := range packet.Detection.GetRobotsYellow() {
				num_yellowrobots++
				yellowrobots[robot.GetRobotId()] = robot
				visible_in_vision_y[robot.GetRobotId()] = true
			}

			for i := 0; i < 16; i++ {
				if !visible_in_vision_b[i] {
					if ourteam == 0 {
						if ourrobot_invisible_count[i] <= 15 {
							ourrobot_invisible_count[i]++
						}
					} else {
						if enemyrobot_invisible_count[i] <= 15 {
							enemyrobot_invisible_count[i]++
						}
					}
				} else {
					if ourteam == 0 {
						ourrobot_invisible_count[i] = 0
					} else {
						enemyrobot_invisible_count[i] = 0
					}
				}

				if !visible_in_vision_y[i] {
					if ourteam == 0 {
						if enemyrobot_invisible_count[i] <= 15 {
							enemyrobot_invisible_count[i]++
						}
					} else {
						if ourrobot_invisible_count[i] <= 15 {
							ourrobot_invisible_count[i]++
						}
					}
				} else {
					if ourteam == 0 {
						enemyrobot_invisible_count[i] = 0
					} else {
						ourrobot_invisible_count[i] = 0
					}
				}
			}

			// Get Most High Confidence ball
			var maxconfball *pb_gen.SSL_DetectionBall = &pb_gen.SSL_DetectionBall{
				Confidence: proto.Float32(0.0),
				X:          proto.Float32(0.0),
				Y:          proto.Float32(0.0),
				PixelX:     proto.Float32(0.0),
				PixelY:     proto.Float32(0.0),
			}
			if packet.Detection.GetBalls() != nil {
				for _, fball := range packet.Detection.GetBalls() {
					var maxconf float32 = *maxconfball.Confidence
					var conf float32 = *fball.Confidence

					if maxconf < conf {
						maxconfball = fball
					}
				}

				ball = maxconfball

			}
		}

		if framecounter-pre_framecounter > 0 {

			/////////////////////////////////////
			//
			//	KALMAN FILTER (BALL)
			//
			/////////////////////////////////////
			if ball != nil {
				t = t.Add(time.Duration(secperframe * 1000 * float32(time.Millisecond)))
				err := filterBallX.Update(t, modelBallX.NewMeasurement(float64(ball.GetX())))

				if err != nil {
					log.Println(err)
				}

				err = filterBallY.Update(t, modelBallY.NewMeasurement(float64(ball.GetY())))
				if err != nil {
					log.Println(err)
				}
				//fmt.Printf("X: before: %f, filtered value: %f\n", ball.GetX(), modelBallX.Value(filterBallX.State()))
				//fmt.Printf("Y: before: %f, filtered value: %f\n", ball.GetY(), modelBallY.Value(filterBallY.State()))
				filtered_ball_x = float32(modelBallX.Value(filterBallX.State()))
				filtered_ball_y = float32(modelBallY.Value(filterBallY.State()))
				//f.WriteString(fmt.Sprintf("%f", ball.GetX()) + "," + fmt.Sprintf("%f", modelBallX.Value(filterBallX.State())) + "\n")
			}

			/////////////////////////////////////
			//
			//	BALL SPEED CALCULATION
			//
			/////////////////////////////////////
			if ball != nil && pre_ball_X != 0 {
				var ball_X float32 = filtered_ball_x
				var ball_Y float32 = filtered_ball_y

				ball_difference_X = ball_X - pre_ball_X
				ball_difference_Y = ball_Y - pre_ball_Y

				if ball_difference_X != 0 || ball_difference_Y != 0 {
					ball_slope = ball_difference_Y / ball_difference_X
					bdX64 := float64(ball_difference_X)
					bdY64 := float64(ball_difference_Y)
					ball_slope_degree = float32(math.Atan2(bdX64, bdY64))
					ball_intercept = ball_Y - (ball_slope * ball_X)
					ball_speed = float32(math.Sqrt(math.Pow(bdX64, 2)+math.Pow(bdY64, 2)) / 0.016)
				} else {
					ball_slope_degree = 0.0
					ball_intercept = 0.0
					ball_speed = 0.0
				}

				pre_ball_X = ball.GetX()
				pre_ball_Y = ball.GetY()

			} else if ball != nil {
				pre_ball_X = ball.GetX()
				pre_ball_Y = ball.GetY()

			} else {
				time.Sleep(1 * time.Second)
				fmt.Println("Observer: Waiting Vision Receiver To Complete...")
			}

			var ourrobots [16]*pb_gen.SSL_DetectionRobot
			var enemyrobots [16]*pb_gen.SSL_DetectionRobot
			if ourteam == 0 {
				ourrobots = bluerobots
				enemyrobots = yellowrobots
			} else {
				ourrobots = yellowrobots
				enemyrobots = bluerobots
			}
			/////////////////////////////////////
			//
			//	OUR ROBOT STATUS CALCULATION
			//
			/////////////////////////////////////
			var rdX64 [16]float64
			var rdY64 [16]float64

			for _, robot := range ourrobots {
				if robot != nil {
					i := robot.GetRobotId()

					robot_difference_X[i] = pre_robot_X[i] - robot.GetX()
					robot_difference_Y[i] = pre_robot_Y[i] - robot.GetY()
					robot_difference_Theta[i] = pre_robot_Theta[i] - robot.GetOrientation()

					rdX64[i] = float64(robot_difference_X[i])
					rdY64[i] = float64(robot_difference_Y[i])

					if robot_difference_Y[i] != 0 || robot_difference_X[i] != 0 {
						robot_slope[i] = robot_difference_Y[i] / robot_difference_X[i]
						robot_intercept[i] = robot.GetY() - (robot_slope[i] * robot.GetX())
						robot_speed[i] = float32(math.Sqrt(math.Pow(rdX64[i], 2)+math.Pow(rdY64[i], 2)) / 0.016)
					} else {
						robot_slope[i] = 0.0
						robot_intercept[i] = robot.GetY()
						robot_speed[i] = 0.0
					}

					robot_angular_velocity[i] = robot_difference_Theta[i] / 0.016

					pre_robot_X[i] = robot.GetX()
					pre_robot_Y[i] = robot.GetY()
					pre_robot_Theta[i] = robot.GetOrientation()

					radian_ball_robot[i] = Calc_degree_normalize(Calc_degree(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY()) - robot.GetOrientation())
					distance_ball_robot[i] = Calc_distance(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY())

				}
			}

			/////////////////////////////////////
			//
			//	ENEMY ROBOT STATUS CALCULATION
			//
			/////////////////////////////////////
			var edX64 [16]float64
			var edY64 [16]float64

			for _, enemy := range enemyrobots {
				if enemy != nil {
					i := enemy.GetRobotId()

					enemy_difference_X[i] = pre_enemy_X[i] - enemy.GetX()
					enemy_difference_Y[i] = pre_enemy_Y[i] - enemy.GetY()
					enemy_difference_Theta[i] = pre_enemy_Theta[i] - enemy.GetOrientation()

					edX64[i] = float64(enemy_difference_X[i])
					edY64[i] = float64(enemy_difference_Y[i])

					if enemy_difference_Y[i] != 0 || enemy_difference_X[i] != 0 {
						enemy_slope[i] = enemy_difference_Y[i] / enemy_difference_X[i]
						enemy_intercept[i] = enemy.GetY() - (enemy_slope[i] * enemy.GetX())
						enemy_speed[i] = float32(math.Sqrt(math.Pow(edX64[i], 2)+math.Pow(edY64[i], 2)) / 0.016)
					} else {
						enemy_slope[i] = 0.0
						enemy_intercept[i] = enemy.GetY()
						enemy_speed[i] = 0.0
					}

					enemy_angular_velocity[i] = enemy_difference_Theta[i] / 0.016

					pre_enemy_X[i] = enemy.GetX()
					pre_enemy_Y[i] = enemy.GetY()
					pre_enemy_Theta[i] = enemy.GetOrientation()
				}
			}

			/////////////////////////////////////
			//
			//	BALL STATUS CALCULATION
			//  FOR SIMULATOR MODE
			//
			/////////////////////////////////////
			if simmode {
				for i := 0; i < 16; i++ {
					if distance_ball_robot[i]/1000 < 0.115 && radian_ball_robot[i]*180/math.Pi < 10 && radian_ball_robot[i]*180/math.Pi > -10 {
						balldetect[i] = true
					} else {
						balldetect[i] = false
					}
				}
			}
		}
		pre_framecounter = framecounter

	}
	chvision <- true
}

func FPSCounter(chfps chan bool) {
	for {
		if framecounter != 0 {
			isvisionrecv = true
			fps = float32(framecounter)

			framecounter = 0

			secperframe = 1 / float32(fps)

			log.Printf("Estimated FPS:  %f FPS, Interval %f ms", fps, secperframe)
		} else {
			isvisionrecv = false
			secperframe = 100000
		}

		time.Sleep(1 * time.Second)
	}
}

func CheckVisionRobot(chvisrobot chan bool) {
	for {
		if isvisionrecv {
			for i := 0; i < 16; i++ {
				if ourrobot_invisible_count[i] <= 10 {
					ourrobot_is_visible[i] = true
				} else {
					ourrobot_is_visible[i] = false
				}

				if enemyrobot_invisible_count[i] <= 10 {
					enemyrobot_is_visible[i] = true
				} else {
					enemyrobot_is_visible[i] = false
				}
			}
		}
		time.Sleep(1 * time.Second)
	}
}

func createRobotInfo(i int, ourteam int, simmode bool) *pb_gen.Robot_Infos {
	var robotid uint32 = bluerobots[i].GetRobotId()
	var x float32 = bluerobots[i].GetX()
	var y float32 = bluerobots[i].GetY()
	var theta float32 = bluerobots[i].GetOrientation()
	var diffx float32 = robot_difference_X[i]
	var diffy float32 = robot_difference_Y[i]
	var difftheta float32 = robot_difference_Theta[i]

	if ourteam == 1 {
		robotid = yellowrobots[i].GetRobotId()
		x = yellowrobots[i].GetX()
		y = yellowrobots[i].GetY()
		theta = yellowrobots[i].GetOrientation()
	}

	var batt float32 = 12.15
	var online bool = true
	pe := &pb_gen.Robot_Infos{
		RobotId:           &robotid,
		X:                 &x,
		Y:                 &y,
		Theta:             &theta,
		DiffX:             &diffx,
		DiffY:             &diffy,
		DiffTheta:         &difftheta,
		DistanceBallRobot: &distance_ball_robot[i],
		RadianBallRobot:   &radian_ball_robot[i],
		Speed:             &robot_speed[i],
		Slope:             &robot_slope[i],
		Intercept:         &robot_intercept[i],
		AngularVelocity:   &robot_angular_velocity[i],
		BallCatch:         &balldetect[i],
		Online:            &online,
		Visible:           &ourrobot_is_visible[i],
		BatteryVoltage:    &batt,
	}
	return pe
}

func addRobotInfoToRobotInfos(robotinfo [16]*pb_gen.Robot_Infos) []*pb_gen.Robot_Infos {
	RobotInfos := []*pb_gen.Robot_Infos{}

	for _, robot := range robotinfo {
		if robot != nil {
			RobotInfos = append(RobotInfos, robot)
		}
	}

	return RobotInfos
}

func createEnemyInfo(i int, ourteam int) *pb_gen.Robot_Infos {
	if enemyrobot_is_visible[i] {
		var robotid uint32 = yellowrobots[i].GetRobotId()
		var x float32 = yellowrobots[i].GetX()
		var y float32 = yellowrobots[i].GetY()
		var theta float32 = yellowrobots[i].GetOrientation()
		var diffx float32 = enemy_difference_X[i]
		var diffy float32 = enemy_difference_Y[i]
		var difftheta float32 = enemy_difference_Theta[i]

		if ourteam == 1 {
			robotid = bluerobots[i].GetRobotId()
			x = bluerobots[i].GetX()
			y = bluerobots[i].GetY()
			theta = bluerobots[i].GetOrientation()
		}

		pe := &pb_gen.Robot_Infos{
			RobotId:         &robotid,
			X:               &x,
			Y:               &y,
			DiffX:           &diffx,
			DiffY:           &diffy,
			DiffTheta:       &difftheta,
			Theta:           &theta,
			Speed:           &enemy_speed[i],
			Slope:           &enemy_slope[i],
			Intercept:       &enemy_intercept[i],
			AngularVelocity: &enemy_angular_velocity[i],
			Visible:         &enemyrobot_is_visible[i],
		}
		return pe
	} else {
		return nil
	}
}

func createBallInfo() *pb_gen.Ball_Info {
	var x float32 = ball.GetX()
	var y float32 = ball.GetY()
	var z float32 = ball.GetZ()
	var sloperadian float32 = ball_slope_degree
	var slope float32 = ball_slope
	var intercept float32 = ball_intercept
	var speed float32 = ball_speed
	var diffx float32 = ball_difference_X
	var diffy float32 = ball_difference_Y
	pe := &pb_gen.Ball_Info{
		FilteredX:   &filtered_ball_x,
		FilteredY:   &filtered_ball_y,
		X:           &x,
		Y:           &y,
		Z:           &z,
		DiffX:       &diffx,
		DiffY:       &diffy,
		SlopeRadian: &sloperadian,
		Intercept:   &intercept,
		Speed:       &speed,
		Slope:       &slope,
	}
	return pe
}

func createGeometryInfo() *pb_gen.Geometry_Info {
	var x float32 = left_geo_goal_x
	var y float32 = left_geo_goal_y
	var FieldLength int32
	var FieldWidth int32
	var GoalWidth int32
	var GoalDepth int32
	var BoundaryWidth int32
	var PenaltyAreaWidth int32
	var PenaltyAreaDepth int32
	var CenterCircleRadius int32
	var LineThickness int32
	var GoalCenterToPenaltyMark int32
	var GoalHeight int32
	var BallRadius float32
	var MaxRobotRadius float32

	if geometrydata != nil {
		FieldLength = geometrydata.Field.GetFieldLength()
		FieldWidth = geometrydata.Field.GetFieldWidth()
		GoalWidth = geometrydata.Field.GetGoalWidth()
		GoalDepth = geometrydata.Field.GetGoalDepth()
		BoundaryWidth = geometrydata.Field.GetBoundaryWidth()
		PenaltyAreaWidth = geometrydata.Field.GetPenaltyAreaWidth()
		PenaltyAreaDepth = geometrydata.Field.GetPenaltyAreaDepth()
		CenterCircleRadius = geometrydata.Field.GetCenterCircleRadius()
		LineThickness = geometrydata.Field.GetLineThickness()
		GoalCenterToPenaltyMark = geometrydata.Field.GetGoalCenterToPenaltyMark()
		GoalHeight = geometrydata.Field.GetGoalHeight()
		BallRadius = geometrydata.Field.GetBallRadius()
		MaxRobotRadius = geometrydata.Field.GetMaxRobotRadius()
	}

	pe := &pb_gen.Geometry_Info{
		FieldLength:             &FieldLength,
		FieldWidth:              &FieldWidth,
		GoalWidth:               &GoalWidth,
		GoalDepth:               &GoalDepth,
		BoundaryWidth:           &BoundaryWidth,
		PenaltyAreaWidth:        &PenaltyAreaWidth,
		PenaltyAreaDepth:        &PenaltyAreaDepth,
		CenterCircleRadius:      &CenterCircleRadius,
		LineThickness:           &LineThickness,
		GoalCenterToPenaltyMark: &GoalCenterToPenaltyMark,
		GoalHeight:              &GoalHeight,
		BallRadius:              &BallRadius,
		MaxRobotRadius:          &MaxRobotRadius,
		GoalX:                   &x,
		GoalY:                   &y,
	}

	return pe
}

func createOtherInfo(goalpos_n int32) *pb_gen.Other_Infos {
	var numofcameras int32 = int32(maxcameras)
	var numofourrobots int32
	var numofenemyrobots int32
	for i := 0; i < 16; i++ {
		if ourrobot_is_visible[i] {
			numofourrobots++
		}
		if enemyrobot_is_visible[i] {
			numofenemyrobots++
		}
	}

	pe := &pb_gen.Other_Infos{
		NumOfCameras:     &numofcameras,
		NumOfOurRobots:   &numofourrobots,
		NumOfEnemyRobots: &numofenemyrobots,
		Secperframe:      &secperframe,
		IsVisionRecv:     &isvisionrecv,
		AttackDirection:  &goalpos_n,
	}
	return pe
}

func createRefInfo(ourteam int) *pb_gen.Referee_Info {
	var yellowcards uint32
	var redcards uint32
	var command *pb_gen.Referee_Info_Command
	var stage *pb_gen.Referee_Info_Stage
	var next_command *pb_gen.Referee_Info_Command
	var bpX float32
	var bpY float32

	if ref_command != nil {
		command = (*pb_gen.Referee_Info_Command)(ref_command.Command)
		stage = (*pb_gen.Referee_Info_Stage)(ref_command.Stage)
		next_command = (*pb_gen.Referee_Info_Command)(ref_command.NextCommand)
		bpX = ref_command.GetDesignatedPosition().GetX()
		bpY = ref_command.GetDesignatedPosition().GetY()
		if ourteam == 0 {
			yellowcards = ref_command.Blue.GetYellowCards()
			redcards = ref_command.Blue.GetRedCards()
		} else {
			yellowcards = ref_command.Yellow.GetYellowCards()
			redcards = ref_command.Blue.GetRedCards()
		}

	} else {
		yellowcards = 0
		redcards = 0
	}

	pe := &pb_gen.Referee_Info{
		Command:        command,
		Stage:          stage,
		YellowCards:    &yellowcards,
		RedCards:       &redcards,
		PreCommand:     last_command,
		NextCommand:    next_command,
		BallPlacementX: &bpX,
		BallPlacementY: &bpY,
	}
	return pe
}

func addEnemyInfoToEnemyInfos(enemyinfo [16]*pb_gen.Robot_Infos) []*pb_gen.Robot_Infos {
	EnemyInfos := []*pb_gen.Robot_Infos{}

	for _, enemy := range enemyinfo {
		if enemy != nil {
			EnemyInfos = append(EnemyInfos, enemy)
		}
	}

	return EnemyInfos
}

func RunServer(chserver chan bool, reportrate uint, ourteam int, goalpose int, debug bool, simmode bool) {
	ipv4 := "127.0.0.1"
	port := "30011"
	addr := ipv4 + ":" + port

	log.Println("Send to:", addr)

	conn, err := net.Dial("udp", addr)
	CheckError(err)
	defer conn.Close()

	var counter int

	for {

		var robot_infos [16]*pb_gen.Robot_Infos
		var enemy_infos [16]*pb_gen.Robot_Infos

		if ourteam == 0 {
			for _, robot := range bluerobots {
				robot_infos[robot.GetRobotId()] = createRobotInfo(int(robot.GetRobotId()), ourteam, simmode)
			}
			for _, enemy := range yellowrobots {
				enemy_infos[enemy.GetRobotId()] = createEnemyInfo(int(enemy.GetRobotId()), ourteam)
			}
		} else {
			for _, robot := range yellowrobots {
				robot_infos[robot.GetRobotId()] = createRobotInfo(int(robot.GetRobotId()), ourteam, simmode)
			}
			for _, enemy := range bluerobots {
				enemy_infos[enemy.GetRobotId()] = createEnemyInfo(int(enemy.GetRobotId()), ourteam)
			}
		}

		RobotInfos := addRobotInfoToRobotInfos(robot_infos)

		EnemyInfos := addEnemyInfoToEnemyInfos(enemy_infos)

		BallInfo := createBallInfo()

		GeometryInfo := createGeometryInfo()
		RefereeInfo := createRefInfo(ourteam)
		OtherInfo := createOtherInfo(int32(goalpose))

		//log.Println(OtherInfo.GetAttackDirection())

		RacoonMWPacket := &pb_gen.RacoonMW_Packet{
			OurRobots:   RobotInfos,
			EnemyRobots: EnemyInfos,
			Geometry:    GeometryInfo,
			Ball:        BallInfo,
			Referee:     RefereeInfo,
			Info:        OtherInfo,
		}

		if debug {
			fmt.Println(RacoonMWPacket)
		}

		Data, _ := proto.Marshal(RacoonMWPacket)
		if isvisionrecv && geometrydata != nil {
			conn.Write([]byte(Data))
		}

		time.Sleep(time.Duration(reportrate) * time.Millisecond)
		counter = counter + 1

	}
	chserver <- true
}

func RunRecorder(chrecorder chan bool, port int) {

	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: port,
	}
	serverConn, err := net.ListenMulticastUDP("udp", nil, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 2048)

	f, _ := os.Create("./DEBUG.txt")

	for {
		n, _, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.SSL_WrapperPacket{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)

		f.WriteString(strconv.Itoa(int(time.Now().UnixMilli())) + "," + packet.String() + "\n")
	}

	chrecorder <- true
}

func main() {

	var (
		visionport = flag.Int("p", 10006, "Vision Multicast Port Number")
		ourteam    = flag.String("t", "blue", "Our Team (blue or yellow)")
		goalpos    = flag.String("g", "N", "Attack Direction Negative or Positive (N or P)")
		reportrate = flag.Uint("r", 16, "How often report to RACOON-AI? (milliseconds)")
		debug      = flag.Bool("d", false, "Show All Send Packet")
		simmode    = flag.Bool("s", false, "Simulation Mode (Emulate Ball Sensor)")
		replay     = flag.Bool("replay", false, "Replay All Packet")
	)

	//OUR TEAM 0 = blue
	//OUR TEAM 1 = yellow

	//GOAL POSE 0 = Negative
	//Goal POSE 1 = Positive

	flag.Parse()

	var ourteam_n int
	if *ourteam == "blue" {
		ourteam_n = 0
	} else {
		ourteam_n = 1
	}

	var goalpos_n int
	if *goalpos == "N" {
		goalpos_n = -1
	} else {
		goalpos_n = 1
	}

	chupdate := make(chan bool)
	chserver := make(chan bool)
	chvision := make(chan bool)
	chref := make(chan bool)
	chfps := make(chan bool)
	chvisrobot := make(chan bool)

	go Update(chupdate)
	go RunServer(chserver, *reportrate, ourteam_n, goalpos_n, *debug, *simmode)
	go VisionReceive(chvision, *visionport, ourteam_n, goalpos_n, *simmode, *replay)
	go CheckVisionRobot(chvisrobot)
	go FPSCounter(chfps)
	go RefereeClient(chref)

	if *debug {
		chrecorder := make(chan bool)
		go RunRecorder(chrecorder, *visionport)
		<-chrecorder
	}

	<-chupdate
	<-chserver
	<-chvision
	<-chref

}

func CheckError(err error) {
	if err != nil {
		log.Println("Error: ", err)
	}
}
