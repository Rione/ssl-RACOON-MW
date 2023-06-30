package main

import (
	"bufio"
	"flag"
	"fmt"
	"log"
	"math"
	"net"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"google.golang.org/protobuf/proto"
)

var BALL_MOVING_THRESHOULD_SPEED float32 = 1000

var NW_ROBOT_UPDATE_INTERFACE_NAME string = "nil"
var NW_VISION_REFEREE_INTERFACE_NAME string = "nil"
var NW_AI_IPADDR string = "127.0.0.1"
var NW_AI_PORT string = "30011"
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

var robot_online_count [16]uint8
var centercircleradius float32
var robot_ipaddr [16]string

func Update(chupdate chan bool) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.69.4"),
		Port: 16941,
	}
	interfacename, _ := net.InterfaceByName(NW_ROBOT_UPDATE_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Robot Update Signal NW Interface Name is wrong! Trying system-default interface!")
	}
	serverConn, err := net.ListenMulticastUDP("udp", interfacename, serverAddr)
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
		if robot_online_count[packet.GetRobotId()] < 5 {
			robot_online_count[packet.GetRobotId()] += 1
		}
		//ロボットIDとIPアドレスの対応付け
		if robot_ipaddr[packet.GetRobotId()] != addr.IP.String(){
			robot_ipaddr[packet.GetRobotId()] = addr.IP.String()
			log.Println("Robot ID", packet.GetRobotId(), " is associated with ", addr.IP.String())
		}

		// log.Printf("State change signal recived from %s ", addr)
	}
}

// RobotのIPのリストをWebでホストする
func RobotIPList(chrobotip chan bool) {
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		//ロボットのIPアドレスを表形式で表示する
		//H1タグでタイトルを表示
		fmt.Fprintf(w, "<h1>The RACOON Web Console</h1>")
		fmt.Fprintf(w, "<html><head><title>The RACOON Web Console</title></head><body><table border=\"1\">")
		fmt.Fprintf(w, "<h2>Robot IP List</h2><tr><th>Robot ID</th><th>Associated IP Address</th><th>Beep</th></tr>")
		for i := 0; i < 16; i++ {
			buzzurl := fmt.Sprintf("location.href=\"http://%s:9191/buzzer/tone/%s/1000\"", robot_ipaddr[i], strconv.Itoa(i))
			fmt.Fprintf(w, "<tr><td>%d</td><td>%s</td><td><button onclick='%s'>Beep</button></td></tr>", i, robot_ipaddr[i], buzzurl)
		}
		fmt.Fprintf(w, "</table>")
		//Date and Time
		//Vision Status
		fmt.Fprintf(w, "<h2>Vision Status: %t</h2>", isvisionrecv)
		fmt.Fprintf(w, "<p>Generated at %s</p>", time.Now())
		fmt.Fprintf(w, "</body></html>")

	})
	log.Fatal(http.ListenAndServe(":8080", nil))

	<-chrobotip
}

var pre_command *pb_gen.Referee_Command
var now_command *pb_gen.Referee_Command
var last_command *pb_gen.Referee_Info_Command

func RefereeClient(chref chan bool) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.1"),
		Port: 10003,
	}
	interfacename, _ := net.InterfaceByName(NW_VISION_REFEREE_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Referee Signal NW Interface Name is wrong! Trying system-default interface!")
	}

	log.Printf("Referee Client started.")
	serverConn, err := net.ListenMulticastUDP("udp", interfacename, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, NW_REF_MAX_DATAGRAM_SIZE)
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

var is_ball_moving bool

var filtered_robot_x [16]float32
var filtered_robot_y [16]float32

var ball_x_history []float32
var ball_y_history []float32

func VisionReceive(chvision chan bool, port int, ourteam int, goalpos int, simmode bool, replay bool, halfswitch_n int) {
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
	var modelBallY *models.SimpleModel

	modelBallX = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     1.0,
		ProcessVariance:     2,
		ObservationVariance: 2.0,
	})
	filterBallX := kalman.NewKalmanFilter(modelBallX)
	//KalmanSmoother
	// smoothedBallX := kalman.NewKalmanSmoother(modelBallX)

	modelBallY = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     1.0,
		ProcessVariance:     2,
		ObservationVariance: 2.0,
	})
	filterBallY := kalman.NewKalmanFilter(modelBallY)
	// smoothedBallY := kalman.NewKalmanSmoother(modelBallX)

	var modelRobotX [16]*models.SimpleModel
	var modelRobotY [16]*models.SimpleModel

	var filterRobotX [16]*kalman.KalmanFilter
	var filterRobotY [16]*kalman.KalmanFilter

	for i := 0; i < 16; i++ {
		modelRobotX[i] = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
			InitialVariance:     1.0,
			ProcessVariance:     2,
			ObservationVariance: 2.0,
		})
		filterRobotX[i] = kalman.NewKalmanFilter(modelRobotX[i])

		modelRobotY[i] = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
			InitialVariance:     1.0,
			ProcessVariance:     2,
			ObservationVariance: 2.0,
		})
		filterRobotY[i] = kalman.NewKalmanFilter(modelRobotY[i])
	}

	//f, _ := os.Create("./DEBUG2.txt")

	var pre_framecounter int = 0

	maxcameras = 0
	framecounter = 0
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: port,
	}

	interfacename, _ := net.InterfaceByName(NW_VISION_REFEREE_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Vision Signal NW Interface Name is wrong! Trying system-default interface!")
	}

	log.Printf("Receiving Vision Multicast at Port %d", port)
	serverConn, _ := net.ListenMulticastUDP("udp", interfacename, serverAddr)
	defer serverConn.Close()

	buf := make([]byte, 4096)
	var reader *bufio.Reader
	var line []byte
	var str string
	var strarr []string
	var before_unix_time int
	var unixtime int

	// open robot_speed_file
	// robot_speed_file, err := os.OpenFile("./test.txt", os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0644)
	// if err != nil {
	// 	log.Fatal(err)
	// }

	for i := 0; i < 60; i++ {

		var n int
		var addr *net.UDPAddr
		var err error
		n, addr, err = serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.SSL_WrapperPacket{}
		err = proto.Unmarshal(buf[0:n], packet)

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
				// Geometry を受け取ったのちは、Visionを受け取らない
				n, _, err = serverConn.ReadFromUDP(buf)
				CheckError(err)
			}

			packet := &pb_gen.SSL_WrapperPacket{}

			err = proto.Unmarshal(buf[0:n], packet)
			CheckError(err)

			visionwrapper[i] = packet
			visiondetection[i] = packet.Detection

			//log.Printf("Vision signal reveived from %s", addr)

			// Receive Geometry Data
			if packet.Geometry != nil { //Geometryパケットが送られていたら
				geometrydata = packet.Geometry
				//log.Println(geometrydata)
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

				//WARNING
				// [0] may be not centercircle

				centercircleradius = packet.Geometry.GetField().GetFieldArcs()[0].GetRadius()

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
				switch halfswitch_n {
				case 0:
					num_bluerobots++
					bluerobots[robot.GetRobotId()] = robot
					visible_in_vision_b[robot.GetRobotId()] = true

				case 1:
					if robot.GetX() > 0 {
						num_bluerobots++
						bluerobots[robot.GetRobotId()] = robot
						visible_in_vision_b[robot.GetRobotId()] = true
					}

				case -1:
					if robot.GetX() <= 0 {
						num_bluerobots++
						bluerobots[robot.GetRobotId()] = robot
						visible_in_vision_b[robot.GetRobotId()] = true
					}
				}
			}

			// Get Yellow Robots
			for _, robot := range packet.Detection.GetRobotsYellow() {
				switch halfswitch_n {
				case 0:
					num_yellowrobots++
					yellowrobots[robot.GetRobotId()] = robot
					visible_in_vision_y[robot.GetRobotId()] = true

				case 1:
					if robot.GetX() > 0 {
						num_yellowrobots++
						yellowrobots[robot.GetRobotId()] = robot
						visible_in_vision_y[robot.GetRobotId()] = true
					}

				case -1:
					if robot.GetX() <= 0 {
						num_yellowrobots++
						yellowrobots[robot.GetRobotId()] = robot
						visible_in_vision_y[robot.GetRobotId()] = true
					}
				}
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
			var is_ball_exists bool = false
			if packet.Detection.GetBalls() != nil {
				var usethisball bool
				for _, fball := range packet.Detection.GetBalls() {
					usethisball = true
					if halfswitch_n == 1 {
						if fball.GetX() < 0 {
							usethisball = false
						}
					} else if halfswitch_n == -1 {
						if fball.GetX() >= 0 {
							usethisball = false
						}
					}
					if usethisball {
						var maxconf float32 = *maxconfball.Confidence
						var conf float32 = *fball.Confidence

						if maxconf < conf {
							maxconfball = fball
						}
						is_ball_exists = true
					}
				}

				if is_ball_exists {
					ball = maxconfball
				}
			}

		}
		framecounter++

		if framecounter-pre_framecounter > 0 {

			/////////////////////////////////////
			//
			//	KALMAN FILTER (BALL)
			//
			/////////////////////////////////////
			if ball != nil {
				//if ball_x_history is too long, remove first element
				if len(ball_x_history) > 5 {
					ball_x_history = ball_x_history[1:]
				}
				//append ball x history
				ball_x_history = append(ball_x_history, ball.GetX())

				t = t.Add(time.Duration(secperframe * 1000 * float32(time.Millisecond)))
				err := filterBallX.Update(t, modelBallX.NewMeasurement(float64(ball.GetX())))
				if err != nil {
					log.Println(err)
				}
				mm_x := make([]*kalman.MeasurementAtTime, len(ball_x_history))
				for i, v := range ball_x_history {
					mm_x[i] = kalman.NewMeasurementAtTime(t, modelBallX.NewMeasurement(float64(v)))
				}
				smoothBallX_states, err := kalman.NewKalmanSmoother(modelBallX).Smooth(mm_x...)

				//if ball_x_history is too long, remove first element
				if len(ball_y_history) > 5 {
					ball_y_history = ball_y_history[1:]
				}
				//append ball x history
				ball_y_history = append(ball_y_history, ball.GetY())

				err = filterBallY.Update(t, modelBallY.NewMeasurement(float64(ball.GetY())))
				if err != nil {
					log.Println(err)
				}

				mm_y := make([]*kalman.MeasurementAtTime, len(ball_x_history))
				for i, v := range ball_y_history {
					mm_y[i] = kalman.NewMeasurementAtTime(t, modelBallY.NewMeasurement(float64(v)))
				}
				// smoothBallY_states, err := kalman.NewKalmanSmoother(modelBallY).Smooth(mm_y...)

				//smoothBallX_states の 4番目のsを取得する modelBallX.Value(smoothBallX_states[3].State)

				if len(smoothBallX_states) >= 4 {
					// robot_speed_fileに書き込み
					// log.Println("smoothBallX_states", smoothBallX_states)
					// robot_speed_file.WriteString(fmt.Sprintf("%f %f %f\n", ball.GetX(), modelBallX.Value(filterBallX.State()), modelBallX.Value(smoothBallX_states[1].State)))
				}

				// log.Println("smoothBallX_states", smoothBallX_states)
				// log.Println("smoothBallY_states", smoothBallY_states)
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
					ball_slope_degree = float32(math.Atan2(bdY64, bdX64))
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
				fmt.Println("[FATAL] Ball is not detected. Please place the ball in the field and restart MW.")
			}

			/////////////////////////////////////
			//
			//	BALL MOVING DIRECTION CALCULATION
			//
			/////////////////////////////////////

			// check if ball is moving
			if ball_speed > BALL_MOVING_THRESHOULD_SPEED {
				is_ball_moving = true
			} else {
				is_ball_moving = false
			}
			//log.Println("ball_speed ", ball_speed)

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

					//Kalman Filter
					err := filterRobotX[i].Update(t, modelRobotX[i].NewMeasurement(float64(robot.GetX())))
					if err != nil {
						log.Println(err)
					}
					err = filterRobotY[i].Update(t, modelRobotY[i].NewMeasurement(float64(robot.GetY())))
					if err != nil {
						log.Println(err)
					}

					filtered_robot_x[i] = float32(modelRobotX[i].Value(filterRobotX[i].State()))
					filtered_robot_y[i] = float32(modelRobotY[i].Value(filterRobotY[i].State()))

					robot_difference_X[i] = filtered_robot_x[i] - pre_robot_X[i]
					robot_difference_Y[i] = filtered_robot_y[i] - pre_robot_Y[i]
					robot_difference_Theta[i] = robot.GetOrientation() - pre_robot_Theta[i]

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

					pre_robot_X[i] = filtered_robot_x[i]
					pre_robot_Y[i] = filtered_robot_y[i]
					pre_robot_Theta[i] = robot.GetOrientation()

					radian_ball_robot[i] = Calc_degree_normalize(Calc_degree(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY()) - robot.GetOrientation())
					distance_ball_robot[i] = Calc_distance(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY())

				}
				//Print robot speed to text file
				// fmt.Fprintf(robot_speed_file, "%f\n", robot_speed[6])
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

					enemy_difference_X[i] = enemy.GetX() - pre_enemy_X[i]
					enemy_difference_Y[i] = enemy.GetY() - pre_enemy_Y[i]
					enemy_difference_Theta[i] = enemy.GetOrientation() - pre_enemy_Theta[i]

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
					if distance_ball_robot[i]/1000 < 0.115 && radian_ball_robot[i]*180/math.Pi < 20 && radian_ball_robot[i]*180/math.Pi > -20 {
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

var imu_reset_time time.Time

func IMUReset(chimu chan bool, ourteam int, simmode bool) {
	for {
		if simmode {
			for i := 0; i < 16; i++ {
				robot_online[i] = true
			}
		}
		if isvisionrecv {
			var signal []*pb_gen.GrSim_Robot_Command

			for i := 0; i < 16; i++ {
				if robot_online[i] {
					signal = append(signal, createIMUSignal(uint32(i), ourteam))
				}
			}

			command := addIMUSignalToIMUSignals(signal)
			packet := &pb_gen.GrSim_Packet{
				Commands: command,
			}
			//log.Println(packet)
			marshalpacket, _ := proto.Marshal(packet)

			for i := 0; i < 16; i++ {
				if robot_online[i] {
					ipv4 := robot_ipaddr[i]
					port := "20011"
					addr := ipv4 + ":" + port

					conn, err := net.Dial("udp", addr)
					CheckError(err)
					conn.Write(marshalpacket)
					time.Sleep(1 * time.Millisecond)
					conn.Write(marshalpacket)
					log.Println("IMU Reset Signal Sent to Robot ID: ", i)
				}
			}
			imu_reset_time = time.Now()
		}
		time.Sleep(IMU_RESET_INTERVAL)
	}

}
func FPSCounter(chfps chan bool, ourteam int) {
	imu_reset_time = time.Now()
	for {
		//Calculate Online Robot IDs
		var online_robot_id_str string
		for i := 0; i < 16; i++ {
			if robot_online[i] {
				online_robot_id_str += fmt.Sprintf("%02d", i) + " "
			} else {
				online_robot_id_str += "-- "
			}
		}
		if framecounter >= 1 {
			isvisionrecv = true
			fps = framecounter

			framecounter = 0

			//Time to second
			log.Printf("Estimated FPS:  %d FPS // Last IMU Reset Time %s ago", fps, time.Duration(time.Since(imu_reset_time).Seconds())*time.Second)
			log.Printf("Vision: %t, Connected Robots: %s", isvisionrecv, online_robot_id_str)
			var our_yellows int
			var our_reds int

			if ref_command != nil {
				if ourteam == 0 {
					our_yellows = int(ref_command.Blue.GetYellowCards())
					our_reds = int(ref_command.Blue.GetRedCards())

					switch ref_command.GetCommand() {
					case pb_gen.Referee_TIMEOUT_BLUE:
						log.Printf("TIMEOUT %s Remain Second: %d, Remain Usable Times %d", ref_command.Blue.GetName(), ref_command.Blue.GetTimeoutTime()/1000000, ref_command.Blue.GetTimeouts())
					case pb_gen.Referee_TIMEOUT_YELLOW:
						log.Printf("TIMEOUT %s Remain Second: %d, Remain Usable Times %d", ref_command.Yellow.GetName(), ref_command.Yellow.GetTimeoutTime()/1000000, ref_command.Yellow.GetTimeouts())
					case pb_gen.Referee_BALL_PLACEMENT_BLUE:
						log.Printf("BALL_PLACEMENT_OUR %s ToPos(%.1f, %.1f) NowPos(%.1f, %.1f), Distance: %.1f", ref_command.Blue.GetName(), ref_command.DesignatedPosition.GetX(), ref_command.DesignatedPosition.GetY(), ball.GetX(), ball.GetY(), math.Sqrt(math.Pow(float64(ref_command.DesignatedPosition.GetX()-ball.GetX()), 2)+math.Pow(float64(ref_command.DesignatedPosition.GetY()-ball.GetY()), 2)))
					default:
						log.Printf("Current Referee Command: %s, Our Yellow Cards: %d, Our Red Cards: %d", ref_command.GetCommand().String(), our_yellows, our_reds)
					}
					// log.Println(ref_command.String())
				} else {
					our_yellows = int(ref_command.Yellow.GetYellowCards())
					our_reds = int(ref_command.Yellow.GetRedCards())

					switch ref_command.GetCommand() {
					case pb_gen.Referee_TIMEOUT_BLUE:
						log.Printf("TIMEOUT %s Remain Second: %d, Remain Usable Times %d", ref_command.Blue.GetName(), ref_command.Blue.GetTimeoutTime()/1000000, ref_command.Blue.GetTimeouts())
					case pb_gen.Referee_TIMEOUT_YELLOW:
						log.Printf("TIMEOUT %s Remain Second: %d, Remain Usable Times %d", ref_command.Yellow.GetName(), ref_command.Yellow.GetTimeoutTime()/1000000, ref_command.Yellow.GetTimeouts())
					case pb_gen.Referee_BALL_PLACEMENT_YELLOW:
						log.Printf("BALL_PLACEMENT_OUR %s ToPos(%.1f, %.1f) NowPos(%.1f, %.1f), Distance: %.1f", ref_command.Yellow.GetName(), ref_command.DesignatedPosition.GetX(), ref_command.DesignatedPosition.GetY(), ball.GetX(), ball.GetY(), math.Sqrt(math.Pow(float64(ref_command.DesignatedPosition.GetX()-ball.GetX()), 2)+math.Pow(float64(ref_command.DesignatedPosition.GetY()-ball.GetY()), 2)))
					default:
						log.Printf("Current Referee Command: %s, Our Yellow Cards: %d, Our Red Cards: %d", ref_command.GetCommand().String(), our_yellows, our_reds)
					}
				}
			}
			secperframe = 1 / float32(fps)

		} else {
			isvisionrecv = false
			secperframe = 100000
			log.Printf("Vision: %t, Connected Robots: %s", isvisionrecv, online_robot_id_str)
		}

		time.Sleep(1 * time.Second)
	}
}

var robot_online [16]bool

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
		for i := 0; i < 16; i++ {
			if robot_online_count[i] != 0 {
				robot_online[i] = true
				robot_online_count[i] -= 1
			} else {
				robot_online[i] = false
				robot_ipaddr[i] = ""
			}
		}
		time.Sleep(1 * time.Second)
	}
}

func createIMUSignal(i uint32, ourteam int) *pb_gen.GrSim_Robot_Command {
	var robotid uint32 = uint32(i + 100)
	var kickspeedx float32 = 0
	var kickspeedz float32 = 0
	var veltangent float32 = 0
	var velnormal float32 = 0
	var velangular float32 = bluerobots[i].GetOrientation()
	if ourteam == 1 {
		velangular = yellowrobots[i].GetOrientation()
	}
	var spinner bool = false
	var wheelsspeed bool = false

	pe := &pb_gen.GrSim_Robot_Command{
		Id:          &robotid,
		Kickspeedx:  &kickspeedx,
		Kickspeedz:  &kickspeedz,
		Veltangent:  &veltangent,
		Velnormal:   &velnormal,
		Velangular:  &velangular,
		Spinner:     &spinner,
		Wheelsspeed: &wheelsspeed,
	}
	return pe
}

func addIMUSignalToIMUSignals(imusignals []*pb_gen.GrSim_Robot_Command) *pb_gen.GrSim_Commands {
	var timestamp float64 = float64(time.Now().UnixNano() / 1e6)
	var isteamyellow bool = false

	var ImuSignal []*pb_gen.GrSim_Robot_Command
	for _, signal := range imusignals {
		if signal != nil {
			ImuSignal = append(ImuSignal, signal)
		}
	}

	ImuSignals := &pb_gen.GrSim_Commands{
		Timestamp:     &timestamp,
		Isteamyellow:  &isteamyellow,
		RobotCommands: ImuSignal,
	}

	return ImuSignals
}

func createRobotInfo(i int, ourteam int, simmode bool) *pb_gen.Robot_Infos {
	var robotid uint32
	var x float32
	var y float32
	var theta float32
	if ourteam == 0 {
		robotid = bluerobots[i].GetRobotId()
		x = bluerobots[i].GetX()
		// x = filtered_robot_x[i]
		y = bluerobots[i].GetY()
		// y = filtered_robot_y[i]
		theta = bluerobots[i].GetOrientation()
	} else {
		robotid = yellowrobots[i].GetRobotId()
		x = yellowrobots[i].GetX()
		// x = filtered_robot_x[i]
		y = yellowrobots[i].GetY()
		// y = filtered_robot_y[i]
		theta = yellowrobots[i].GetOrientation()
	}
	var diffx float32 = robot_difference_X[i]
	var diffy float32 = robot_difference_Y[i]
	var difftheta float32 = robot_difference_Theta[i]

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

func addRobotIpToRobotIps(robotip [16]*pb_gen.RobotIP_Infos) []*pb_gen.RobotIP_Infos {
	RobotIps := []*pb_gen.RobotIP_Infos{}

	for _, robot := range robotip {
		if robot != nil {
			RobotIps = append(RobotIps, robot)
		}
	}

	return RobotIps
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
		var robotid uint32
		var x float32
		var y float32
		var theta float32
		var diffx float32 = enemy_difference_X[i]
		var diffy float32 = enemy_difference_Y[i]
		var difftheta float32 = enemy_difference_Theta[i]

		if ourteam == 0 {
			robotid = yellowrobots[i].GetRobotId()
			x = yellowrobots[i].GetX()
			y = yellowrobots[i].GetY()
			theta = yellowrobots[i].GetOrientation()
		} else {
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
		CenterCircleRadius = int32(centercircleradius)
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
		IsBallMoving:     &is_ball_moving,
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

func addRobotIPInfoToRobotIPInfos(robotipinfo [16]*pb_gen.RobotIP_Infos) []*pb_gen.RobotIP_Infos {
	RobotIPInfos := []*pb_gen.RobotIP_Infos{}

	for _, robotip := range robotipinfo {
		if robotip != nil {
			RobotIPInfos = append(RobotIPInfos, robotip)
		}
	}

	return RobotIPInfos
}

func RunServer(chserver chan bool, reportrate uint, ourteam int, goalpose int, debug bool, simmode bool) {
	ipv4 := NW_AI_IPADDR
	port := NW_AI_PORT
	addr := ipv4 + ":" + port

	log.Println("Send to:", addr)

	conn, err := net.Dial("udp", addr)
	CheckError(err)
	defer conn.Close()

	var counter int

	for {

		var robot_infos [16]*pb_gen.Robot_Infos
		var enemy_infos [16]*pb_gen.Robot_Infos
		var robotip_infos [16]*pb_gen.RobotIP_Infos

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

		//Robot IP
		for i, ipaddr := range robot_ipaddr {
			if ipaddr != "" {
				idtouint32 := uint32(i)
				ip := ipaddr
				robotip_infos[i] = &pb_gen.RobotIP_Infos{
					RobotId: &idtouint32,
					Ip:      &ip,
				}
			}
		}

		RobotInfos := addRobotInfoToRobotInfos(robot_infos)

		EnemyInfos := addEnemyInfoToEnemyInfos(enemy_infos)

		BallInfo := createBallInfo()

		RobotIpInfo := addRobotIPInfoToRobotIPInfos(robotip_infos)

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
			RobotIps:    RobotIpInfo,
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

func main() {

	var (
		visionport        = flag.Int("p", 10006, "Vision Multicast Port Number")
		ourteam           = flag.String("t", "blue", "Our Team (blue or yellow)")
		goalpos           = flag.String("g", "N", "Attack Direction(Enemy goal) Negative or Positive (N or P)")
		reportrate        = flag.Uint("r", 16, "How often report to RACOON-AI? (milliseconds)")
		debug             = flag.Bool("d", false, "Show All Send Packet")
		simmode           = flag.Bool("s", false, "Simulation Mode (Emulate Ball Sensor)")
		replay            = flag.Bool("replay", false, "Replay All Packet")
		halfswitch        = flag.String("c", "F", "Where to use (N, P, F) F to Full")
		ballmovethreshold = flag.Float64("b", 1000, "Ball Detect Threshold (Default 1000")
		nw_robot          = flag.String("rif", "none", "NW Robot Update Interface Name (ex. en0)")
		nw_vision         = flag.String("vif", "none", "NW Vision and Referee receive Interface Name (ex. en1)")
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

	var halfswitch_n int
	if *halfswitch == "N" {
		halfswitch_n = -1
	} else if *halfswitch == "P" {
		halfswitch_n = 1
	} else {
		halfswitch_n = 0
	}

	if *nw_robot != "none" {
		NW_ROBOT_UPDATE_INTERFACE_NAME = *nw_robot
	}

	if *nw_vision != "none" {
		NW_VISION_REFEREE_INTERFACE_NAME = *nw_vision
	}

	if *ballmovethreshold != 1000 {
		BALL_MOVING_THRESHOULD_SPEED = float32(*ballmovethreshold)
	}

	chupdate := make(chan bool)
	chserver := make(chan bool)
	chvision := make(chan bool)
	chref := make(chan bool)
	chfps := make(chan bool)
	chvisrobot := make(chan bool)
	chimu := make(chan bool)
	chrobotip := make(chan bool)

	go Update(chupdate)
	go RunServer(chserver, *reportrate, ourteam_n, goalpos_n, *debug, *simmode)
	go VisionReceive(chvision, *visionport, ourteam_n, goalpos_n, *simmode, *replay, halfswitch_n)
	go CheckVisionRobot(chvisrobot)
	go FPSCounter(chfps, ourteam_n)
	go RefereeClient(chref)
	go IMUReset(chimu, ourteam_n, *simmode)
	go RobotIPList(chrobotip)

	<-chupdate
	<-chserver
	<-chvision
	<-chref
	<-chfps
	<-chvisrobot
	<-chimu
	<-chrobotip

}

func CheckError(err error) {
	if err != nil {
		log.Println("Error: ", err)
	}
}
