package main

import (
	"bufio"
	"fmt"
	"log"
	"math"

	"net"
	"strconv"
	"strings"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	// "github.com/rosshemsley/kalman"
	// "github.com/rosshemsley/kalman/models"
	"gonum.org/v1/gonum/mat"
	"google.golang.org/protobuf/proto"
)

func VisionReceive(chvision chan bool, port int, ourteam int, goalpos int, simmode bool, replay bool, halfswitch_n int, matchmode bool, initial_variance float64, process_variance float64, observation_variance float64) {

	var pre_ball_X float32
	var pre_ball_Y float32
	var pre_robot_X [16]float32
	var pre_robot_Y [16]float32
	var pre_robot_Theta [16]float32

	var pre_enemy_X [16]float32
	var pre_enemy_Y [16]float32
	var pre_enemy_Theta [16]float32

	// var t time.Time

	// var modelBallX *models.SimpleModel
	// var modelBallY *models.SimpleModel

	// var sum float64 = 0
	// var sumx float64 = 0
	// var sumy float64 = 0
	// var averagex float64 = 0
	// var averagey float64 = 0

	var ax float64
	var bx float64
	var ay float64
	var by float64
	var at float64
	var bt float64
	var Qv *mat.Dense
	var Qw *mat.Dense
	var our_xh_k_1 [16]*mat.Dense
	var our_P_k_1 [16]*mat.Dense
	var our_u_k_1 [16]*mat.Dense

	var enemy_Qv *mat.Dense
	var enemy_xh_k_1 [16]*mat.Dense
	var enemy_P_k_1 [16]*mat.Dense
	var enemy_u_k_1 [16]*mat.Dense

	var m float64 = 0.046 //[kg] mass of the ball
	// var mu float64 = 0.05  //[N・s/m] friction coefficient
	var Ts float64 = 0.016 //[s] sampling time
	var K float64 = 20.0
	var Ad_lowpass float64 = 0.818731
	var Bd_lowpass float64 = 0.181269
	var DeltaPmax float64 = 6 * Ts * 4
	var ObPosX_k_1 float64 = 0.0
	var ObPosY_k_1 float64 = 0.0
	var ObPosX_lowpass float64 = 0.0
	var ObPosY_lowpass float64 = 0.0
	var ObPosX_lowpass_k_1 float64 = 0.0
	var ObPosY_lowpass_k_1 float64 = 0.0
	var ObVelX float64 = 0.0
	var ObVelY float64 = 0.0
	var ObVelX_k_1 float64 = 0.0
	var ObVelY_k_1 float64 = 0.0
	var tempX_1 float64 = 0.0
	var tempY_1 float64 = 0.0
	var tempX_2 float64 = 0.0
	var tempY_2 float64 = 0.0
	var ObPosX float64 = 0.0
	var ObPosY float64 = 0.0
	var Thru_Count int = 0
	var initial_flag bool = true

	// modelBallX = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
	// 	InitialVariance:     initial_variance,
	// 	ProcessVariance:     process_variance,
	// 	ObservationVariance: observation_variance,
	// })
	// filterBallX := kalman.NewKalmanFilter(modelBallX)
	// //KalmanSmoother
	// // smoothedBallX := kalman.NewKalmanSmoother(modelBallX)

	// modelBallY = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
	// 	InitialVariance:     initial_variance,
	// 	ProcessVariance:     process_variance,
	// 	ObservationVariance: observation_variance,
	// })
	// filterBallY := kalman.NewKalmanFilter(modelBallY)
	// smoothedBallY := kalman.NewKalmanSmoother(modelBallX)

	// var modelRobotX [16]*models.SimpleModel
	// var modelRobotY [16]*models.SimpleModel

	// var filterRobotX [16]*kalman.KalmanFilter
	// var filterRobotY [16]*kalman.KalmanFilter

	// Ax := []float64{-ax, 0, 0, 0, -ay, 0, 0, 0, -at}
	// Bx := []float64{bx, 0, 0, 0, by, 0, 0, 0, bt}
	Cx := []float64{1, 0, 0, 0, 1, 0, 0, 0, 1}
	// x0x := []float64{0, 0, 0, 0, 0, 0}
	xh0x := []float64{0.1, 0, 0}
	// A := mat.NewDense(3, 3, Ax)
	// B := mat.NewDense(3, 3, Bx)
	C := mat.NewDense(3, 3, Cx)
	// x0 := mat.NewDense(6, 1, x0x)
	xh0 := mat.NewDense(3, 1, xh0x)

	Qvx := mat.NewDiagDense(3, []float64{1, 1, 1})

	Qv = mat.NewDense(3, 3, nil)
	Qv.Scale(0.1, Qvx)

	// Qwx := mat.NewDiagDense(3, []float64{1, 1, 1})
	kalman_data := []float64{0.004, 0.0, 0.0, 0.0, 0.004, 0, 0, 0, 4.3e-10}
	Qw = mat.NewDense(3, 3, kalman_data)
	// Qw.Scale(0.1, Qwx)

	// P_k := []float64{1, 0, 0, 0, 1, 0, 0, 0, 1}

	enemy_Ax := []float64{-ax, 0, 0, 0, -ay, 0, 0, 0, -at}
	enemy_Bx := []float64{bx, 0, 0, 0, by, 0, 0, 0, bt}
	enemy_Cx := []float64{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}

	enemy_xh0x := []float64{0.1, 0, 0, 0, 0, 0}
	enemy_A := mat.NewDense(3, 3, enemy_Ax)
	enemy_B := mat.NewDense(3, 3, enemy_Bx)
	enemy_C := mat.NewDense(3, 6, enemy_Cx)
	// x0 := mat.NewDense(6, 1, x0x)
	enemy_xh0 := mat.NewDense(6, 1, enemy_xh0x)
	// enemy_Qvx := mat.NewDiagDense(3, []float64{1, 1, 1})
	enemy_Qvx := mat.NewDiagDense(6, []float64{1, 1, 1, 1, 1, 1})

	enemy_Qv = mat.NewDense(6, 6, nil)
	enemy_Qv.Scale(0.1, enemy_Qvx)
	// enemy_Qwx := mat.NewDiagDense(3, []float64{1, 1, 1})
	// Qw = mat.NewDense(3, 3, nil)
	// enemy_Qw.Scale(0.1, Qwx)

	for i := 0; i < 16; i++ {
		our_xh_k_1[i] = mat.DenseCopyOf(xh0)
		our_P_k_1[i] = mat.NewDense(3, 3, nil)
		our_u_k_1[i] = mat.NewDense(3, 1, nil)

		enemy_xh_k_1[i] = mat.DenseCopyOf(enemy_xh0)
		enemy_P_k_1[i] = mat.NewDense(6, 6, nil)
		enemy_u_k_1[i] = mat.NewDense(3, 1, nil)
	}

	var pre_framecounter int = 0
	// var count int = 0

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
	// robot_cords_file := new(os.File)

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

	// f := new(os.File)
	// f, _ = os.OpenFile("./ball_cords.txt", os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0644)

	for {
		var visible_in_vision_b [16]bool
		var visible_in_vision_y [16]bool
		for i := 0; i < 16; i++ {
			visible_in_vision_b[i] = false
			visible_in_vision_y[i] = false
		}

		//チームカラー検査
		if teamcolor_from_ref != -1 && matchmode {
			ourteam = teamcolor_from_ref
		}

		var is_ball_exists bool = false
		flag_ball = false
		// var get_ball bool = false
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

			err = proto.Unmarshal(buf[0:n], packet)
			CheckError(err)

			visionwrapper[i] = packet
			visiondetection[i] = packet.Detection

			// Receive Geometry Data
			if packet.Geometry != nil { //Geometry Data
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

			// Get Blue Robots
			for _, robot := range packet.Detection.GetRobotsBlue() {
				switch halfswitch_n {
				case 0:
					bluerobots[robot.GetRobotId()] = robot
					visible_in_vision_b[robot.GetRobotId()] = true

				case 1:
					if robot.GetX() > 0 {
						bluerobots[robot.GetRobotId()] = robot
						visible_in_vision_b[robot.GetRobotId()] = true
					}

				case -1:
					if robot.GetX() <= 0 {
						bluerobots[robot.GetRobotId()] = robot
						visible_in_vision_b[robot.GetRobotId()] = true
					}
				}
			}

			// Get Yellow Robots
			for _, robot := range packet.Detection.GetRobotsYellow() {
				switch halfswitch_n {
				case 0:
					yellowrobots[robot.GetRobotId()] = robot
					visible_in_vision_y[robot.GetRobotId()] = true

				case 1:
					if robot.GetX() > 0 {
						yellowrobots[robot.GetRobotId()] = robot
						visible_in_vision_y[robot.GetRobotId()] = true
					}

				case -1:
					if robot.GetX() <= 0 {
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

			is_ball_exists = false

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
						flag_ball = true
						initial_flag = false
					}
				}

				if is_ball_exists {
					ball = maxconfball
				}

				if !flag_ball {
					flag_ball = true
				}

			} else {
				if initial_flag {
					if halfswitch_n == 1 {
						ball = &pb_gen.SSL_DetectionBall{
							Confidence: proto.Float32(0.0),
							X:          proto.Float32(150.0),
							Y:          proto.Float32(0.0),
							PixelX:     proto.Float32(0.0),
							PixelY:     proto.Float32(0.0),
						}
					} else if halfswitch_n == -1 {
						ball = &pb_gen.SSL_DetectionBall{
							Confidence: proto.Float32(0.0),
							X:          proto.Float32(-150.0),
							Y:          proto.Float32(0.0),
							PixelX:     proto.Float32(0.0),
							PixelY:     proto.Float32(0.0),
						}
					} else {
						ball = &pb_gen.SSL_DetectionBall{
							Confidence: proto.Float32(0.0),
							X:          proto.Float32(0.0),
							Y:          proto.Float32(0.0),
							PixelX:     proto.Float32(0.0),
							PixelY:     proto.Float32(0.0),
						}
					}
				}
			}
		}
		// log.Print("flag_ball: ", flag_ball)
		framecounter++
		// count++

		// var detection *pb_gen.SSL_DetectionFrame
		// var t_received float32
		// var frameinterval float32

		// t_received += float32(detection.GetTSent()) - float32(detection.GetTCapture())
		// log.Println("t_capture: ", detection.GetTCapture())
		// log.Println("t_sent: ", detection.GetTSent())
		// log.Println("t_received: ", t_received)
		// if t_received > 0 {
		// 	frameinterval += t_received / float32(count)
		// 	framecounter = int(1 / frameinterval)
		// }

		// log.Println("framecounter: ", framecounter)

		if framecounter-pre_framecounter > 0 {

			/////////////////////////////////////
			//
			//	KALMAN FILTER (BALL)
			//
			/////////////////////////////////////
			if ball != nil {

				if framecounter == 1 {
					ObPosX_k_1 = float64(ball.GetX() / 1000)
					ObPosY_k_1 = float64(ball.GetY() / 1000)
					ObPosX_lowpass_k_1 = float64(ball.GetX() / 1000)
					ObPosY_lowpass_k_1 = float64(ball.GetY() / 1000)
				}

				TempX := ObPosX_k_1
				TempY := ObPosY_k_1
				Temp_lowpassX := 0.0
				Temp_lowpassY := 0.0
				TempVelX := 0.0
				TempVelY := 0.0

				ballPosXInMeter := float32(ball.GetX() / 1000)
				ballPosYInMeter := float32(ball.GetY() / 1000)

				for i := 0; i < 3; i++ {
					DeltaX := ballPosXInMeter - float32(ObPosX_lowpass_k_1)
					DeltaY := ballPosYInMeter - float32(ObPosY_lowpass_k_1)
					if math.Abs(float64(DeltaX)) > DeltaPmax {
						tempX_2 = 0.0
						Thru_Count++
					} else {
						tempX_2 = float64(ballPosXInMeter) - TempX
					}

					if math.Abs(float64(DeltaY)) > DeltaPmax {
						tempY_2 = 0.0
						Thru_Count++
					} else {
						tempY_2 = float64(ballPosYInMeter) - TempY
					}
					if flag_ball {
						tempX_1 = tempX_2
						tempY_1 = tempY_2
					} else {
						tempX_1 = 0.0
						tempY_1 = 0.0
					}

					accX := (K / m) * tempX_1
					accY := (K / m) * tempY_1
					// log.Println("accX: ", accX, "accY: ", accY)
					TempVelX = ObVelX_k_1 + Ts*accX
					TempVelY = ObVelY_k_1 + Ts*accY
					TempX = ObPosX_k_1 + Ts*TempVelX
					TempY = ObPosY_k_1 + Ts*TempVelY
					// fmt.Printf("TempX: %f, TempY: %f\n", TempX, TempY)
					Temp_lowpassX = Bd_lowpass*TempX + Ad_lowpass*ObPosX_lowpass_k_1
					Temp_lowpassY = Bd_lowpass*TempY + Ad_lowpass*ObPosY_lowpass_k_1
				}
				ObPosX = TempX
				ObPosY = TempY
				ObVelX = TempVelX
				ObVelY = TempVelY
				// ObVelX = ObVelX * math.Exp(-mu*(m*10))
				// ObVelY = ObVelY * math.Exp(-mu*(m*10))

				if Thru_Count < 150 {
					ObPosX_lowpass = Temp_lowpassX
					ObPosY_lowpass = Temp_lowpassY
				} else {
					ObPosX_lowpass = float64(ballPosXInMeter)
					ObPosY_lowpass = float64(ballPosYInMeter)
					ObPosX = float64(ballPosXInMeter)
					ObPosY = float64(ballPosYInMeter)
					ObVelX = 0.0
					ObVelY = 0.0
				}

				ObPosX_k_1 = TempX
				ObPosY_k_1 = TempY
				ObVelX_k_1 = ObVelX
				ObVelY_k_1 = ObVelY
				ObPosX_lowpass_k_1 = ObPosX_lowpass
				ObPosY_lowpass_k_1 = ObPosY_lowpass

				filtered_ball_x = float32(ObPosX * 1000)
				filtered_ball_y = float32(ObPosY * 1000)
				// log.Println("filtered_ball_x: ", filtered_ball_x, "filtered_ball_y: ", filtered_ball_y, "X: ", ball.GetX(), "Y: ", ball.GetY())
				// log.Println("ball_x: ", ball.GetX(), "ball_y: ", ball.GetY())

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
					ball_speed = float32(math.Sqrt(math.Pow(bdX64, 2)+math.Pow(bdY64, 2)) / float64(secperframe))
				} else {
					ball_slope_degree = 0.0
					ball_intercept = 0.0
					ball_speed = 0.0
				}

				pre_ball_X = filtered_ball_x
				pre_ball_Y = filtered_ball_y

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
					x3 := our_xh_k_1[i].At(2, 0)
					// x4 := xh_k_1[i].At(3, 0)
					// x5 := xh_k_1[i].At(4, 0)
					// x6 := xh_k_1[i].At(5, 0)
					our_u_k_1[i].Set(0, 0, float64(controllerRobotVelocitys[i].X))
					our_u_k_1[i].Set(1, 0, float64(controllerRobotVelocitys[i].Y))
					our_u_k_1[i].Set(2, 0, float64(controllerRobotVelocitys[i].Angular))

					//Sampling time
					dt := 1e-2

					//Observe theta
					the := x3

					//create A and B matrix
					R := mat.NewDense(3, 3, []float64{math.Cos(the), -math.Sin(math.Pi), 0, math.Sin(the), math.Cos(the), 0, 0, 0, 1})
					DR := mat.NewDense(3, 3, []float64{-math.Sin(the), math.Cos(the), 0, math.Cos(the), -math.Sin((the)), 0, 0, 0, 0})

					v := mat.NewDense(3, 1, []float64{our_u_k_1[i].At(0, 0), our_u_k_1[i].At(1, 0), our_u_k_1[i].At(2, 0)})

					// RA := mat.NewDense(6, 3, nil)
					At := mat.NewDense(3, 3, nil)
					// Bx := mat.NewDense(6, 3, nil)
					// RA.Stack(R, A)
					At.Scale(dt, R)
					// Bx.Stack(zero2, B)

					I := mat.NewDense(3, 3, []float64{1, 0, 0, 0, 1, 0, 0, 0, 1})
					Ad := mat.NewDense(3, 3, nil)
					Bd := mat.NewDense(3, 3, nil)
					Ad.Add(I, At)
					Bd.Scale(dt, I)

					zero3 := mat.NewDense(3, 2, nil)
					// zero4 := mat.NewDense(3, 1, nil)
					F_k_1 := mat.NewDense(3, 3, nil)
					DRv := mat.NewDense(3, 1, nil)
					DRvdt := mat.NewDense(3, 1, nil)
					// DRvzero4 := mat.NewDense(6, 1, nil)
					// zero3DRvdt := mat.NewDense(6, 3, nil)
					xhb_k := mat.NewDense(3, 1, nil)
					Adxh_k_1 := mat.NewDense(3, 1, nil)
					Bdu_k_1 := mat.NewDense(3, 1, nil)
					Pb_k := mat.NewDense(3, 3, nil)

					DRv.Product(DR, v)
					DRvdt.Scale(dt, DRv)
					// DRvzero4.Stack(DRvdt, zero4)
					// zero3DRvdt.Augment(zero3, DRvdt)
					F_k_1.Augment(zero3, DRvdt)
					F_k_1.Add(Ad, F_k_1)
					Adxh_k_1.Product(Ad, our_xh_k_1[i])
					Bdu_k_1.Product(Bd, our_u_k_1[i])
					xhb_k.Add(Adxh_k_1, Bdu_k_1)
					Pb_k.Product(F_k_1, our_P_k_1[i], F_k_1.T())
					Pb_k.Add(Pb_k, Qv)

					//Filtering Process
					inv := mat.NewDense(3, 3, nil)
					G_k := mat.NewDense(3, 3, nil)
					xh_k := mat.NewDense(3, 1, nil)
					y_k := mat.NewDense(3, 1, nil)
					y_kCxhb_k := mat.NewDense(3, 1, nil)
					G_ky_kCxhb_k := mat.NewDense(3, 1, nil)

					inv.Product(C, Pb_k, C.T())
					inv.Add(inv, Qw)
					inv.Inverse(inv)
					G_k.Product(Pb_k, C.T(), inv)
					y_k.Set(0, 0, float64(robot.GetX()))
					y_k.Set(1, 0, float64(robot.GetY()))
					y_k.Set(2, 0, float64(robot.GetOrientation()))
					y_kCxhb_k.Product(C, xhb_k)
					y_kCxhb_k.Sub(y_k, y_kCxhb_k)
					G_ky_kCxhb_k.Product(G_k, y_kCxhb_k)
					xh_k.Add(xhb_k, G_ky_kCxhb_k)
					our_xh_k_1[i] = xh_k

					P_k := mat.NewDense(3, 3, nil)
					G_kC := mat.NewDense(3, 3, nil)
					IG_kC := mat.NewDense(3, 3, nil)
					G_kC.Product(G_k, C)
					IG_kC.Sub(I, G_kC)
					P_k.Product(IG_kC, Pb_k)
					our_P_k_1[i] = P_k

					filtered_robot_x[i] = float32(xh_k.At(0, 0))
					filtered_robot_y[i] = float32(xh_k.At(1, 0))
					filtered_robot_theta[i] = float32(xh_k.At(2, 0))
					// fmt.Println("ID: ", i, "X:", robot.GetX(), "Y:", robot.GetY(), "Theta:", robot.GetOrientation(), "Filtered X:", filtered_robot_x[i], "Filtered Y:", filtered_robot_y[i], "Filtered Theta:", filtered_robot_theta[i])

					robot_difference_X[i] = filtered_robot_x[i] - pre_robot_X[i]
					robot_difference_Y[i] = filtered_robot_y[i] - pre_robot_Y[i]
					robot_difference_Theta[i] = Calc_degree_normalize(filtered_robot_theta[i] - pre_robot_Theta[i])

					rdX64[i] = float64(robot_difference_X[i])
					rdY64[i] = float64(robot_difference_Y[i])

					if robot_difference_Y[i] != 0 || robot_difference_X[i] != 0 {
						robot_slope[i] = robot_difference_Y[i] / robot_difference_X[i]
						robot_intercept[i] = filtered_robot_y[i] - (robot_slope[i] * filtered_robot_x[i])
						robot_speed[i] = float32(math.Sqrt(math.Pow(rdX64[i], 2)+math.Pow(rdY64[i], 2)) / 0.016)
					} else {
						robot_slope[i] = 0.0
						robot_intercept[i] = filtered_robot_y[i]
						robot_speed[i] = 0.0
					}

					robot_angular_velocity[i] = robot_difference_Theta[i] / 0.016

					pre_robot_X[i] = filtered_robot_x[i]
					pre_robot_Y[i] = filtered_robot_y[i]
					pre_robot_Theta[i] = filtered_robot_theta[i]

					radian_ball_robot[i] = Calc_degree_normalize(Calc_degree(filtered_ball_x, filtered_ball_y, filtered_robot_x[i], filtered_robot_y[i]) - filtered_robot_theta[i])
					distance_ball_robot[i] = Calc_distance(filtered_ball_x, filtered_ball_y, filtered_robot_x[i], filtered_robot_y[i])

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

					//Kalman Filter
					x3 := enemy_xh_k_1[i].At(2, 0)
					x4 := enemy_xh_k_1[i].At(3, 0)
					x5 := enemy_xh_k_1[i].At(4, 0)
					x6 := enemy_xh_k_1[i].At(5, 0)

					dt := 1e-2

					the := x3

					R := mat.NewDense(3, 3, []float64{math.Cos(the), -math.Sin(math.Pi), 0, math.Sin(the), math.Cos(the), 0, 0, 0, 1})
					DR := mat.NewDense(3, 3, []float64{-math.Sin(the), math.Cos(the), 0, math.Cos(the), -math.Sin((the)), 0, 0, 0, 0})

					v := mat.NewDense(3, 1, []float64{x4, x5, x6})

					zero1 := mat.NewDense(6, 3, []float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})
					zero2 := mat.NewDense(3, 3, []float64{0, 0, 0, 0, 0, 0, 0, 0, 0})
					RA := mat.NewDense(6, 3, nil)
					Ax := mat.NewDense(6, 6, nil)
					Bx := mat.NewDense(6, 3, nil)
					RA.Stack(R, enemy_A)
					Ax.Augment(zero1, RA)
					Bx.Stack(zero2, enemy_B)

					I := mat.NewDense(6, 6, []float64{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1})
					Ad := mat.NewDense(6, 6, nil)
					Bd := mat.NewDense(6, 3, nil)
					Ad.Scale(dt, Ax)
					Ad.Add(I, Ad)
					Bd.Scale(dt, Bx)

					zero3 := mat.NewDense(6, 2, nil)
					zero4 := mat.NewDense(3, 1, nil)
					F_k_1 := mat.NewDense(6, 6, nil)
					DRv := mat.NewDense(3, 1, nil)
					DRvdt := mat.NewDense(3, 1, nil)
					DRvzero4 := mat.NewDense(6, 1, nil)
					zero3DRvdt := mat.NewDense(6, 3, nil)
					xhb_k := mat.NewDense(6, 1, nil)
					Adxh_k_1 := mat.NewDense(6, 1, nil)
					Bdu_k_1 := mat.NewDense(6, 1, nil)
					Pb_k := mat.NewDense(6, 6, nil)

					DRv.Product(DR, v)
					DRvdt.Scale(dt, DRv)
					DRvzero4.Stack(DRvdt, zero4)
					zero3DRvdt.Augment(zero3, DRvzero4)
					F_k_1.Augment(zero3DRvdt, zero1)
					Adxh_k_1.Product(Ad, enemy_xh_k_1[i])
					Bdu_k_1.Product(Bd, enemy_u_k_1[i])
					xhb_k.Add(Adxh_k_1, Bdu_k_1)
					Pb_k.Product(F_k_1, enemy_P_k_1[i], F_k_1.T())
					Pb_k.Add(Pb_k, enemy_Qv)

					inv := mat.NewDense(3, 3, nil)
					G_k := mat.NewDense(6, 3, nil)
					xh_k := mat.NewDense(6, 1, nil)
					y_k := mat.NewDense(3, 1, nil)
					y_kCxhb_k := mat.NewDense(3, 1, nil)
					G_ky_kCxhb_k := mat.NewDense(6, 1, nil)

					inv.Product(enemy_C, Pb_k, enemy_C.T())
					inv.Add(inv, Qw)
					inv.Inverse(inv)
					G_k.Product(Pb_k, enemy_C.T(), inv)
					y_k.Set(0, 0, float64(enemy.GetX()))
					y_k.Set(1, 0, float64(enemy.GetY()))
					y_k.Set(2, 0, float64(enemy.GetOrientation()))
					y_kCxhb_k.Product(enemy_C, xhb_k)
					y_kCxhb_k.Sub(y_k, y_kCxhb_k)
					G_ky_kCxhb_k.Product(G_k, y_kCxhb_k)
					xh_k.Add(xhb_k, G_ky_kCxhb_k)
					enemy_xh_k_1[i] = xh_k

					P_k := mat.NewDense(6, 6, nil)
					G_kC := mat.NewDense(6, 6, nil)
					IG_kC := mat.NewDense(6, 6, nil)
					G_kC.Product(G_k, enemy_C)
					IG_kC.Sub(I, G_kC)
					P_k.Product(IG_kC, Pb_k)
					enemy_P_k_1[i] = P_k

					filtered_enemy_x[i] = float32(xh_k.At(0, 0))
					filtered_enemy_y[i] = float32(xh_k.At(1, 0))
					filtered_enemy_theta[i] = float32(xh_k.At(2, 0))

					enemy_difference_X[i] = filtered_enemy_x[i] - pre_enemy_X[i]
					enemy_difference_Y[i] = filtered_enemy_y[i] - pre_enemy_Y[i]
					enemy_difference_Theta[i] = Calc_degree_normalize(filtered_enemy_theta[i] - pre_enemy_Theta[i])

					edX64[i] = float64(enemy_difference_X[i])
					edY64[i] = float64(enemy_difference_Y[i])

					if enemy_difference_Y[i] != 0 || enemy_difference_X[i] != 0 {
						enemy_slope[i] = enemy_difference_Y[i] / enemy_difference_X[i]
						enemy_intercept[i] = filtered_enemy_y[i] - (enemy_slope[i] * filtered_enemy_x[i])
						enemy_speed[i] = float32(math.Sqrt(math.Pow(edX64[i], 2)+math.Pow(edY64[i], 2)) / 0.016)
					} else {
						enemy_slope[i] = 0.0
						enemy_intercept[i] = filtered_enemy_y[i]
						enemy_speed[i] = 0.0
					}

					enemy_angular_velocity[i] = enemy_difference_Theta[i] / 0.016

					pre_enemy_X[i] = filtered_enemy_x[i]
					pre_enemy_Y[i] = filtered_enemy_y[i]
					pre_enemy_Theta[i] = filtered_enemy_theta[i]
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
						is_detect_photo_sensor[i] = true
						is_detect_dribbler_sensor[i] = true
						is_new_dribbler[i] = true

					} else {
						is_detect_photo_sensor[i] = false
						is_detect_dribbler_sensor[i] = false
						is_new_dribbler[i] = false
					}
				}
			}
		}
		pre_framecounter = framecounter

	}
	chvision <- true
}
