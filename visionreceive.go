package main

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"net"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"github.com/rosshemsley/kalman"
	"github.com/rosshemsley/kalman/models"
	"google.golang.org/protobuf/proto"
)

func VisionReceive(chvision chan bool, port int, ourteam int, goalpos int, simmode bool, replay bool, halfswitch_n int, debug_for_sono bool) {
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
		InitialVariance:     100,
		ProcessVariance:     0,
		ObservationVariance: 0.1,
	})
	filterBallX := kalman.NewKalmanFilter(modelBallX)
	//KalmanSmoother
	// smoothedBallX := kalman.NewKalmanSmoother(modelBallX)

	modelBallY = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     100,
		ProcessVariance:     0,
		ObservationVariance: 0.1,
	})
	filterBallY := kalman.NewKalmanFilter(modelBallY)
	// smoothedBallY := kalman.NewKalmanSmoother(modelBallX)

	var modelRobotX [16]*models.SimpleModel
	var modelRobotY [16]*models.SimpleModel

	var filterRobotX [16]*kalman.KalmanFilter
	var filterRobotY [16]*kalman.KalmanFilter

	for i := 0; i < 16; i++ {
		modelRobotX[i] = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
			InitialVariance:     100.0,
			ProcessVariance:     0,
			ObservationVariance: 0.1,
		})
		filterRobotX[i] = kalman.NewKalmanFilter(modelRobotX[i])

		modelRobotY[i] = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
			InitialVariance:     100,
			ProcessVariance:     0,
			ObservationVariance: 0.1,
		})
		filterRobotY[i] = kalman.NewKalmanFilter(modelRobotY[i])
	}

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
	robot_cords_file := new(os.File)
	if debug_for_sono {
		err := error(nil)
		robot_cords_file, err = os.OpenFile("./debug_for_sono.txt", os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0644)
		if err != nil {
			log.Fatal(err)
		}
	}

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

				filtered_ball_x = float32(modelBallX.Value(filterBallX.State()))
				filtered_ball_y = float32(modelBallY.Value(filterBallY.State()))

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

					robot_difference_X[i] = robot.GetX() - pre_robot_X[i]
					robot_difference_Y[i] = robot.GetY() - pre_robot_Y[i]
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

					pre_robot_X[i] = robot.GetX()
					pre_robot_Y[i] = robot.GetY()
					pre_robot_Theta[i] = robot.GetOrientation()

					radian_ball_robot[i] = Calc_degree_normalize(Calc_degree(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY()) - robot.GetOrientation())
					distance_ball_robot[i] = Calc_distance(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY())

					//Print microtime and robot 0 cords to text file
					if i == 0 && debug_for_sono {
						fmt.Fprintf(robot_cords_file, "%d, %f, %f, %f, %f, %f, %f\n", time.Now().UnixNano(), filtered_robot_x[0], filtered_robot_y[0], robot.GetOrientation(), robot.GetX(), robot.GetY(), robot.GetOrientation())
					}

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
