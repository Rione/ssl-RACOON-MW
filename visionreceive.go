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
	"gonum.org/v1/gonum/mat"
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
	var xh_k_1 [16]*mat.Dense
	var P_k_1 [16]*mat.Dense
	var u_k_1 [16]*mat.Dense

	modelBallX = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     100,
		ProcessVariance:     0.1,
		ObservationVariance: 0.18,
	})
	filterBallX := kalman.NewKalmanFilter(modelBallX)
	//KalmanSmoother
	// smoothedBallX := kalman.NewKalmanSmoother(modelBallX)

	modelBallY = models.NewSimpleModel(t, 0.0, models.SimpleModelConfig{
		InitialVariance:     100,
		ProcessVariance:     0.1,
		ObservationVariance: 0.08,
	})
	filterBallY := kalman.NewKalmanFilter(modelBallY)
	// smoothedBallY := kalman.NewKalmanSmoother(modelBallX)

	// var modelRobotX [16]*models.SimpleModel
	// var modelRobotY [16]*models.SimpleModel

	// var filterRobotX [16]*kalman.KalmanFilter
	// var filterRobotY [16]*kalman.KalmanFilter

	Ax := []float64{-ax, 0, 0, 0, -ay, 0, 0, 0, -at}
	Bx := []float64{bx, 0, 0, 0, by, 0, 0, 0, bt}
	Cx := []float64{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}
	// x0x := []float64{0, 0, 0, 0, 0, 0}
	xh0x := []float64{0.1, 0, 0, 0, 0, 0}
	A := mat.NewDense(3, 3, Ax)
	B := mat.NewDense(3, 3, Bx)
	C := mat.NewDense(3, 6, Cx)
	// x0 := mat.NewDense(6, 1, x0x)
	xh0 := mat.NewDense(6, 1, xh0x)

	Qvx := mat.NewDiagDense(6, []float64{1, 1, 1, 1, 1, 1})

	Qv = mat.NewDense(6, 6, nil)
	Qv.Scale(0.1, Qvx)

	kalman_data := []float64{
		0.185, 0.0, 0.0,
		0.0, 0.084, 0.0,
		0.0, 0.0, 8.3e-6,
	}
	Qw = mat.NewDense(3, 3, kalman_data)

	for i := 0; i < 16; i++ {
		xh_k_1[i] = mat.DenseCopyOf(xh0)
		P_k_1[i] = mat.NewDense(6, 6, nil)
		u_k_1[i] = mat.NewDense(3, 1, nil)
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

	// f := new(os.File)
	// f, _ = os.OpenFile("./ball_cords.txt", os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0644)

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

				// log.Println("filtered_ball: ", filtered_ball_x, filtered_ball_y)
				// fmt.Printf("X: before: %f, filtered value: %f\n", ball.GetX(), modelBallX.Value(filterBallX.State()))
				// fmt.Printf("Y: before: %f, filtered value: %f\n", ball.GetY(), modelBallY.Value(filterBallY.State()))
				// fmt.Printf("%f\n", modelBallX.Value(filterBallX.State()))
				// fmt.Printf("Y: %f\n", modelBallY.Value(filterBallY.State()))
				// sum++

				// if sum < 300 {
				// 	f.WriteString(fmt.Sprintf("%f", ball.GetX()) + "," + fmt.Sprintf("%f", math.Abs(modelBallX.Value(filterBallX.State()))) + "\n")
				// 	sumx += math.Abs(modelBallX.Value(filterBallX.State()))
				// 	sumy += math.Abs(modelBallY.Value(filterBallY.State()))
				// }
				// if sum == 300 {
				// 	averagex = sumx / 300
				// 	averagey = sumy / 300
				// }

				// fmt.Printf("X: %f\n", averagex)
				// fmt.Printf("Y: %f\n", averagey)

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

					// //Kalman Filter
					// err := filterRobotX[i].Update(t, modelRobotX[i].NewMeasurement(float64(robot.GetX())))
					// if err != nil {
					// 	log.Println(err)
					// }
					// err = filterRobotY[i].Update(t, modelRobotY[i].NewMeasurement(float64(robot.GetY())))
					// if err != nil {
					// 	log.Println(err)
					// }

					// filtered_robot_x[i] = float32(modelRobotX[i].Value(filterRobotX[i].State()))
					// filtered_robot_y[i] = float32(modelRobotY[i].Value(filterRobotY[i].State()))

					//Kalman Filter
					x3 := xh_k_1[i].At(2, 0)
					x4 := xh_k_1[i].At(3, 0)
					x5 := xh_k_1[i].At(4, 0)
					x6 := xh_k_1[i].At(5, 0)
					u_k_1[i].Set(0, 0, float64(controllerRobotVelocitys[i].X))
					u_k_1[i].Set(1, 0, float64(controllerRobotVelocitys[i].Y))
					u_k_1[i].Set(2, 0, float64(controllerRobotVelocitys[i].Angular))

					//サンプリング周期
					dt := 1e-2

					//観測値のtheta
					the := x3

					//予測ステップ
					R := mat.NewDense(3, 3, []float64{math.Cos(the), math.Sin(math.Pi), 0, -math.Sin(the), math.Cos(the), 0, 0, 0, 1})
					DR := mat.NewDense(3, 3, []float64{-math.Sin(the), math.Cos(the), 0, math.Cos(the), -math.Sin((the)), 0, 0, 0, 0})

					//xh_k_1の4番目~6番目を行列として取得
					v := mat.NewDense(3, 1, []float64{x4, x5, x6})

					//6x6の単位行列
					//zero1は6x3のゼロ行列
					zero1 := mat.NewDense(6, 3, []float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})
					zero2 := mat.NewDense(3, 3, []float64{0, 0, 0, 0, 0, 0, 0, 0, 0})
					RA := mat.NewDense(6, 3, nil)
					Ax := mat.NewDense(6, 6, nil)
					Bx := mat.NewDense(6, 3, nil)
					RA.Stack(R, A)
					Ax.Augment(zero1, RA)
					Bx.Stack(zero2, B)

					//6x6の単位行列
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
					Adxh_k_1.Product(Ad, xh_k_1[i])
					Bdu_k_1.Product(Bd, u_k_1[i]) //改良予定
					xhb_k.Add(Adxh_k_1, Bdu_k_1)
					Pb_k.Product(F_k_1, P_k_1[i], F_k_1.T())
					Pb_k.Add(Pb_k, Qv)

					//フィルタリングステップ
					inv := mat.NewDense(3, 3, nil)
					G_k := mat.NewDense(6, 3, nil)
					xh_k := mat.NewDense(6, 1, nil)
					y_k := mat.NewDense(3, 1, nil)
					y_kCxhb_k := mat.NewDense(3, 1, nil)
					G_ky_kCxhb_k := mat.NewDense(6, 1, nil)

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
					xh_k_1[i] = xh_k

					P_k := mat.NewDense(6, 6, nil)
					G_kC := mat.NewDense(6, 6, nil)
					IG_kC := mat.NewDense(6, 6, nil)
					G_kC.Product(G_k, C)
					IG_kC.Sub(I, G_kC)
					P_k.Product(IG_kC, Pb_k)
					P_k_1[i] = P_k

					filtered_robot_x[i] = float32(xh_k.At(0, 0))
					filtered_robot_y[i] = float32(xh_k.At(1, 0))
					filtered_robot_theta[i] = float32(xh_k.At(2, 0))

					// fmt.Printf("robot:x %d: before: %f, filtered value: %f\n", i, robot.GetX(), filtered_robot_x[i])
					// fmt.Printf("robot:y %d: before: %f, filtered value: %f\n", i, robot.GetY(), filtered_robot_y[i])
					// fmt.Printf("robot:theta %d: before: %f, filtered value: %f\n", i, robot.GetOrientation(), filtered_robot_theta[i])

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
