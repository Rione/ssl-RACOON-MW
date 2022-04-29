package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"net"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"github.com/golang/protobuf/proto"
)

//グローバル宣言
//更新時のみ置き換えるようにする
var balldetect [16]bool
var visionwrapper [16]*pb_gen.SSL_WrapperPacket
var visiondetection [16]*pb_gen.SSL_DetectionFrame
var left_geo_goal_x float32
var left_geo_goal_y float32

var num_bluerobots int
var num_yellowrobots int

var bluerobots [16]*pb_gen.SSL_DetectionRobot
var yellowrobots [16]*pb_gen.SSL_DetectionRobot

var ball *pb_gen.SSL_DetectionBall

var maxcameras int

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
var degree_ball_robot [16]float32

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
	chupdate <- true
}

func RefereeReceive(chref chan bool) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.1"),
		Port: 10003,
	}

	serverConn, err := net.ListenMulticastUDP("udp", nil, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 2048)

	for {
		n, addr, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.State{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)

		log.Printf("Referee signal reveived from %s", addr)

		fmt.Println(packet)
	}
	chref <- true
}

func VisionReceive(chvision chan bool, port int, ourteam int, goalpos int) {
	maxcameras = 0

	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: port,
	}
	log.Printf("Receiving Vision Multicast at Port %d", port)
	serverConn, err := net.ListenMulticastUDP("udp", nil, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 2048)

	for i := 0; i < 60; i++ {
		n, addr, err := serverConn.ReadFromUDP(buf)
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

	for {
		for i := 0; i < maxcameras; i++ {
			n, _, err := serverConn.ReadFromUDP(buf)
			CheckError(err)

			packet := &pb_gen.SSL_WrapperPacket{}
			err = proto.Unmarshal(buf[0:n], packet)
			CheckError(err)

			visionwrapper[i] = packet
			visiondetection[i] = packet.Detection

			//log.Printf("Vision signal reveived from %s", addr)

			// Receive Geometry Data
			if packet.Geometry != nil {
				var lgtlp1x, lgtlp1y, lgtlp2x float32
				var lgblp2y float32
				for _, line := range packet.Geometry.GetField().GetFieldLines() {
					if line.GetName() == "LeftGoalTopLine" {
						lgtlp1x = line.GetP1().GetX()
						lgtlp1y = line.GetP1().GetY()
						lgtlp2x = line.GetP2().GetX()
					}
					if line.GetName() == "LeftGoalBottomLine" {
						lgblp2y = line.GetP2().GetY()
					}
				}
				left_geo_goal_x = (lgtlp1x + lgtlp2x) * 0.5
				left_geo_goal_y = (lgtlp1y + lgblp2y) * 0.5

				//Invert
				if goalpos == 1 {
					left_geo_goal_x = left_geo_goal_x * -1
				}
			}

			// Get Blue Robots
			num_bluerobots = 0
			for _, robot := range packet.Detection.GetRobotsBlue() {
				num_bluerobots++
				bluerobots[robot.GetRobotId()] = robot
				if goalpos == 1 {
					*robot.X = *robot.X * -1
				}
			}

			// Get Yellow Robots
			num_yellowrobots = 0
			for _, robot := range packet.Detection.GetRobotsYellow() {
				num_yellowrobots++
				yellowrobots[robot.GetRobotId()] = robot
				if goalpos == 1 {
					*robot.X = *robot.X * -1
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
				if goalpos == 1 {
					*maxconfball.X = *maxconfball.X * -1
				}

				ball = maxconfball

			}
		}
	}
	chvision <- true
}

func Observer(chobserver chan bool, ourteam int, goalpos int) {
	var pre_ball_X float32
	var pre_ball_Y float32
	var pre_robot_X [16]float32
	var pre_robot_Y [16]float32
	var pre_robot_Theta [16]float32

	var pre_enemy_X [16]float32
	var pre_enemy_Y [16]float32
	var pre_enemy_Theta [16]float32

	for {
		/////////////////////////////////////
		//
		//	BALL SPEED CALCULATION
		//
		/////////////////////////////////////
		if ball != nil && pre_ball_X != 0 {
			var ball_X float32 = *ball.X
			var ball_Y float32 = *ball.Y

			var ball_difference_X = ball_X - pre_ball_X
			var ball_difference_Y = ball_Y - pre_ball_Y

			if ball_difference_X != 0 && ball_difference_Y != 0 {
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

		/////////////////////////////////////
		//
		//	OUR ROBOT STATUS CALCULATION
		//
		/////////////////////////////////////
		var robot_difference_X [16]float32
		var robot_difference_Y [16]float32
		var robot_difference_Theta [16]float32
		var rdX64 [16]float64
		var rdY64 [16]float64

		for _, robot := range bluerobots {
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

				degree_ball_robot[i] = Calc_degree_normalize(Calc_degree(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY()) - robot.GetOrientation())
				distance_ball_robot[i] = Calc_distance(ball.GetX(), ball.GetY(), robot.GetX(), robot.GetY())

			}
		}

		/////////////////////////////////////
		//
		//	ENEMY ROBOT STATUS CALCULATION
		//
		/////////////////////////////////////
		var enemy_difference_X [16]float32
		var enemy_difference_Y [16]float32
		var enemy_difference_Theta [16]float32
		var edX64 [16]float64
		var edY64 [16]float64

		for _, enemy := range yellowrobots {
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

		//Wait 16ms (60FPS)
		time.Sleep(16 * 1000 * 1000)
	}

	<-chobserver
}

func createRobotInfo(i int, ourteam int) *pb_gen.Robot_Infos {
	var robotid uint32 = bluerobots[i].GetRobotId()
	var x float32 = bluerobots[i].GetX()
	var y float32 = bluerobots[i].GetY()
	var theta float32 = bluerobots[i].GetOrientation()

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
		DistanceBallRobot: &distance_ball_robot[i],
		DegreeBallRobot:   &degree_ball_robot[i],
		Speed:             &robot_speed[i],
		Slope:             &robot_slope[i],
		Intercept:         &robot_intercept[i],
		AngularVelocity:   &robot_angular_velocity[i],
		BallCatch:         &balldetect[i],
		Online:            &online,
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

func createEnemyInfo(i int, ourteam int) *pb_gen.Enemy_Infos {
	var robotid uint32 = yellowrobots[i].GetRobotId()
	var x float32 = yellowrobots[i].GetX()
	var y float32 = yellowrobots[i].GetY()
	var theta float32 = yellowrobots[i].GetOrientation()

	if ourteam == 1 {
		robotid = bluerobots[i].GetRobotId()
		x = bluerobots[i].GetX()
		y = bluerobots[i].GetY()
		theta = bluerobots[i].GetOrientation()
	}

	pe := &pb_gen.Enemy_Infos{
		RobotId:         &robotid,
		X:               &x,
		Y:               &y,
		Theta:           &theta,
		Speed:           &enemy_speed[i],
		Slope:           &enemy_slope[i],
		Intercept:       &enemy_intercept[i],
		AngularVelocity: &enemy_angular_velocity[i],
	}
	return pe
}

func createBallInfo() *pb_gen.Ball_Info {
	var x float32 = ball.GetX()
	var y float32 = ball.GetY()
	var slopedegree float32 = ball_slope_degree
	var slope float32 = ball_slope
	var intercept float32 = ball_intercept
	var speed float32 = ball_speed
	pe := &pb_gen.Ball_Info{
		X:            &x,
		Y:            &y,
		Slope_Degree: &slopedegree,
		Intercept:    &intercept,
		Speed:        &speed,
		Slope:        &slope,
	}
	return pe
}

func createGoalInfo() *pb_gen.Goal_Info {
	var x float32 = left_geo_goal_x
	var y float32 = left_geo_goal_y
	pe := &pb_gen.Goal_Info{
		X: &x,
		Y: &y,
	}
	return pe
}

func createOtherInfo() *pb_gen.Other_Infos {
	var numofcameras int32 = int32(maxcameras)
	pe := &pb_gen.Other_Infos{
		NumOfCameras: &numofcameras,
	}
	return pe
}

func createRefInfo() *pb_gen.Referee_Info {
	var command uint32 = 1
	var yellowcards uint32 = 1
	var redcards uint32 = 1
	pe := &pb_gen.Referee_Info{
		Command:     &command,
		YellowCards: &yellowcards,
		RedCards:    &redcards,
	}
	return pe
}

func addEnemyInfoToEnemyInfos(enemyinfo [16]*pb_gen.Enemy_Infos) []*pb_gen.Enemy_Infos {
	EnemyInfos := []*pb_gen.Enemy_Infos{}

	for _, enemy := range enemyinfo {
		if enemy != nil {
			EnemyInfos = append(EnemyInfos, enemy)
		}
	}

	return EnemyInfos
}

func RunServer(chserver chan bool, reportrate uint, ourteam int, goalpose int) {
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
		for _, robot := range bluerobots {
			robot_infos[robot.GetRobotId()] = createRobotInfo(int(robot.GetRobotId()), ourteam)
		}

		RobotInfos := addRobotInfoToRobotInfos(robot_infos)

		var enemy_infos [16]*pb_gen.Enemy_Infos
		for _, enemy := range yellowrobots {
			enemy_infos[enemy.GetRobotId()] = createEnemyInfo(int(enemy.GetRobotId()), ourteam)
		}

		EnemyInfos := addEnemyInfoToEnemyInfos(enemy_infos)

		BallInfo := createBallInfo()

		GoalInfo := createGoalInfo()
		RefereeInfo := createRefInfo()
		OtherInfo := createOtherInfo()

		RacoonMWPacket := &pb_gen.RacoonMW_Packet{
			OurRobots:   RobotInfos,
			EnemyRobots: EnemyInfos,
			Goal:        GoalInfo,
			Ball:        BallInfo,
			Referee:     RefereeInfo,
			Info:        OtherInfo,
		}

		fmt.Println(RacoonMWPacket)
		Data, _ := proto.Marshal(RacoonMWPacket)

		conn.Write([]byte(Data))

		time.Sleep(time.Duration(reportrate) * time.Millisecond)
		counter = counter + 1

	}
	chserver <- true
}

func main() {

	var (
		visionport = flag.Int("p", 10006, "Vision Multicast Port Number")
		ourteam    = flag.String("t", "blue", "Our Team (blue or yellow)")
		goalpos    = flag.String("g", "N", "Attack Direction Negative or Positive (N or P)")
		reportrate = flag.Uint("r", 16, "How often report to RACOON-AI? (milliseconds)")
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
		goalpos_n = 0
	} else {
		goalpos_n = 1
	}

	chupdate := make(chan bool)
	chserver := make(chan bool)
	chvision := make(chan bool)
	chobserver := make(chan bool)
	chref := make(chan bool)

	go Update(chupdate)
	go RunServer(chserver, *reportrate, ourteam_n, goalpos_n)
	go VisionReceive(chvision, *visionport, ourteam_n, goalpos_n)
	go Observer(chobserver, ourteam_n, goalpos_n)
	go RefereeReceive(chref)

	<-chupdate
	<-chserver
	<-chvision
	<-chobserver
	<-chref

}

func CheckError(err error) {
	if err != nil {
		log.Fatal("Error: ", err)
	}
}
