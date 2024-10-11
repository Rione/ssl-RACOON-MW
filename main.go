package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"net"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

func FPSCounter(chfps chan bool, ourteam int) {
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

			log.Printf("Estimated FPS:  %d FPS", fps)
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

func RunServer(chserver chan bool, reportrate uint, ourteam int, goalpose int, debug bool, simmode bool, ignore_ref_mismatch bool, match_mode bool, grsim_send_port int, goal_keeper uint, halfswitch int, teamname string) {
	ipv4 := NW_AI_IPADDR
	port := NW_AI_PORT
	port_controller := NW_AI_PORT_CONTROLLER
	port_gui := NW_AI_PORT_GUI
	addr := ipv4 + ":" + port
	addr_controller := ipv4 + ":" + port_controller
	addr_gui := ipv4 + ":" + port_gui

	ip_other := NW_OUT_MCAST_IPADDR
	port_other := NW_OUT_MCAST_PORT
	addr_other := ip_other + ":" + port_other

	log.Println("Send to:", addr)

	conn, err := net.Dial("udp", addr)
	CheckError(err)
	conn_controller, err := net.Dial("udp", addr_controller)
	CheckError(err)
	conn_gui, err := net.Dial("udp", addr_gui)
	CheckError(err)

	conn_other, err := net.Dial("udp", addr_other)
	CheckError((err))

	defer conn.Close()
	defer conn_controller.Close()
	defer conn_gui.Close()

	defer conn_other.Close()

	var counter int

	for {

		//チームカラー検査
		if teamcolor_from_ref != -1 && match_mode {
			ourteam = teamcolor_from_ref
		}

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
		RefereeInfo := createRefInfo(ourteam, goalpose, ignore_ref_mismatch, goal_keeper, match_mode, teamname)
		OtherInfo := createOtherInfo(int32(goalpose), ourteam, match_mode, grsim_send_port, simmode, halfswitch)

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
			conn_controller.Write([]byte(Data))
			conn_gui.Write([]byte(Data))
			conn_other.Write([]byte(Data))
		}

		time.Sleep(time.Duration(reportrate) * time.Millisecond)
		counter = counter + 1

	}
	chserver <- true
}

func main() {

	var (
		visionport           = flag.Int("p", 10006, "Vision Multicast Port Number. Force 10006 when match mode is true")
		gcport               = flag.Int("gcp", 10003, "Game Controller Listen Port Number (Default 10003) Force 10003 when match mode is true")
		ourteam              = flag.String("t", "blue", "Our Team (blue or yellow). Disable when match mode is true")
		goalpos              = flag.String("g", "N", "Attack Direction(Enemy goal) Negative or Positive (N or P)")
		reportrate           = flag.Uint("r", 16, "How often report to RACOON-AI? (milliseconds)")
		debug                = flag.Bool("d", false, "Show All Send Packet")
		simmode              = flag.Bool("s", false, "Simulation Mode (Emulate Ball Sensor). Disable when match mode is true")
		replay               = flag.Bool("replay", false, "Replay All Packet")
		halfswitch           = flag.String("c", "F", "Where to use (N, P, F) F to Full. Disable when match mode is true")
		ballmovethreshold    = flag.Float64("b", 1000, "Ball Detect Threshold (Default 1000")
		nw_robot             = flag.String("rif", "none", "NW Robot Update Interface Name (ex. en0)")
		nw_vision            = flag.String("vif", "none", "NW Vision and Referee receive Interface Name (ex. en1)")
		ignore_ref_mismatch  = flag.Bool("igref", false, "Ignore Referee Team Color & Attack Direction Mismatch Errors. Disable when match mode is true")
		match_mode           = flag.Bool("m", false, "Match Mode (Disable Some Options! Most option get from GC)")
		grsim_send_port      = flag.Int("grsimport", 20011, "grSim Command Listen Port Number")
		goal_keeper          = flag.Uint("gk", 0, "Goal Keeper ID (0-15) Disable when match mode is true")
		initial_variance     = flag.Float64("iv", 100, "Initial Variance for Ball Kalman Filter(Default 100)")
		process_variance     = flag.Float64("pv", 0.1, "Process Variance for Ball Kalman Filter(Default 0.1)")
		observation_variance = flag.Float64("ov", 0.18, "Observation Variance for Ball Kalman Filter(Default 0.18)")
		teamname             = flag.String("tn", "Ri-one", "Team Name (Default Ri-one)")
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

	if *match_mode {
		*visionport = 10006
		*gcport = 10003
		*ignore_ref_mismatch = false
		*simmode = false
		*halfswitch = "F"
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

	if *match_mode {
		log.Println("=============================================")
		log.Println("[RACOON-MW] Match Mode Enabled! Some Options are Disabled! GOOD LUCK!")
		log.Println("=============================================")
	}
	chupdate := make(chan bool)
	chserver := make(chan bool)
	chvision := make(chan bool)
	chref := make(chan bool)
	chfps := make(chan bool)
	chvisrobot := make(chan bool)
	chctrlfb := make(chan bool)
	chrobotip := make(chan bool)

	go Update(chupdate)
	go RunServer(chserver, *reportrate, ourteam_n, goalpos_n, *debug, *simmode, *ignore_ref_mismatch, *match_mode, *grsim_send_port, *goal_keeper, halfswitch_n, *teamname)
	go VisionReceive(chvision, *visionport, ourteam_n, goalpos_n, *simmode, *replay, halfswitch_n, *match_mode, *initial_variance, *process_variance, *observation_variance)
	go CheckVisionRobot(chvisrobot)
	go FPSCounter(chfps, ourteam_n)
	go RefereeClient(chref, *gcport)
	go controllerFeedback(chctrlfb)
	go RobotIPList(chrobotip)

	<-chupdate
	<-chserver
	<-chvision
	<-chref
	<-chfps
	<-chvisrobot
	<-chctrlfb
	<-chrobotip

}

func CheckError(err error) {
	if err != nil {
		log.Println("Error: ", err)
	}
}
