package main

import (
	"log"
	"math"

	"net"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"

	"google.golang.org/protobuf/proto"
)

func TrackerReceive(chvision chan bool, port int, ourteam int, goalpos int, simmode bool, replay bool, halfswitch_n int, matchmode bool, initial_variance float64, process_variance float64, observation_variance float64, port_tracker int) {

	//get geometry

	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: port,
	}

	interfacename, _ := net.InterfaceByName(NW_VISION_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Vision Signal NW Interface Name is wrong! Trying system-default interface!")
	}

	log.Printf("Receiving Vision Multicast at Port %d", port)
	serverConn, _ := net.ListenMulticastUDP("udp", interfacename, serverAddr)
	defer serverConn.Close()

	buf := make([]byte, 4096)

	for {
		n, _, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.SSL_WrapperPacket{}

		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)

		if packet.Geometry != nil { //Geometry Data
			geometrydata = packet.Geometry
			break
		}
	}

	maxcameras = 0
	framecounter = 0
	serverAddr = &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.2"),
		Port: port_tracker,
	}

	interfacename, _ = net.InterfaceByName(NW_VISION_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Tracker Signal NW Interface Name is wrong! Trying system-default interface!")
	}

	log.Printf("Receiving Tracker Multicast at Port %d", port_tracker)
	serverConn, _ = net.ListenMulticastUDP("udp", interfacename, serverAddr)
	defer serverConn.Close()

	buf = make([]byte, 4096)

	maxcameras = 1

	log.Printf("MAX CAMERAS: %d", maxcameras)
	log.Printf("Receive Loop and Send Start: Tracker addr %s", serverAddr)

	var pre_ourrobots [16]*pb_gen.TrackedRobot
	var pre_enemyrobots [16]*pb_gen.TrackedRobot
	var pre_ball *pb_gen.TrackedBall
	var pre_packet *pb_gen.TrackerWrapperPacket

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

		flag_ball = false

		var n int
		var err error

		n, _, err = serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.TrackerWrapperPacket{}

		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)
		if pre_packet != nil && packet.TrackedFrame.GetTimestamp()-pre_packet.TrackedFrame.GetTimestamp() == 0 {
			continue
		}

		pre_packet = packet

		var lgtlp1x, lgtlp1y, lgtlp2x float32
		var lgblp2y float32
		for _, line := range geometrydata.GetField().GetFieldLines() {
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

		centercircleradius = geometrydata.GetField().GetFieldArcs()[0].GetRadius()

		//Invert
		if goalpos == -1 {
			left_geo_goal_x = left_geo_goal_x * -1
		}

		// Get Robots
		for _, robot := range packet.TrackedFrame.GetRobots() {
			if robot.GetRobotId().GetTeam() == pb_gen.Team_BLUE {
				switch halfswitch_n {
				case 0:
					bluerobots_tracked[int(robot.GetRobotId().GetId())] = robot
					visible_in_vision_b[int(robot.GetRobotId().GetId())] = true

				case 1:
					if robot.GetPos().GetX() > 0 {
						bluerobots_tracked[robot.GetRobotId().GetId()] = robot
						visible_in_vision_b[robot.GetRobotId().GetId()] = true
					}

				case -1:
					if robot.GetPos().GetX() <= 0 {
						bluerobots_tracked[robot.GetRobotId().GetId()] = robot
						visible_in_vision_b[robot.GetRobotId().GetId()] = true
					}
				}
			} else {
				switch halfswitch_n {
				case 0:
					yellowrobots_tracked[int(robot.GetRobotId().GetId())] = robot
					visible_in_vision_y[int(robot.GetRobotId().GetId())] = true

				case 1:
					if robot.GetPos().GetX() > 0 {
						yellowrobots_tracked[robot.GetRobotId().GetId()] = robot
						visible_in_vision_y[robot.GetRobotId().GetId()] = true
					}

				case -1:
					if robot.GetPos().GetX() <= 0 {
						yellowrobots_tracked[robot.GetRobotId().GetId()] = robot
						visible_in_vision_y[robot.GetRobotId().GetId()] = true
					}
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

		// Get Ball
		var visibility float32
		var ball_exist bool = false
		for _, ball := range packet.TrackedFrame.GetBalls() {
			if ball.GetVisibility() > visibility {
				ball_tracked = ball
				ball_exist = true
			}
		}
		if ball_exist {
			flag_ball = true
		} else {
			flag_ball = false
		}

		var ourrobots [16]*pb_gen.TrackedRobot
		var enemyrobots [16]*pb_gen.TrackedRobot

		if ourteam == 0 {
			ourrobots = bluerobots_tracked
			enemyrobots = yellowrobots_tracked
		} else {
			ourrobots = yellowrobots_tracked
			enemyrobots = bluerobots_tracked
		}

		// Calculate Ball Speed
		if ball_tracked != nil {
			ball_speed_x := ball_tracked.Vel.GetX()
			ball_speed_y := ball_tracked.Vel.GetY()
			ball_speed = float32(math.Sqrt(float64(ball_speed_x*ball_speed_x + ball_speed_y*ball_speed_y)))
			if pre_ball != nil {
				ball_difference_X = ball_tracked.Pos.GetX() - pre_ball.Pos.GetX()
				ball_difference_Y = ball_tracked.Pos.GetY() - pre_ball.Pos.GetY()
			} else {
				ball_difference_X = 0
				ball_difference_Y = 0
			}

			if ball_difference_X != 0 || ball_difference_Y != 0 {
				ball_slope = ball_difference_Y / ball_difference_X
				bdX64 := float64(ball_difference_X)
				bdY64 := float64(ball_difference_Y)
				ball_slope_degree = float32(math.Atan2(bdY64, bdX64))
				ball_intercept = ball_tracked.Pos.GetY() - (ball_slope * ball_tracked.Pos.GetX())
			} else {
				ball_slope_degree = 0.0
				ball_intercept = 0.0
			}

			ball_difference_X = ball_tracked.Vel.GetX()
			ball_difference_Y = ball_tracked.Vel.GetY()

		}

		// Calculate Robot Speed
		for i := 0; i < 16; i++ {
			if ourrobots[i] != nil {
				robot_speed_x := ourrobots[i].Vel.GetX() * 1000
				robot_speed_y := ourrobots[i].Vel.GetY() * 1000
				robot_speed[i] = float32(math.Sqrt(float64(robot_speed_x*robot_speed_x + robot_speed_y*robot_speed_y)))

				if pre_ourrobots[i] != nil {
					robot_difference_X[i] = (ourrobots[i].Pos.GetX() - pre_ourrobots[i].Pos.GetX()) * 1000
					robot_difference_Y[i] = (ourrobots[i].Pos.GetY() - pre_ourrobots[i].Pos.GetY()) * 1000
					robot_difference_Theta[i] = ourrobots[i].GetOrientation() - pre_ourrobots[i].GetOrientation()
				} else {
					robot_difference_X[i] = 0
					robot_difference_Y[i] = 0
					robot_difference_Theta[i] = 0
				}

				robot_angular_velocity[i] = ourrobots[i].GetVelAngular()

				if robot_difference_Y[i] != 0 || robot_difference_X[i] != 0 {
					robot_slope[i] = robot_difference_Y[i] / robot_difference_X[i]
					robot_intercept[i] = ourrobots[i].Pos.GetY() - (robot_slope[i] * ourrobots[i].Pos.GetX())
				} else {
					robot_slope[i] = 0.0
					robot_intercept[i] = ourrobots[i].Pos.GetY()
					robot_speed[i] = 0.0
				}

				robot_difference_Theta[i] = ourrobots[i].GetVelAngular()
				robot_difference_X[i] = ourrobots[i].Vel.GetX() * 1000
				robot_difference_Y[i] = ourrobots[i].Vel.GetY() * 1000

			}
			if enemyrobots[i] != nil {
				enemy_speed_x := enemyrobots[i].Vel.GetX() * 1000
				enemy_speed_y := enemyrobots[i].Vel.GetY() * 1000
				enemy_speed[i] = float32(math.Sqrt(float64(enemy_speed_x*enemy_speed_x + enemy_speed_y*enemy_speed_y)))
				if pre_enemyrobots[i] != nil {
					enemy_difference_Theta[i] = enemyrobots[i].GetOrientation() - pre_enemyrobots[i].GetOrientation()
					enemy_difference_X[i] = enemyrobots[i].Pos.GetX() - pre_enemyrobots[i].Pos.GetX()*1000
					enemy_difference_Y[i] = enemyrobots[i].Pos.GetY() - pre_enemyrobots[i].Pos.GetY()*1000
				} else {
					enemy_difference_Theta[i] = 0
					enemy_difference_X[i] = 0
					enemy_difference_Y[i] = 0
				}
				enemy_angular_velocity[i] = enemyrobots[i].GetVelAngular()

				if enemy_difference_Y[i] != 0 || enemy_difference_X[i] != 0 {
					enemy_slope[i] = enemy_difference_Y[i] / enemy_difference_X[i]
					enemy_intercept[i] = enemyrobots[i].Pos.GetY() - (enemy_slope[i] * enemyrobots[i].Pos.GetX())
				} else {
					enemy_slope[i] = 0.0
					enemy_intercept[i] = enemyrobots[i].Pos.GetY()
					enemy_speed[i] = 0.0
				}

				enemy_difference_Theta[i] = enemyrobots[i].GetVelAngular()
				enemy_difference_X[i] = enemyrobots[i].Vel.GetX() * 1000
				enemy_difference_Y[i] = enemyrobots[i].Vel.GetY() * 1000
			}
		}

		// Calc Distance Ball Robot
		for i := 0; i < 16; i++ {
			if ourrobots[i] != nil {
				robot_x := bluerobots_tracked[i].GetPos().GetX()
				robot_y := bluerobots_tracked[i].GetPos().GetY()
				ball_x := ball_tracked.GetPos().GetX()
				ball_y := ball_tracked.GetPos().GetY()
				distance_ball_robot[i] = float32(math.Sqrt(float64((robot_x-ball_x)*(robot_x-ball_x)+(robot_y-ball_y)*(robot_y-ball_y)))) * 1000
				radian_ball_robot[i] = Calc_degree_normalize(Calc_degree(ball_x, ball_y, robot_x, robot_y) - bluerobots_tracked[i].GetOrientation())
			}
		}

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

		pre_enemyrobots = enemyrobots
		pre_ourrobots = ourrobots
		pre_ball = ball_tracked

		framecounter++

	}
	chvision <- true
}
