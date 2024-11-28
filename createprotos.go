package main

import (
	"log"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
)

func createRobotInfo(i int, ourteam int, simmode bool) *pb_gen.Robot_Infos {
	var robotid uint32
	var x float32
	var y float32
	var theta float32
	if ourteam == 0 {
		robotid = bluerobots[i].GetRobotId()
		x = filtered_robot_x[i]
		y = filtered_robot_y[i]
		theta = filtered_robot_theta[i]
	} else {
		robotid = yellowrobots[i].GetRobotId()
		x = filtered_robot_x[i]
		y = filtered_robot_y[i]
		theta = filtered_robot_theta[i]
	}
	var diffx float32 = robot_difference_X[i]
	var diffy float32 = robot_difference_Y[i]
	var difftheta float32 = robot_difference_Theta[i]

	var batt float32 = battery_voltage[i]
	var online bool = robot_online[i]
	if simmode {
		batt = 16.5
		online = true
	}
	var cap_power uint32 = uint32(cap_power[i])
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
		CapPower:          &cap_power,
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
		var robotid uint32 = uint32(i)
		var x float32
		var y float32
		var theta float32
		var diffx float32 = enemy_difference_X[i]
		var diffy float32 = enemy_difference_Y[i]
		var difftheta float32 = enemy_difference_Theta[i]

		x = filtered_enemy_x[i]
		y = filtered_enemy_y[i]
		theta = filtered_enemy_theta[i]

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
	var x float32 = filtered_ball_x
	var y float32 = filtered_ball_y
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
		IsVisible:   &flag_ball,
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

func createOtherInfo(goalpos_n int32, ourteam int, match_mode bool, grsim_send_port int, simmode bool, halfswitch int) *pb_gen.Other_Infos {
	var numofcameras int32 = int32(maxcameras)
	var numofourrobots int32
	var numofenemyrobots int32
	var teamcolor int
	var isYellow bool
	var isHalfcourt bool
	var grSimSendPort uint32 = uint32(grsim_send_port)
	var attackdir int32
	for i := 0; i < 16; i++ {
		if ourrobot_is_visible[i] {
			numofourrobots++
		}
		if enemyrobot_is_visible[i] {
			numofenemyrobots++
		}
	}

	isReal := !simmode

	if match_mode {
		isReal = true
		teamcolor = teamcolor_from_ref
		attackdir = int32(attack_direction_from_ref)
	} else {
		teamcolor = ourteam
		attackdir = goalpos_n * -1
	}

	if teamcolor == 1 { //YELLOW
		isYellow = true
	} else { //BLUE
		isYellow = false
	}

	if halfswitch == 1 || halfswitch == -1 {
		isHalfcourt = true
	} else {
		isHalfcourt = false
	}

	pe := &pb_gen.Other_Infos{
		NumOfCameras:           &numofcameras,
		NumOfOurRobots:         &numofourrobots,
		NumOfEnemyRobots:       &numofenemyrobots,
		Secperframe:            &secperframe,
		IsVisionRecv:           &isvisionrecv,
		AttackDirection:        &attackdir,
		IsBallMoving:           &is_ball_moving,
		IsReal:                 &isReal,
		IsTeamYellow:           &isYellow,
		IsHalfCourt:            &isHalfcourt,
		IsMatchmode:            &match_mode,
		GrsimCommandListenPort: &grSimSendPort,
	}
	return pe
}

func createRefInfo(ourteam int, attackdirection int, ignore_ref_mismatch bool, goal_keeper uint, match_mode bool, teamname string) *pb_gen.Referee_Info {
	var yellowcards uint32
	var redcards uint32
	var command *pb_gen.Referee_Info_Command
	var teaminfo_our *pb_gen.Referee_Info_TeamInfo
	var teaminfo_their *pb_gen.Referee_Info_TeamInfo
	var stage *pb_gen.Referee_Info_Stage
	var next_command *pb_gen.Referee_Info_Command
	var bpX float32
	var bpY float32
	var gameevent []*pb_gen.GameEvent

	goal_keeper_id := uint32(goal_keeper)

	if ref_command != nil {
		command = (*pb_gen.Referee_Info_Command)(ref_command.Command)
		stage = (*pb_gen.Referee_Info_Stage)(ref_command.Stage)
		next_command = (*pb_gen.Referee_Info_Command)(ref_command.NextCommand)
		bpX = ref_command.GetDesignatedPosition().GetX()
		bpY = ref_command.GetDesignatedPosition().GetY()
		gameevent = ref_command.GetGameEvents()
		if ourteam == 0 {
			yellowcards = ref_command.Blue.GetYellowCards()
			redcards = ref_command.Blue.GetRedCards()
			teaminfo_our = (*pb_gen.Referee_Info_TeamInfo)(ref_command.Blue)
			teaminfo_their = (*pb_gen.Referee_Info_TeamInfo)(ref_command.Yellow)
		} else {
			yellowcards = ref_command.Yellow.GetYellowCards()
			redcards = ref_command.Yellow.GetRedCards()
			teaminfo_our = (*pb_gen.Referee_Info_TeamInfo)(ref_command.Yellow)
			teaminfo_their = (*pb_gen.Referee_Info_TeamInfo)(ref_command.Blue)
		}

		if !ignore_ref_mismatch && !match_mode {
			// Check if the team color is correct
			if ourteam == 0 && ref_command.GetYellow().GetName() == teamname {
				log.Println("[MW WARNING!!] INCORRECT TEAM COLOR! Referee says (", teamname, "== YELLOW)")
			} else if ourteam == 1 && ref_command.GetBlue().GetName() == teamname {
				log.Println("[MW WARNING!!] INCORRECT TEAM COLOR! Referee says (", teamname, "== Blue)")
			}

			// Check if the attack direction is correct
			if ourteam == 0 && *ref_command.BlueTeamOnPositiveHalf && attackdirection == 1 {
				log.Println("[MW WARNING!!] INCORRECT ATTACK DIRECTION! Referee says (BlueTeamOnPositiveHalf == true)")
			} else if ourteam == 1 && !*ref_command.BlueTeamOnPositiveHalf && attackdirection == 1 {
				log.Println("[MW WARNING!!] INCORRECT ATTACK DIRECTION! Referee says (BlueTeamOnPositiveHalf == false)")
			}

			if ourteam == 0 && !*ref_command.BlueTeamOnPositiveHalf && attackdirection == -1 {
				log.Println("[MW WARNING!!] INCORRECT ATTACK DIRECTION! Referee says (BlueTeamOnPositiveHalf == true)")
			} else if ourteam == 1 && *ref_command.BlueTeamOnPositiveHalf && attackdirection == -1 {
				log.Println("[MW WARNING!!] INCORRECT ATTACK DIRECTION! Referee says (BlueTeamOnPositiveHalf == false)")
			}
		}

		if match_mode {
			// Get team color from referee
			if ref_command.GetYellow().GetName() == teamname && ref_command.GetBlue().GetName() == teamname {
				log.Println("[MW WARNING!!] Team Name is Both ", teamname, "! Referee says (", teamname, "== YELLOW) and (", teamname, "== BLUE). Forced to set team color to BLUE.")
			}
			if ref_command.GetBlue().GetName() == teamname {
				teamcolor_from_ref = 0
				goal_keeper_id = ref_command.GetBlue().GetGoalkeeper()
				if ref_command.GetBlueTeamOnPositiveHalf() == true {
					attack_direction_from_ref = -1
				} else {
					attack_direction_from_ref = 1
				}
			} else if ref_command.GetYellow().GetName() == teamname {
				teamcolor_from_ref = 1
				goal_keeper_id = ref_command.GetYellow().GetGoalkeeper()
				if ref_command.GetBlueTeamOnPositiveHalf() == true {
					attack_direction_from_ref = 1
				} else {
					attack_direction_from_ref = -1
				}
			}
		}

	} else {
		yellowcards = 0
		redcards = 0
	}

	pe := &pb_gen.Referee_Info{
		Command:         command,
		Stage:           stage,
		TeaminfoOur:     teaminfo_our,
		TeaminfoTheir:   teaminfo_their,
		YellowCards:     &yellowcards,
		RedCards:        &redcards,
		Event:           gameevent,
		PreCommand:      last_command,
		NextCommand:     next_command,
		BallPlacementX:  &bpX,
		BallPlacementY:  &bpY,
		OurGoalkeeperId: &goal_keeper_id,
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
