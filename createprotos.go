package main

import (
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
)

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

	var batt float32 = battery_voltage[i]
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
	var teaminfo_our *pb_gen.Referee_Info_TeamInfo
	var teaminfo_their *pb_gen.Referee_Info_TeamInfo
	var stage *pb_gen.Referee_Info_Stage
	var next_command *pb_gen.Referee_Info_Command
	var bpX float32
	var bpY float32
	var gameevent []*pb_gen.GameEvent

	if ref_command != nil {
		command = (*pb_gen.Referee_Info_Command)(ref_command.Command)
		stage = (*pb_gen.Referee_Info_Stage)(ref_command.Stage)
		next_command = (*pb_gen.Referee_Info_Command)(ref_command.NextCommand)
		bpX = ref_command.GetDesignatedPosition().GetX()
		bpY = ref_command.GetDesignatedPosition().GetY()
		// gameevent = ref_command.GetGameEvents()
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

	} else {
		yellowcards = 0
		redcards = 0
	}

	pe := &pb_gen.Referee_Info{
		Command:        command,
		Stage:          stage,
		TeaminfoOur:    teaminfo_our,
		TeaminfoTheir:  teaminfo_their,
		YellowCards:    &yellowcards,
		RedCards:       &redcards,
		Event:          gameevent,
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
