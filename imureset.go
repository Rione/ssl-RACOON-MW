package main

import (
	"log"
	"net"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

func IMUReset(chimu chan bool, ourteam int, simmode bool, ignoreimureset bool) {
	var isInBallPlacement bool = false
	for {
		if simmode {
			for i := 0; i < 16; i++ {
				robot_online[i] = true
			}
		}
		if isvisionrecv {
			for i := 0; i < 3; i++ {
				var signal []*pb_gen.GrSim_Robot_Command

				for i := 0; i < 16; i++ {
					if robot_online[i] {
						signal = append(signal, createIMUSignal(uint32(i), ourteam))
					}
				}

				//check if in ball placement

				if ref_command != nil {
					if ourteam == 0 && ref_command.GetCommand() == pb_gen.Referee_BALL_PLACEMENT_BLUE {
						isInBallPlacement = true
					} else if ourteam == 1 && ref_command.GetCommand() == pb_gen.Referee_BALL_PLACEMENT_YELLOW {
						isInBallPlacement = true
					} else {
						isInBallPlacement = false
					}
				}

				command := addIMUSignalToIMUSignals(signal)
				packet := &pb_gen.GrSim_Packet{
					Commands: command,
				}
				//log.Println(packet)
				marshalpacket, _ := proto.Marshal(packet)
				if !isInBallPlacement && !ignoreimureset {
					for i := 0; i < 16; i++ {
						if robot_online[i] && ourrobot_is_visible[i] {
							ipv4 := robot_ipaddr[i]
							port := "20011"
							addr := ipv4 + ":" + port

							conn, err := net.Dial("udp", addr)
							CheckError(err)
							conn.Write(marshalpacket)
							time.Sleep(1 * time.Millisecond)
							conn.Write(marshalpacket)
							// log.Println("IMU Reset Signal Sent to Robot ID: ", i)
						}
					}
				} else {
					log.Println("IMU Reset Ignored due to Ball Placement Mode or Disable flag")
				}
			}

		}
		imu_reset_time = time.Now()
		time.Sleep(IMU_RESET_INTERVAL)
	}

}
