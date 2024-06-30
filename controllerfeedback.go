package main

import (
	"log"
	"net"
	"sync"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

// x, y, theta の速度成分を構造体に格納
type RobotVelocity struct {
	X       float32
	Y       float32
	Angular float32
}

var controllerRobotVelocitys [16]RobotVelocity

// mutex
var mutex sync.Mutex

func controllerFeedback(chctrlfb chan bool) {
	// コントローラからの通信をUDPで待ち受ける
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("127.0.0.1"),
		Port: 56940,
	}

	serverConn, err := net.ListenUDP("udp", serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 1024)

	for {
		n, _, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.GrSim_Packet{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)

		mutex.Lock()

		for i := range controllerRobotVelocitys {
			controllerRobotVelocitys[i] = RobotVelocity{}
		}
		for _, command := range packet.Commands.RobotCommands {
			if command.GetId() > 0 || command.GetId() < 16 {
				controllerRobotVelocitys[command.GetId()] = RobotVelocity{
					X:       command.GetVeltangent(),
					Y:       command.GetVelnormal(),
					Angular: command.GetVelangular(),
				}
			} else {
				log.Println("[MW-Controller-Feedback] Invalid robot id: ", command.GetId())
			}
		}
		mutex.Unlock()
	}

	chctrlfb <- true
}
