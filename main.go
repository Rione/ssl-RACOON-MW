package main

import (
	"fmt"
	"log"
	"net"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"github.com/golang/protobuf/proto"
)

//グローバル宣言
//更新時のみ置き換えるようにする
var robotstatus [16]*pb_gen.Robot_Status

func createRobotStatus(id int32, infrared bool, flatkick bool, chipkick bool) *pb_gen.Robot_Status {
	pe := &pb_gen.Robot_Status{
		RobotId:  &id,
		Infrared: &infrared,
		FlatKick: &flatkick,
		ChipKick: &chipkick,
	}
	return pe
}

func addRobotToRobots(robotstatus ...*pb_gen.Robot_Status) []*pb_gen.Robot_Status {
	Robotstatus := []*pb_gen.Robot_Status{}

	for _, robot := range robotstatus {
		if robot != nil {
			Robotstatus = append(Robotstatus, robot)
		}
	}

	return Robotstatus
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
	
		log.Printf("State change signal rceived from %s", addr)
	
		robotstatus[*packet.RobotId] = createRobotStatus(*packet.RobotId, *packet.Infrared, *packet.FlatKick, *packet.ChipKick)
	}
	chupdate <- true
}

func RunServer(chserver chan bool) {
	ipv4 := "127.0.0.1"
	port := "30011"
	addr := ipv4 + ":" + port

	fmt.Println("Send to:", addr)

	conn, err := net.Dial("udp", addr)
	CheckError(err)
	defer conn.Close()

	var counter int

	for {

		Robot_Status := addRobotToRobots(robotstatus[0], robotstatus[1], robotstatus[3],
			robotstatus[4], robotstatus[5], robotstatus[6],
			robotstatus[7], robotstatus[8], robotstatus[9],
			robotstatus[10], robotstatus[11], robotstatus[12])

		Robots_Status := &pb_gen.Robots_Status{
			RobotsStatus: Robot_Status,
		}

		Data, _ := proto.Marshal(Robots_Status)

		conn.Write([]byte(Data))

		time.Sleep(2 * time.Millisecond)

		counter = counter + 1

		fmt.Println(Robots_Status)
		fmt.Println(counter)
		log.Println("======================================")
	}
	chserver <- true
}

func main() {

	chupdate := make(chan bool)
	chserver := make(chan bool)

	go Update(chupdate)
	go RunServer(chserver)

	<-chupdate
	<-chserver
}

func CheckError(err error) {
	if err != nil {
		log.Fatal("Error: ", err)
	}
}
