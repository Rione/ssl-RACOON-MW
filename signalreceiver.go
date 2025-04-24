package main

import (
	"fmt"
	"log"
	"net"
	"net/http"
	"strconv"
	"time"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

var cap_power [16]uint8

func Update(chupdate chan bool) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.69.4"),
		Port: 16941,
	}
	interfacename, _ := net.InterfaceByName(NW_ROBOT_UPDATE_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Robot Update Signal NW Interface Name is wrong! Trying system-default interface!")
	}
	serverConn, err := net.ListenMulticastUDP("udp", interfacename, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 1024)

	for {
		n, addr, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.PiToMw{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)
		balldetect[packet.RobotsStatus.GetRobotId()] = packet.RobotsStatus.GetInfrared()
		if robot_online_count[packet.RobotsStatus.GetRobotId()] < 5 {
			robot_online_count[packet.RobotsStatus.GetRobotId()] += 1
		}
		//Assign IP address to robot ID
		if robot_ipaddr[packet.RobotsStatus.GetRobotId()] != addr.IP.String() {
			robot_ipaddr[packet.RobotsStatus.GetRobotId()] = addr.IP.String()
			log.Println("Robot ID", packet.RobotsStatus.GetRobotId(), " is associated with ", addr.IP.String())
		}

		battery_voltage[packet.RobotsStatus.GetRobotId()] = float32(*packet.RobotsStatus.BatteryVoltage) / 10
		cap_power[packet.RobotsStatus.GetRobotId()] = uint8(packet.RobotsStatus.GetCapPower())
		is_ball_exit[packet.RobotsStatus.GetRobotId()] = packet.BallStatus.GetIsBallExit()
		ball_camera_X[packet.RobotsStatus.GetRobotId()] = packet.BallStatus.GetBallCameraX()
		ball_camera_Y[packet.RobotsStatus.GetRobotId()] = packet.BallStatus.GetBallCameraY()
		adjustment[packet.RobotsStatus.GetRobotId()].Max_Threshold = packet.Ball.GetMaxThreshold()
		adjustment[packet.RobotsStatus.GetRobotId()].Min_Threshold = packet.Ball.GetMinThreshold()
		adjustment[packet.RobotsStatus.GetRobotId()].Ball_Detect_Radius = packet.Ball.GetBallDetectRadius()
		adjustment[packet.RobotsStatus.GetRobotId()].Circularity_Threshold = packet.Ball.GetCircularityThreshold()
	}
}

// Host a web server to display the robot IP addresses
func RobotIPList(chrobotip chan bool) {
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		//Set the content type to HTML
		fmt.Fprintf(w, "<h1>The RACOON Web Console</h1>")
		fmt.Fprintf(w, "<html><head><title>The RACOON Web Console</title></head><body><table border=\"1\">")
		fmt.Fprintf(w, "<tr><th>Robot ID</th><th>Associated IP Address</th><th>Beep</th><th>Min Threshold</th><th>Max Threshold</th><th>Radius</th><th>Circularity</th></tr>")
		for i := 0; i < 16; i++ {
			buzzurl := fmt.Sprintf("location.href=\"http://%s:9191/buzzer/tone/%s/1000\"", robot_ipaddr[i], strconv.Itoa(i))
			fmt.Fprintf(w, "<tr><td>%d</td><td>%s</td><td><button onclick='%s'>Beep</button></td><td>%s</td><td>%s</td><td>%d</td><td>%f</td></tr>", i, robot_ipaddr[i], buzzurl, adjustment[i].Min_Threshold, adjustment[i].Max_Threshold, adjustment[i].Ball_Detect_Radius, adjustment[i].Circularity_Threshold)
		}
		fmt.Fprintf(w, "</table>")
		//Date and Time
		//Vision Status
		fmt.Fprintf(w, "<h2>Vision Status: %t</h2>", isvisionrecv)
		fmt.Fprintf(w, "<p>Generated at %s</p>", time.Now())
		fmt.Fprintf(w, "</body></html>")

	})
	log.Fatal(http.ListenAndServe(":8080", nil))

	<-chrobotip
}
