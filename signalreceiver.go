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

		packet := &pb_gen.Robot_Status{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)
		balldetect[packet.GetRobotId()] = packet.GetInfrared()
		if robot_online_count[packet.GetRobotId()] < 5 {
			robot_online_count[packet.GetRobotId()] += 1
		}
		//Assign IP address to robot ID
		if robot_ipaddr[packet.GetRobotId()] != addr.IP.String() {
			robot_ipaddr[packet.GetRobotId()] = addr.IP.String()
			log.Println("Robot ID", packet.GetRobotId(), " is associated with ", addr.IP.String())
		}

		battery_voltage[packet.GetRobotId()] = float32(packet.GetBatteryVoltage()) / 10
		cap_power[packet.GetRobotId()] = uint8(packet.GetCapPower())

	}
}

// Host a web server to display the robot IP addresses
func RobotIPList(chrobotip chan bool) {
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		//Set the content type to HTML
		fmt.Fprintf(w, "<h1>The RACOON Web Console</h1>")
		fmt.Fprintf(w, "<html><head><title>The RACOON Web Console</title></head><body><table border=\"1\">")
		fmt.Fprintf(w, "<h2>Robot IP List</h2><tr><th>Robot ID</th><th>Associated IP Address</th><th>Beep</th></tr>")
		for i := 0; i < 16; i++ {
			buzzurl := fmt.Sprintf("location.href=\"http://%s:9191/buzzer/tone/%s/1000\"", robot_ipaddr[i], strconv.Itoa(i))
			fmt.Fprintf(w, "<tr><td>%d</td><td>%s</td><td><button onclick='%s'>Beep</button></td></tr>", i, robot_ipaddr[i], buzzurl)
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
