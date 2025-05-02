package main

import (
	"fmt"
	"io"
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
		is_detect_photo_sensor[packet.RobotsStatus.GetRobotId()] = packet.RobotsStatus.GetIsDetectPhotoSensor()
		is_detect_dribbler_sensor[packet.RobotsStatus.GetRobotId()] = packet.RobotsStatus.GetIsDetectDribblerSensor()
		is_new_dribbler[packet.RobotsStatus.GetRobotId()] = packet.RobotsStatus.GetIsNewDribbler()

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
		if packet.BallStatus.GetBallCameraX() != 0 {
			ball_camera_X[packet.RobotsStatus.GetRobotId()] = packet.BallStatus.GetBallCameraX()
			ball_camera_Y[packet.RobotsStatus.GetRobotId()] = packet.BallStatus.GetBallCameraY()
		}
		adjustment[packet.RobotsStatus.GetRobotId()].Max_Threshold = packet.Ball.GetMaxThreshold()
		adjustment[packet.RobotsStatus.GetRobotId()].Min_Threshold = packet.Ball.GetMinThreshold()
		adjustment[packet.RobotsStatus.GetRobotId()].Ball_Detect_Radius = packet.Ball.GetBallDetectRadius()
		adjustment[packet.RobotsStatus.GetRobotId()].Circularity_Threshold = packet.Ball.GetCircularityThreshold()
	}
}

// Host a web server to display the robot IP addresses
func RobotIPList(chrobotip chan bool) {
	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		fmt.Fprintf(w, "<html><head><title>The RACOON Web Console</title>")
		fmt.Fprintf(w, `<script>
						function showImage(id) {
							fetch("/image/" + id, { method: "GET" })
								.then(response => {
									if (!response.ok) {
										throw new Error("Network Error: " + response.status);
									}
									return response.json();
								})
								.then(data => {
									document.getElementById("image").src = "data:image/jpeg;base64," + data.image;
								})
								.catch(error => {
									console.error("Error fetching image:", error);
								});
							document.getElementById("imagetitle").innerText = "Robot " + id + "'s Image";
							}
						</script>`)
		fmt.Fprintf(w, "</head><body><h1>The RACOON Web Console</h1>")
		fmt.Fprintf(w, "<table border=\"1\"><h2>Robot IP List</h2><tr><th>Robot ID</th><th>Associated IP Address</th><th>Beep</th><th>Image</th></tr>")
		for i := 0; i < 16; i++ {
			buzzurl := fmt.Sprintf("http://%s:9191/buzzer/tone/%s/1000", robot_ipaddr[i], strconv.Itoa(i))
			image := i
			fmt.Fprintf(w,
				"<tr><td>%d</td><td>%s</td><td><button onclick='location.href=\"%s\"'>Beep</button></td><td><button onclick='showImage(%d)'>Image</button></td></tr>",
				i, robot_ipaddr[i], buzzurl, image)
		}
		fmt.Fprintf(w, "</table>")
		fmt.Fprintf(w, "<h2>Vision Status: %t</h2>", isvisionrecv)
		fmt.Fprintf(w, "<p>Generated at %s</p>", time.Now().Format(time.RFC1123))

		fmt.Fprintf(w, "<h2 id='imagetitle'>Robot Image</h2>")
		fmt.Fprintf(w, "<img id='image' src='' width='320' alt='No Image Selected'/>")
		fmt.Fprintf(w, "</body></html>")

	})

	http.HandleFunc("/image/", func(w http.ResponseWriter, r *http.Request) {
		idStr := r.URL.Path[len("/image/"):]
		id, err := strconv.Atoi(idStr)
		if err != nil || id < 0 || id >= 16 {
			http.Error(w, "Invalid robot ID", http.StatusBadRequest)
			return
		}

		robotIP := robot_ipaddr[id]
		if robotIP == "" {
			http.Error(w, "Robot not found", http.StatusNotFound)
			return
		}

		imageURL := fmt.Sprintf("http://%s:9191/image", robotIP)
		resp, err := http.Get(imageURL)
		if err != nil {
			http.Error(w, "Failed to fetch image from robot", http.StatusInternalServerError)
			return
		}
		defer resp.Body.Close()

		if resp.StatusCode != http.StatusOK {
			http.Error(w, "Robot returned error", resp.StatusCode)
			return
		}

		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(http.StatusOK)
		bodyBytes, err := io.ReadAll(resp.Body)
		if err != nil {
			http.Error(w, "Failed to read image data", http.StatusInternalServerError)
			return
		}
		w.Write([]byte(fmt.Sprintf(`{"image": %s}`, bodyBytes)))
	})

	log.Fatal(http.ListenAndServe(":8080", nil))

	<-chrobotip
}
