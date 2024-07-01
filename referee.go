package main

import (
	"log"
	"net"

	"github.com/Rione-SSL/RACOON-MW/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

func RefereeClient(chref chan bool, gcport int) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP("224.5.23.1"),
		Port: gcport,
	}
	interfacename, _ := net.InterfaceByName(NW_VISION_REFEREE_INTERFACE_NAME)

	if interfacename == nil {
		log.Println("[WARNING] MW Referee Signal NW Interface Name is wrong! Trying system-default interface!")
	}

	log.Printf("Referee Client started.")
	serverConn, err := net.ListenMulticastUDP("udp", interfacename, serverAddr)
	CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, NW_REF_MAX_DATAGRAM_SIZE)
	refcounter := 0

	for {
		n, addr, err := serverConn.ReadFromUDP(buf)
		CheckError(err)

		packet := &pb_gen.Referee{}
		err = proto.Unmarshal(buf[0:n], packet)
		CheckError(err)
		ref_command = packet

		now_command = packet.Command
		if pre_command == nil {
			pre_command = packet.Command
		}

		if now_command.String() != pre_command.String() {
			last_command = (*pb_gen.Referee_Info_Command)(pre_command)
		}
		pre_command = packet.Command
		if refcounter%600 == 0 {
			log.Printf("==== REFEREE OK KEEP RECEIVING FROM %s ====", addr)
			refcounter = 0
		}

		refcounter++
	}
}
