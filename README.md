# RACOON-MW

## Overview

**_NOTE: This instruction is for Linux (Ubuntu) and MacOS. (Windows need some modification)_**

This is our communication software for [RoboCup Soccer SSL](https://ssl.robocup.org/)  
`RACOON-MW` stands for Ri-one ssl Accurate Operation Middleware.

RACOON-MW aggregates information from the Vision server and GameController and provides various useful information such as ball speed and robot speed together.

### Minimal Requirements

- 64bit machienes (Tested on: x86_64, ARM64)
- Robot are also needed to send packet to MW.

# Usage

## Generate Go Proto files
```bash
protoc --proto_path=proto/pb_src --go_out=proto/pb_gen --go_opt=paths=source_relative proto/pb_src/*.proto
```

## Run

```bash
./RACOON-MW
```

If you want to change Vision Multicast Port, run with:

```bash
./RACOON-MW -p 10020
```

All option can see in the following flags:

```bash
./RACOON-MW -h
```

---

# Options

- Port (-p) - Change Vision Multicast Port
- Report Rate (-r) - How often report to RACOON-AI? (milliseconds)
- Goal-Position (-g) - Attack Direction Negative or Positive (N or P)
- Team (-t) - Our Team (blue or yellow)
- Team name (-tn) - Team name (Ex. Ri-one)
- Ball Moving Detect Threshould (-b) - Default 1000mm/s
- Half Court Mode (-c) - Where to use (N, P, F) F to Full. Disable when match mode is true (default "F")
- Debug Mode (-d) - Show All Send Packet
- Game Controller Listen Port (-gcp) - (Default 10003) Force 10003 when match mode is true
- Goal Keeper ID (-gk) - Goal Keeper ID (0-15) Disable when match mode is true
- grSim Command Listen Port Number (-grsimport) - Default 20011
- Ignore Referee Team Color & Attack Direction Mismatch Errors (-igref) - Disable when match mode is true
- Match Mode (-m) - Disable some options! Most option get from GC.
- Referee & Vision Listen Network Interface (-vif) - (ex. en1) (default "none")
- Robot Listen Network Interface (-rif) - (ex. en0) (default "none")
- Simulatiom Mode (-s) - (Emulate Ball Sensor). Disable when match mode is true

## Web Interface
Access to http://localhost:8080
You'll get each robots IP Addresses.

## Output Packet
You can see in https://github.com/Rione/ssl-RACOON-MW/blob/master/proto/pb_src/to_racoonai.proto

## From Robots Packet
For packets from the robot, see [Here](https://github.com/Rione/ssl-RACOON-MW/blob/master/proto/pb_src/pi_to_mw.proto) you will find each piece of information sent by the robot.

