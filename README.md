# RACOON-MW

## Overview

**_NOTE: This instruction is for Linux (Ubuntu) and MacOS. (Windows need some modification)_**

This is our communication software for [RoboCup Soccer SSL](https://ssl.robocup.org/)  
`RACOON-MW` stands for Ri-one ssl Accurate Operation Middleware

### Minimal Requirements

- 64bit machienes (Tested on: x86_64, ARM64)

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

- Port (-h) - Change Vision Multicast Port
- Report Rate (-r) How often report to RACOON-AI? (milliseconds)
- Goal-Pose (-g) Attack Direction Negative or Positive (N or P)
- Team (-t) Our Team (blue or yellow)



## Future Features

- Record and Replay
- Logging
- Web Graph
- Robot Online Check
