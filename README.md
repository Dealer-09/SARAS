# Project Tomato (Sentinel Swarm)

A decentralized, GPS-denied disaster management swarm built for our hackathon.

## System Architecture (WIP)
We are building a dual-layer swarm:
- **Aerial Node:** 1x Quadrotor (Raspberry Pi 5 8GB + ESP32 Flight Controller)
- **Ground Nodes:** 2x Sentinel Ground Bots (Raspberry Pi 4)
- **Communications:** 3x XBee PRO S2C modules creating a standalone mesh network

## Hackathon To-Do List
- [ ] Initialize repository and standard ignore files
- [ ] Setup ground bot motor control via serial
- [ ] Test basic XBee mesh (Drone <-> Ground Bots)
- [ ] Implement YOLO casualty detection
- [ ] Implement decentralized swarm leader election
- [ ] Setup aerial node flight controller (FreeRTOS)
- [ ] Build simulation environment for testing
