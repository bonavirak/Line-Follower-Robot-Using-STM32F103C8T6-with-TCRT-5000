# Line Follower Robot with STM32F103C8T6 and TCRT5000

This Line Follower Robot, winner of Line Follwer Robot Competition 2025, was designed and developed by Bona Virak. The project uses an STM32F103C8T6 microcontroller, TCRT5000 5-way sensors, HC-05 Bluetooth module, TB6612FNG motor driver, and N20 motors (1000 RPM). It features two modes: Auto (PID-based line following with Case 0/Case 1 logic) and Manual (Bluetooth RC control via app).

## Credits
- **Author**: Bona Virak - Sole developer of the code, PCB design, and implementation.
- **Team**: Tire-d - Acknowledged for participation in Line Follwer Robot Competition 2025, with minimal contributions.

## Features
- C-based firmware with PID control for auto mode line following.
- Manual mode for Bluetooth RC car control via HC-05.
- PCB design (schematics, Gerber files, BOM) optimized for STM32 integration.
- Case-based navigation (Case 0 for straight paths, Case 1 for turns) in auto mode.

## Setup
### Hardware
1. **Components**:
   - STM32F103C8T6 microcontroller
   - TCRT5000 5-way sensor array
   - HC-05 Bluetooth module
   - TB6612FNG motor driver
   - 2x N20 motors (1000 RPM)
   - Power supply (e.g., 11.1V LiPo battery)
2. **Wiring**:
   - See schematic:
   ![Schematic](LineFollowerrRobot.png)

### Software
1. **Requirements**:
   - STM32CubeIDE for compiling C code.
   - ST-Link/V2 for flashing.
   - Bluetooth RC app (e.g., "Bluetooth RC Controller" on Android/iOS) for manual mode.
2. **Steps**:
   - Clone: `git clone https://github.com/yourusername/STM32F103C8T6-Line-Follower-Robot-TCRT5000.git`.
   - Open `/code/` in STM32CubeIDE.
   - Configure PID in `pid.c` (e.g., Kp=0.5, Ki=0.1, Kd=0.05) for auto mode.
   - Build/flash code to STM32F103C8T6 using ST-Link.
3. **Auto Mode**:
   - Test Case 0/Case 1 on sample map (`/docs/map_example.png`).
4. **Manual Mode**:
   - Pair HC-05 with Bluetooth RC app (default PIN: 1234 or 0000).
   - Use app to control motors (forward, backward, left, right).

## Demo
Watch the robot in action:
- **Video**: [YouTube/Google Drive link showing auto and manual modes]
- **Images**: [Imgur/Google Drive link with robot, PCB, and app interface]
- **Description**: Auto mode uses PID control for line following (Case 0 for straight, Case 1 for turns). Manual mode allows RC car control via Bluetooth app, demonstrated in [Competition Name] winning run.

## License
Licensed under the MIT License. See [LICENSE](LICENSE) for details.
