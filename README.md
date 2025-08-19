# Line Follower Robot with STM32F103C8T6 and TCRT5000

This Line Follower Robot, winner of Line Follwer Robot Competition 2025, was designed and developed by Bona Virak. The project uses an STM32F103C8T6 microcontroller, TCRT5000 5-way sensors, HC-05 Bluetooth module, TB6612FNG motor driver, and N20 motors (1000 RPM). It features two modes: Auto (PID-based line following with Case 0/Case 1 logic) and Manual (Bluetooth RC control via app).

## Credits
- **Author**: [Bona Virak](https://github.com/bonavirak) - Sole developer of the code, PCB design, and implementation.
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
2. **Schematic**:
   - See schematic: <img width="900" height="500" alt="Image" src="https://github.com/user-attachments/assets/2412f9f5-5756-41ab-bb71-dae4e4280583" />
3. **Wiring**:
- **🔌 Connect TCRT5000 Sensors to STM32**:
  - Pins: **PA0-PA4** 
- **📡 Link HC-05 TX/RX to STM32**:
  - USART1: **PA9/PA10** (TX/RX) 📶
- **⚙️ Attach TB6612FNG PWM/GPIO Pins to STM32**:
  - PWMA: **PB7** 🚀 (TIM4 CH2)
  - PWMB: **PB6** 🚀 (TIM4 CH1)
  - AI1: **PB12** 🔧
  - AI2: **PB13** 🔧
  - BI1: **PB14** 🔧
  - BI2: **PB15** 🔧
  - STBY: **PB1** ✅
  - VCC: **3.3V** ⚡
- **🎮 Add Servo to STM32**:
  - PWM: **PB4** (TIM3 CH1) 🕹️
- **🔋 Wire N20 Motors to TB6612FNG Outputs**:
  - Power: **11.1V LiPo** 💪
- **📋 Refer to Detailed Pin Assignments**:
  - Check out the epic diagram! 🕵️‍♂️
   <img width="600" height="500" alt="Pin Assignments" src="https://github.com/user-attachments/assets/fe4a90a4-a5c9-4494-888d-3d386bc1e5b7" />
     

### Software
1. **Requirements**:
   - STM32CubeIDE for compiling C code.
   - ST-Link/V2 for flashing.
   - Bluetooth RC app (e.g., "Bluetooth RC Controller" on Android) for manual mode.
2. **Steps**:
   - Clone: `[git clone https://github.com/bonavirak/STM32F103C8T6-Line-Follower-Robot-TCRT5000.git`.
   - Open `/code/` in STM32CubeIDE.
   - Configure PID in `pid.c` (e.g., Kp=0.5, Ki=0.1, Kd=0.05) for auto mode.
   - Build/flash code to STM32F103C8T6 using ST-Link.
3. **Auto Mode**:
   - Test Case 0/Case 1 on sample map ![Image](https://github.com/user-attachments/assets/e99e4eb1-bb90-4a00-8fd3-97d342d0a8da)
4. **Manual Mode**:
   - Pair HC-05 with Bluetooth RC app (default PIN: 1234 or 0000).
   - Use app to control motors (forward, backward, left, right).

## Demo
Watch the robot in action:
- **Video**: [Soon!]
- **Images**: [Soon!]
- **Description**: Auto mode uses PID control for line following (Case 0 for straight, Case 1 for turns). Manual mode allows RC car control via Bluetooth app, demonstrated in Line Follwer Robot Competition 2025 winning run.

## License
Licensed under the MIT License. See [LICENSE](LICENSE) for details.
