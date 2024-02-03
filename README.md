This is the repository for our competition bot, yet to be named, for 2024 Crescendo.

# Reference

 * https://www.youtube.com/@UnqualifiedQuokkasRi3D/featured
 * Programming Style Guide: https://docs.google.com/document/d/1gFDDgxJKg1U1g2qiMRLbdnMCAiIlr2gexJFJaxHWk/edit?usp=sharing

# Actions

The robot will need to pick up notes from the ground, pick up notes from the human player station, shoot notes in the speaker and amp, drive under the stage, and climb onto a chain (the stage), possibly climbing with another robot.

Teleop strategy: During teleop, the player will control the robot to pick up notes on the ground on its side of the field and also go across the field to pick up notes dropped by the human player. The robot will then shoot the notes in both the speaker and the amp until the endgame buzzer goes off, after which the player will have the robot climb the chain and stay elevated on the stage.

Auto strategy...

 * Action 1 ...
 * Action 2 ...
 * Action 3 ...

# Subsystems

The robot has 3 main subsystems and utilizes swerve drive for movement:

## Arm

The arm uses two motors to move up and down, which are configured as a group, allowing the intake/shooter subsystem to move to the ground to pick up the notes and move back up to shoot them.

| Component ID | Interface | Connection | Role |
|---|---|---|---|
| Arm Motor 1 | CANSparkMax |CAN ID: 9| A **motor** for controlling the arm |
| Arm Motor 2 | CANSparkMax |CAN ID: 10| A **motor** for controlling the arm |
| Arm Encoder | DutyCycleEncoder | PWM Ports 0 & 1 | An **absolute encoder** for sensing the arm angle |

## Intake/Shooter

The intake and shooter are both in one subsystem, connected to the top of the arm and using one motor. It picks up a ground note using 2 sets of wheels, then another set of wheels pushes the note into a shooter where it is then shot into the air through another set of wheels.

| Component ID | Interface | Connection | Role |
|---|---|---|---|
| Intake Motor | CANSparkMax | CAN ID: 11, PDH Port 6 | A **motor** for controlling the intake. If necessary, the current may be measured from the PDH port |
| Color Sensor V3/Proximity Sensor | ColorSensor | I^2C Port | A **color sensor** that detects when a piece has been received |
| Note Camera | PhotonCamera | Raspberry PI USB | A **camera** that can identify notes | 
| Shooter Motor | CANSparkMax | CAN ID: 12 | A **motor** for controlling the shooter |
| Shooter Motor | CANSparkMax | CAN ID: 13 | A **motor** for controlling the shooter |
## Climber

The robot will grab onto the chain using a two-piece cranberry elevator with a specially designed hook to grab onto the chain.

| Component ID | Interface | Connection | Role |
|---|---|---|---|
| Climer Motor Left | CANSparkMax | CAN ID: 14 | A **motor** for controlling the left climber |
| Climer Motor Right | CANSparkMax | CAN ID: 15 | A **motor** for controlling the right climber |
| Solenoid Left | Relay | Relay Port: 0 | A solenoid that is powered by a **relay** that locks the position of the left climber |
| Solenoid Right | Relay | Relay Port: 1 | A solenoid that is powered by a **relay** that locks the position of the right climber |

## Swerve Drive

| Component ID | Interface | Connection | Role |
|---|---|---|---|
| Turn Motor Front Left | CANSparkMax | CAN ID: 8 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Front Left | CANSparkMax | CAN ID: 7 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | Absolute Encoder (Analog Encoder) | Analog Input ID: 3 | An **absolute encoder** for sensing swerve module angle. It is a custom class extending the WPILib-provided class of analog encoder|
| | | | |
| Turn Motor Front Right | CANSparkMax | CAN ID: 6 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Front Right | CANSparkMax | CAN ID: 5 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | Absolute Encoder (Analog Encoder) | Analog Input ID: 2 | An **absolute encoder** for sensing swerve module angle. |
| | | | |
| Turn Motor Back Left | CANSparkMax | CAN ID: 2 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Back Left | CANSparkMax | CAN ID: 1 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | Absolute Encoder (Analog Encoder) | Analog Input ID: 0 | An **absolute encoder** for sensing swerve module angle. |
| | | | |
| Turn Motor Back Right | CANSparkMax | CAN ID: 4 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Back Right | CANSparkMax | CAN ID: 3 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | Absolute Encoder (Analog Encoder) | Analog Input ID: 1 | An **absolute encoder** for sensing swerve module angle. |


# Commands

Overview of commands and how they support actions here...

| Command Name | Subsystems | Game Mode | Priority | Complexity | Description | Status |
|---|---|---|---|---|---|---|
| In RobotContainer | Swerve Drive | Teleop | **Required** | L2 | Joystick control for controlling the swerve drive | CRL 2 |
| SpinTillSensor | IntakeShooter | Both | **Required** | L1 | Run the intake until sensor identifies a note. | CRL 1 |
| SpeakerShoot | IntakeShooter | Both | **Required** | L1 | Shooting a note into a speaker. Pass in speeds | CRL 1 |
| AmpShoot | IntakeShooter | Both | **Required** | L1 | Shooting a note into a amp. Pass in speeds | CRL 0 |
| SetArmAngle | Arm | Both | **Required** | L1 | Setting the arm to an angle | CRL 0 |
| IntakePosition | Arm | Both | **Required** | L1 | Arm near ground to intake notes | CRL 0 |
| CarryPosition | Arm | Both | **Required** | L1 | Arm raised to a safe movement position | CRL 0 |
| AmpPosition | Arm | Both | **Required** | L1 | Arm at position to score in Amp | CRL 0 |

| Left Climb Up | Climber | **Required**
| Right Climb Up | Climber | **Required**
| Left Climb Down | Climber | **Required**
| Right Climb Down | Climber | **Required**
| Lock Solenoid Left | Climber | **Required**
| Lock Solenoid Right | Climber | **Required**
| SpeakerScore |
| AmpScore |
| ShootingFallback | Camera | 
| ComplicatedPickupCommand | 
| PassToTeammate | 
| Mobility |
| ShootToSpeakerInAuto | 
| ShootToAmpInAuto | 

## Competition Readiness Level (CRL)

| Level | Label | Description | Completion Criteria | 
| --- | --- | --- | --- |
| 0 | Experimentation | Just testing a new idea or not started. | N/A |
| 1 | In Progress | Developing a basic capability | Code compiles and runs on subsystem in testing mode. |
| 2 | MVP | Minimal Viable Product. Could be used in competition. | Integrated and tested with competition bot. Can be manually controlled by drivers to meet basic competition needs. | 
| 3 | Prototype | Capability meets competition expectations. Most issues resolved and ready for competition if needed. | Verified to meet specs. Well tested by programming team. Approved by programming lead and mentor. | 
| 4 | Tested | Ready for drive team practice. Is fast, reliable, and easy to use. | Drive team tested in competition-like scenarios. Drive team approval. |
| 5 | Competition Ready | Fully integrated with other functions and well tested. | Programming and Drive Team Approval |


