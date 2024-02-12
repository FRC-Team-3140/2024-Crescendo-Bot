This is the repository for our competition bot, yet to be named, for 2024 Crescendo.

# Reference

 * https://www.youtube.com/@UnqualifiedQuokkasRi3D/featured
 * Programming Style Guide: https://docs.google.com/document/d/1gFDDgxJKg1U1g2qiMRLbdnMCAiIlr2gexJFJaxHWk/edit?usp=sharing

# Actions

The robot will need to pick up notes from the ground, pick up notes from the human player station, shoot notes in the speaker and amp, drive under the stage, and climb onto a chain (the stage), possibly climbing with another robot.

Teleop strategy: During teleop, the player will control the robot to pick up notes on the ground on its side of the field and also go across the field to pick up notes dropped by the human player. The robot will then shoot the notes in both the speaker and the amp until the endgame buzzer goes off, after which the player will have the robot climb the chain and stay elevated on the stage.

Auto strategy...

 * Taxi
 * Shoot into Speaker (Bot starts off with the back facing the Speaker)
 * Shoot & Taxi
 * Two Note Auto (Using Shoot into Speaker auto and then going to pick up a note and shoot)

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

## Subsystem Progress

| Subsystem Name | Description | CRL Rating |
| -------------- | ------------------------- | ---------- |
| SwerveDrive    | The drivetrain for the robot   | CRL 3      |
| IntakeShooter    | Intake is at the front of the robot and shooting is out the back.  Both attached at the end of the arm.   | CRL 3      |
| Arm    | Moves to the ground to intake notes and then rases up to shoot or drop notes in amp.  Controlled with a setpoint.  | CRL 3 |
| Climber    | Two hooks that lift the robot of the ground in endgame  | CRL 2  |
| Camera | Interfaces to two cameras through photon vision | CRL 2 |

# Commands

This table is a strategic tool for FRC robot development. It outlines all robot commands, their complexity, and current readiness levels. Start by implementing **Required** commands, beginning with simpler ones. The Status column helps track progress and identify areas needing attention. The goal is to create a robot that can efficiently execute game actions, ideally at a button press, to score points, speed up movements, and simplify the driver interface.

| Command Name | Subsystems | Game Mode | Priority | Complexity | Description | Status |
|---|---|---|---|---|---|---|
| ControlSwerveDrive (In RobotContainer) | SwerveDrive | Teleop | **Required** | L2 | Joystick control for the swerve drive | CRL 2 |
| IntakeUntilNoteDetected | IntakeShooter | Both | **Required** | L1 | Run the intake until sensor identifies a note | CRL 1 |
| ShootNoteIntoSpeaker | IntakeShooter | Both | **Required** | L1 | Shoot a note into a speaker. At kSpeakerSpeed. | CRL 2 |
| ShootNoteIntoAmp | IntakeShooter | Both | **Required** | L1 | Shoot a note into an amp. At kAmpSpeed. | CRL 2 |
| SetArmToAngle | Arm | Both | **Required** | L1 | Set the arm to an angle | CRL 3 |
| SetArmToDistance | Arm | Both | **Required** | L1 | Use an interpolator to estimate the angle at distace | CRL 2 |
| ScoreInSpeaker | IntakeShooter, Arm | Both | **Required** | L2 | Bumper on Speaker, shoot note at Speaker speed | CRL 0 |
| ScoreInAmp | IntakeShooter, Arm | Both | **Required** | L2 | Bumper on Amp, direct arm to AmpPosition, shoot note at Amp speed | CRL 0 |
| SpeakerShootDistance | Camera, Drivetrain, Arm, IntakeShooter | Both | Optional | L3 | Shoot from a distance, auto aim using camera and apriltags. | CRL 2 |
| PickupNoteWithBackwardMovement | Drivetrain, Arm, IntakeShooter | Teleop | Optional | L2 | Move intake to ground, run intake, move backward until sensor triggers | CRL 0 |
| PassNoteToTeammate | Arm, IntakeShooter | Teleop | Optional | L2 | Shoot note horizontally toward teammate. At kSpeakerSpeed. | CRL 0 |
| PerformMobilityMovement | SwerveDrive | Auto | **Required** | L1 | Move Drivetrain in a specific way to score mobility point | CRL 0 |
| AutoShootIntoSpeaker | IntakeShooter, Arm, SwerveDrive | Auto | **Required** | L2 | Set up bot toward speaker, set arm position, shoot at speaker, move to score mobility point | CRL 0 |
| AutoShootIntoAmp | IntakeShooter, Arm, SwerveDrive | Auto | Optional | L2 | Set up bot toward Amp, set arm position, shoot at Amp, move to score mobility point | CRL 0 |
| PrepareClimber | Climber | Endgame | **Required** | L1 | Raise hooks, ready to climb | CRL 0 |
| LiftRobotWithClimber | Climber | Endgame | **Required** | L1 | Lower hooks, lift bot off ground | CRL 0 |
| LockClimber | Climber | Endgame | **Required** | L1 | Lock climber in place | CRL 0 |
| ReleaseClimber | Climber | Endgame | **Required** | L1 | Release climber | CRL 0 |

## Competition Readiness Level (CRL)

This table outlines the Competition Readiness Level (CRL) system used to track the progress of subsystems and capabilities in our FRC robot development. It ranges from CRL 0, where a concept is proposed, to CRL 6, indicating full competition readiness. Each level has specific completion criteria, providing a clear roadmap for development and integration. This system ensures that our robot is thoroughly tested and ready for competition.

| Level | Label | Description | Completion Criteria | 
| --- | --- | --- | --- |
| CRL 0 | Concept | Idea for a command or capability is proposed. | Concept is documented and approved by team lead |
| CRL 1 | Designed | Detailed design with optional incomplete/untested implementation. | Design is documented and approved by team lead |
| CRL 2 | Developed | Code is complete and tested in isolation | Code compiles, runs, and passes initial tests |
| CRL 3 | Integrated | Code has been integrated with other subsystems | Code is integrated, compiles, and runs on the competition bot |
| CRL 4 | Tested | Subsystem or capability has been tested in competition-like scenarios | Passes all tests, approved by programming lead and mentor |
| CRL 5 | Practice Ready | Ready for drive team practice. Is fast, reliable, and easy to use | Drive team tested in practice scenarios, drive team approval |
| CRL 6 | Competition Ready | Fully integrated with other functions and well tested | Passes all final tests, programming and drive team approval |

## Command Complexity Levels for FRC Robot Commands

In the context of FRC robot programming, we categorize our commands into different complexity levels. This helps us manage the development process, prioritize tasks, and communicate about the functionality of our robot. Each level, from L1 to L4, represents a different degree of complexity, from simple commands that control a single subsystem to highly complex commands that can autonomously complete game objectives.

| Level | Label | Description | 
| --- | --- | --- |
| L1 | Simple Command | Simple to implement and achieves a basic game objective. Typically uses one subsystem and is executed by drivers. |
| L2 | Complex Command | Involves multiple subsystems or commands working together to execute more complex tasks quickly or to simplify driver controls. |
| L3 | Intelligent Command | Completes complex tasks that integrate sensing and control, and may be composed of multiple simpler commands. Can complete complex objectives autonomously or with minimal driver control. | 
| L4 | Skynet | Wins the game autonomously. No human intervention needed. |

