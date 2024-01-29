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

The arm uses one motor to move up and down, allowing the intake/shooter subsystem to move to the ground to pick up the notes and move back up to shoot them.

## Intake/Shooter

The intake and shooter are both in one subsystem, connected to the top of the arm and using one motor. It picks up a ground note using 2 sets of wheels, then another set of wheels pushes the note into a shooter where it is then shot into the air through another set of wheels.

## Climber

The robot will grab onto the chain using a two-piece cranberry elevator with a specially designed hook to grab onto the chain.

## Swerve Drive

| Component ID | Interface | Connection | Role |
|---|---|---|---|
| Turn Motor Front Left | CANSparkMax | CAN ID: 8 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Front Left | CANSparkMax | CAN ID: 7 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | ThriftyBot | Analog Core ID: 3 | An **absolute encoder** for sensing swerve module angle. |

| Turn Motor Front Right | CANSparkMax | CAN ID: 6 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Front Right | CANSparkMax | CAN ID: 5 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | ThriftyBot | Analog Core ID: 2 | An **absolute encoder** for sensing swerve module angle. |

| Turn Motor Back Left | CANSparkMax | CAN ID: 2 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Back Left | CANSparkMax | CAN ID: 1 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | ThriftyBot | Analog Core ID: 0 | An **absolute encoder** for sensing swerve module angle. |

| Turn Motor Back Right | CANSparkMax | CAN ID: 4 | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Back Right | CANSparkMax | CAN ID: 3 | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | ThriftyBot | Analog Core ID: 1 | An **absolute encoder** for sensing swerve module angle. |





# Commands

Overview of commands and how they support actions here...

| Command Name | Game Mode | Priority | Difficulty | Description |
|---|---|---|---|---|
| SwerveController | Teleop | **Required** | 2 | Joystick control for controlling the swerve drive. Adopted and updated from 2023. |
| RunIntakeGround | Both | **Required** | 3 | Position and run the intake to load a note from the ground. |
| OneNoteAutoLeft | Auto | Medium | 5 | Shoot a note into the speaker and then move out of the zone. Starting in the left position. **## Points** | 




