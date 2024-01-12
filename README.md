Introduction here...

# Reference

 * https://www.youtube.com/@UnqualifiedQuokkasRi3D/featured
 * Programming Style Guide: 

# Actions

For planning purposes, discussion of actions the robot will need to do here ...

Teleop strategy...

Auto strategy...

 * Action 1 ...
 * Action 2 ...
 * Action 3 ...

# Subsystems

Discussion of subsystems and how they work together here...

## Swerve Drive

Overview of swerve drive here...

| Component ID | Interface | Connection | Role |
|---|---|---|---|
| Turn Motor Front Left | CANSparkMax | CAN ID ? | A **motor** and **encoder** for changing the swerve module angle. |
| Drive Motor Front Left | CANSparkMax | CAN ID ? | A **motor** and **encoder** for changing the swerve module speed. |
| Turn Encoder | CanCoder | CAN ID ? | An **absolute encoder** for sensing swerve module angle. |

## Subsystem 2

## Subsystem 3

# Commands

Overview of commands and how they support actions here...

| Command Name | Game Mode | Priority | Difficulty | Description |
|---|---|---|---|---|
| SwerveController | Teleop | **Required** | 2 | Joystick control for controlling the swerve drive. Adopted and updated from 2023. |
| RunIntakeGround | Both | **Required** | 3 | Position and run the intake to load a note from the ground. |
| OneNoteAutoLeft | Auto | Medium | 5 | Shoot a note into the speaker and then move out of the zone. Starting in the left position. **## Points** | 




