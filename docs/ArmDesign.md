
Here is the checklist in Markdown format:

## Arm Setup and Test Procedure

1. **Electrical Connection Check**
    - [ ] Connect the arm to the power source.
    - [ ] Verify that all electrical connections are secure.
    - [ ] Check for any signs of electrical damage or wear.

2. **Encoder Verification**
    - [ ] Verify that the arm encoder is connected properly.
    - [ ] Check that the encoder is functioning correctly by observing its readings as the arm moves in the network table.

3. **Motor Direction Check**
    - [ ] Run the motors at a positive speed.
    - [ ] Verify that both motors are running in the correct direction (positive speeds should lift the arm).
    - [] Set Max and min angles

4. **Forward Control Parameter Calibration**
    - [ ] Start with a small value for the forward control parameter.
    - [ ] Gradually increase the value until the arm maintains its position against gravity.
    - [ ] When set properly the arm should be easy to move manually in either direction for all arm locations with PID values set to 0.

5. **PID Tuning**
    - [ ] Start with small values for P, I, and D.
    - [ ] Gradually increase P until the arm moves quickly to a new position without oscillating.
    - [ ] Adjust I and D as necessary to improve the response.
    - [ ] Repeat the tuning process for gradually larger arm movements.

6. **Overall Functionality Check**
    - [ ] Test the arm's movement in all directions.
    - [ ] Verify that the arm can reach all expected positions.
    - [ ] Check that the arm maintains its position when not being actively moved.

Remember to always follow safety procedures when working with robotics.

Sure, here's an introduction and a table of constants that should be checked or updated during the setup process:

## Arm Setup Constants

During the setup and testing of the arm, there are several constants that you may need to adjust. These constants control various aspects of the arm's behavior, such as the PID controller parameters and the forward control parameter. You can easily change these values through the network table. Here's a list of the important constants that you should check during setup:


| Constant | Description |
|----------|-------------|
| kArmRightID | The ID for the right arm motor |
| kArmLeftID | The ID for the left arm motor |
| kArmEncoderID | The ID for the arm encoder |
| kMotorCurrentLimit | The current limit for the arm motors |
| kArmRightReversed | Whether the right arm motor is reversed |
| kArmLeftReversed | Whether the left arm motor is reversed |
| kEnabledMotorMode | The motor mode when the arm is enabled |
| kDisabledMotorMode | The motor mode when the arm is disabled |
| kDefaultP | The default proportional gain for the PID controller |
| kDefaultI | The default integral gain for the PID controller |
| kDefaultD | The default derivative gain for the PID controller |
| kDefaultSetpoint | The default setpoint for the arm |
| kMaxSetpoint | The maximum setpoint for the arm |
| kMinSetpoint | The minimum setpoint for the arm |
| kDefaultForwardParam | The default forward control parameter |
| kArmEncoderOffset | The offset of the arm encoder from the zero position |

Each of these constants can be adjusted to fine-tune the behavior of the arm. Be sure to test the arm thoroughly after making any changes to these values.

# Network Table Entries

| Network Table Key | Description |
|-------------------|-------------|
| ForwardParam | The default forward control parameter |
| ForwardPower | The forward control needed, proportional to the cosine of the angle |
| PidPower | The power calculated by the PID controller |
| Setpoint | The current setpoint of the PID controller |
| MotorSpeed | The speed at which the arm motors are set |
| Arm | The network table for the arm |
| P | The proportional gain for the PID controller |
| I | The integral gain for the PID controller |
| D | The derivative gain for the PID controller |
| CurrentAngle | The current angle of the arm |
| ErrorCode | Any error codes related to the arm encoder |
| IsEnabled | Whether the arm is enabled for movement |