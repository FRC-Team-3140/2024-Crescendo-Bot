## Camera Setup and Test Procedure

1. **Electrical Connection Check**
    - [ ] Replace battery! 
    - [ ] Connect the Raspberry Pi to the power source.
    - [ ] Verify that all electrical connections are secure (Including Cameras!).
    - [ ] Check for any signs of electrical damage or wear.

2. **Photonvision Connection**
    - [ ] Verify that cameras can be viewed using Photonvision.
    
## Camera class information
* __Camera Names:__ 
  * There are two variables: 'aprilTagCameraName' 'shapeCameraName'
* __Connection Attempts, Delay between attempts, & min ambiguity:__ 
  * These are all fields that you configure in the Camera.getInstance() method. 
* __AprilTag field layout:__
  * You will need to configure the field layout for the Photonvision Pose estimator. This can be done at the top of the class.

### Error Checking related methods
* testConnection() - Checks if previous value is equal to the new value useing the Photonvision "Heart Beat" value (basically a runtime counter) 
* attemptToReconnect() - If **connection** boolean becomes **false** (from testConnection()), attemptToReconnect() will run in a thread and every specified period of time will use the testConnection() to check if Photonvision has started responding again.
* checkVersion() - Will test if PhotonLib version matches Photonvision version matches on the Raspberry PI. If it doesn't it will prevent any methods from accessing the cameras to prevent the version mismatch error. 
