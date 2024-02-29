## Camera Setup and Test Procedure

1. **Electrical Connection Check**
    - [ ] Replace battery! 
    - [ ] Connect the Raspberry Pi to the power source.
    - [ ] Verify that all electrical connections are secure (Including Cameras!).
    - [ ] Check for any signs of electrical damage or wear.

2. **Photonvision Connection**
    - [ ] Verify that cameras can be viewed using Photonvision.
    
## Camera class information
    * All "getDistance" functions must return 0 if Photonvision has an issue or if there's no target in view (for the methods that you specify an ID & for the best quality methods). If they didn't return 0, any methods that were using them for movement measurements would get a valid value/point to move to. 
    * Same type of thing for angle related methods (they return 999) due to it being out of the valid 360 & +- 180 ranges. 

### Error Checking related methods
    * testConnection() - Checks if previous value is equal to the new value useing the Photonvision "Heart Beat" value (basically a runtime counter) 
    * attemptToReconnect() - If **connection** boolean becomes **false** (from testConnection()), attemptToReconnect() will run in a thread and every specified period of time will use the testConnection() to check if Photonvision has started responding again.
    * checkVersion() - Will test if PhotonLib version matches Photonvision version matches on the Raspberry PI. If it doesn't it will prevent any methods from accessing the cameras to prevent the version mismatch error. 