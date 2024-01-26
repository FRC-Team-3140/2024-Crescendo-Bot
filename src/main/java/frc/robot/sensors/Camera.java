// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera extends SubsystemBase {
  
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private PhotonCamera april = new PhotonCamera(inst, "april"); 
  
  public Camera() {}

  @Override
  public void periodic() {
    faceAprilTagID(1);
  }

  public int getAprilTagID() {
    /* If this function returns a 0, that means that PhotonVision either isn't configured
     * correctly, not connected, or their is some other issue (e.g. Camera name isn't matching 
     * the code).
     * If this function returns a 1, that means there is not any detected targets
     */
    
    if (inst.getTable("photonvision") != null){
      if (april.getLatestResult().getBestTarget() != null) {
        return april.getLatestResult().getBestTarget().getFiducialId();
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  }

  public double getAprilTagYaw() {
    /* If this function returns a 0, that means that PhotonVision either isn't configured
     * correctly, not connected, or their is some other issue (e.g. Camera name isn't matching 
     * the code).
     * If this function returns a 1, that means there is not any detected targets
     */

    if (inst.getTable("photonvision") != null){
      if (april.getLatestResult().getBestTarget() != null) {
        return april.getLatestResult().getBestTarget().getYaw();
      } else {
        return 1;
      }
    } else {
      return 0;
    }
  }

  public void faceAprilTagID(int ID){
    System.out.println(getAprilTagID() + " " + getAprilTagYaw());
  }
}
