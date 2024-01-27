// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.utils.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Camera extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private PhotonCamera april;
  private PhotonCamera notes;

  private boolean connected = false;

  public Camera() {
    april = new PhotonCamera(inst, "april");
    notes = new PhotonCamera(inst, "notes");

    while (connected == false) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in 5 seconds.");
        Timer.delay(5);
      }
    }
  }

  @Override
  public void periodic() {
    faceAprilTagID(1, true);
  }

  public int getAprilTagID() {
    /*
     * If this function returns a 0, that means that PhotonVision either isn't
     * configured
     * correctly, not connected, or their is some other issue (e.g. Camera name
     * isn't matching
     * the code).
     * If this function returns a -1, that means there is not any detected targets
     */

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getFiducialId();
    } else {
      return -1;
    }
  }

  public double getAprilTagYaw() {
    /*
     * If this function returns a 0, that means that PhotonVision either isn't
     * configured
     * correctly, not connected, or their is some other issue (e.g. Camera name
     * isn't matching
     * the code).
     * If this function returns a -1, that means there is not any detected targets
     */

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getYaw();
    } else {
      return -1;
    }
  }

  public double getAprilTagPitch() {
    /*
     * If this function returns a 0, that means that PhotonVision either isn't
     * configured
     * correctly, not connected, or their is some other issue (e.g. Camera name
     * isn't matching
     * the code).
     * If this function returns a -1, that means there is not any detected targets
     */

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getPitch();
    } else {
      return -1;
    }
  }

  public void faceAprilTagID(int ID, boolean verbose) {
    if (verbose == true) {
      System.out.println("----------------\nID: " + getAprilTagID() + "\nYaw:" + getAprilTagYaw() + "\nPitch: "
          + getAprilTagPitch() + "\n----------------");
    }
  }

  public double getNoteDistance() {
    /*
     * If this function returns a 0, that means that PhotonVision either isn't
     * configured
     * correctly, not connected, or their is some other issue (e.g. Camera name
     * isn't matching
     * the code).
     * If this function returns a -1, that means there is not any detected targets
     */

     return 0.0;
  }
}