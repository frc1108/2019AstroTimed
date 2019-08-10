package frc.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Camera extends IterativeRobot {

  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }
}