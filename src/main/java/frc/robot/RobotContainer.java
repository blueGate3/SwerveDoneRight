// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(0);

  
  private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
  .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());

  public RobotContainer() {
    configureBindings();
    m_drivetrain.setDefaultCommand(
      Commands.run(
        () -> m_drivetrain.drive(
          -m_driverController.getRawAxis(1), 
          -m_driverController.getRawAxis(0), 
          m_driverController.getRawAxis(4), 
          true), 
        m_drivetrain)
    );
  }

  private void configureBindings() {
  }

  public void repeat() {
    SmartDashboard.putNumber("OdometryX", poseSub.get().getX());
    SmartDashboard.putNumber("OdometryY", poseSub.get().getTranslation().getY()); //TODO for some reason this is backwards?
    SmartDashboard.putNumber("OdometryRot", poseSub.get().getRotation().getDegrees());
  }

}
