// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final AutoFactory autoFactory;
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(0);
  
  private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
    .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());

  public RobotContainer() {
    autoFactory = new AutoFactory(
      m_drivetrain::getOdometry, // A function that returns the current robot pose
      m_drivetrain::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      m_drivetrain::followTrajectory, // The drive subsystem trajectory follower 
      true, // If alliance flipping should be enabled 
      m_drivetrain 
    );

    configureBindings();
  }

  private void configureBindings() {
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

  public void updateTelemetry() {
    SmartDashboard.putNumber("OdometryX", poseSub.get().getX());
    SmartDashboard.putNumber("OdometryY", poseSub.get().getTranslation().getY()); //TODO for some reason this is backwards?
    SmartDashboard.putNumber("OdometryRot", poseSub.get().getRotation().getDegrees());
  }

   public Command getAutonomousCommand() {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }


    //return autoFactory.trajectoryCmd("SquarePath");
  }
}
