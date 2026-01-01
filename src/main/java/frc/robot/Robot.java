// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private NetworkTables m_NetworkTables = new NetworkTables();
  private final RobotContainer m_robotContainer;
  private final StructSubscriber<Pose2d> poseSub = NetworkTableInstance.getDefault()
      .getStructTopic("Robot/CurrentPose", Pose2d.struct).subscribe(new Pose2d());


  
    //private final Drivetrain drivetrain = new Drivetrain();

    private final Timer timer = new Timer();
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("SquarePath");

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.repeat();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  public void autonomousInit() {
        // if (trajectory.isPresent()) {
        //     // Get the initial pose of the trajectory
        //     Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

        //     if (initialPose.isPresent()) {
        //         // Reset odometry to the start of the trajectory
        //         drivetrain.resetOdometry(initialPose.get());
        //     }
        // }

        // // Reset and start the timer when the autonomous period begins
        // timer.restart();
    }

    @Override
    public void autonomousPeriodic() {
        // if (trajectory.isPresent()) {
        //     // Sample the trajectory at the current time into the autonomous period
        //     Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());
        //     if (sample.isPresent()) {
        //         drivetrain.followTrajectory(sample.get());
        //     }
        // }
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
