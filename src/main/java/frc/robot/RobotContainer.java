// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(0);

  public RobotContainer() {
    configureBindings();

    new RunCommand(
      () -> m_drivetrain.drive(
          m_driverController.getRawAxis(0), 
          m_driverController.getRawAxis(1), 
          m_driverController.getRawAxis(4), false), 
        m_drivetrain);
    
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
