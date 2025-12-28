// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  Drivetrain drivetrain = new Drivetrain();
  CommandPS4Controller m_dc = new CommandPS4Controller(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(Commands.run(
      () -> 
      drivetrain.drive(
        m_dc.getRawAxis(0), 
        m_dc.getRawAxis(1), 
        -m_dc.getRawAxis(4), true), 
        drivetrain
      ));

      
  }

}
