// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.utils.OpInterface;

public class RobotContainer {

  //Subsystems
  private final OpInterface m_oi = new OpInterface();

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return print("No autonomous command configured");
  }
}
