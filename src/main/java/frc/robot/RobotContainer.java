// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  // joystick
  private final CommandGenericHID m_driverctlr = new CommandGenericHID(0);

  // subsystems
  private final Climber m_climber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();

  // misc vars
  private double shootervel = 0;
  private boolean hopperdeployed = false;

  public RobotContainer() {
    m_shooter.setDefaultCommand(shooterDefault());
    configureBindings();
  }

  private void configureBindings() {
    // shooter bindings
    m_driverctlr.button(1).whileTrue(runEnd(() -> shootervel = 10, () -> shootervel = 0));
    m_driverctlr.button(2).onTrue(
      run(() -> m_feeder.feed(), m_feeder)
        .withTimeout(1)
        .finallyDo(() -> m_feeder.stop()));

    // hopper bindings
    m_driverctlr.button(8).onTrue(
      runOnce(() -> m_hopper.extend(), m_hopper)
      .until(() -> m_hopper.bootyCurrent() > 15)
      .finallyDo(() -> runOnce(() -> m_hopper.stop(), m_hopper)));
  }

  /** Default command for the shooter subsytem. */
  public Command shooterDefault() {
    return run(() -> m_shooter.closedLoop(shootervel), m_shooter);
  }

  public Command getAutonomousCommand() {
    return print("No autonomous command configured");
  }
}
