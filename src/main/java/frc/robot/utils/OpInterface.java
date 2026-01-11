// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import static frc.robot.utils.Constants.OIConstants.*;
import static frc.robot.utils.Constants.SwerveDriveConstants.*;

public class OpInterface extends SubsystemBase {
  
  // controllers
  private final CommandGenericHID m_driverctlr = new CommandGenericHID(0);

  // max speed multiplier and slew, slew control ramp rate up or down in speed when changing
  private final SlewRateLimiter m_speedslew = new SlewRateLimiter(3);
  private double maxspeed = 0;

  public OpInterface() {}

  /** Returns the field-oriented drive commanded speeds. */
  public ChassisSpeeds fieldSpeedsTeleop() {
    return new ChassisSpeeds(
      MathUtil.applyDeadband(m_driverctlr.getRawAxis(0), k_deadzone) * maxspeed,
      MathUtil.applyDeadband(m_driverctlr.getRawAxis(1), k_deadzone) * maxspeed,
      MathUtil.applyDeadband(m_driverctlr.getRawAxis(3), k_deadzone) * maxspeed
    );
  }

  /** Returns the controller object to interface bindings. */
  public CommandGenericHID getDriverCtlr() {
    return m_driverctlr;
  }

  @Override
  public void periodic() {
    maxspeed = m_speedslew.calculate(m_driverctlr.button(0).getAsBoolean() ? k_maxlinspeedteleop : k_maxlinspeed);
  }
}
