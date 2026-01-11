// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class Telemetry extends SubsystemBase {

  private final Hopper m_hopper;
  private final Climber m_climber;
  private final Feeder m_feeder;
  private final Shooter m_shooter;

  /** Telemetry class, handles all telemetry and smart dashboard data. */
  public Telemetry(Hopper hopper, Climber climber, Feeder feeder, Shooter shooter) {
    m_hopper = hopper;
    m_climber = climber;
    m_feeder = feeder;
    m_shooter = shooter;
  }

  @Override
  public void periodic() {
    // climber
    SmartDashboard.putNumber("climber leftpos", m_climber.climberPositions().getFirst());
    SmartDashboard.putNumber("climber rightpos", m_climber.climberPositions().getSecond());
    // shooter
    SmartDashboard.putNumber("shooter velocity", m_shooter.getVelocity());
  }
}
