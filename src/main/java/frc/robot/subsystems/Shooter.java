// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  // motors
  private final TalonFX m_shooter = new TalonFX(k_shootermotorID);
  private final TalonFX m_shooterfeed = new TalonFX(k_shootermotorID);

  // configurators
  private final TalonFXConfigurator m_shooterconfigurator = m_shooter.getConfigurator();
  private final TalonFXConfigurator m_shooterfeedconfigurator = m_shooterfeed.getConfigurator();

  // ff and pid controller, need to characterize
  private final ProfiledPIDController m_velocityPID = new ProfiledPIDController(
    k_velocitykP,
    k_velocitykI, 
    k_velocitykD, 
    k_velocityConstraints);
  private final SimpleMotorFeedforward m_velocityFF = new SimpleMotorFeedforward(
    k_velocitykS, 
    k_velocitykV);

  // misc vars
  private double velocityMPS = 0;

  public Shooter() {

    // apply the built config to the motor
    m_shooterconfigurator.apply(k_shooterconfig);
    m_shooterfeedconfigurator.apply(k_shooterintakeconfig);
  }

  /** Closed loop control using PIDF. */
  public void closedLoop(double velocitysp) {
    m_velocityPID.setGoal(velocitysp);
    double pid = m_velocityPID.calculate(velocityMPS);
    double ff = m_velocityFF.calculate(velocitysp);
    m_shooter.setVoltage(ff + pid);
  }

  /** Open loop control using only the FF */
  public void openLoop(double velocitysp) {
    m_shooter.setVoltage(m_velocityFF.calculate(velocitysp));
  }

  /** Stops the shooter motor. */
  public void stopShooter() {
    m_shooter.setVoltage(0);
  }

  /** Runs the feeder roller. */
  public void feed() {
    m_shooterfeed.setVoltage(12);
  }

  /** Stops the feeder roller. */
  public void stopfeed() {
    m_shooterfeed.setVoltage(0);
  }

  @Override
  public void periodic() {
    velocityMPS = (m_shooter.getVelocity().getValue().in(Units.RPM) * k_shooterwheelcircum) / 60; //returns in units of m/s
  }
}
