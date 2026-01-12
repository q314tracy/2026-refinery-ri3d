// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

  // motor instances
  private final TalonFX m_leftclimber = new TalonFX(k_leftclimberID);
  private final TalonFX m_rightclimber = new TalonFX(k_rightclimberID);
  private final TalonFX m_climberextend = new TalonFX(k_climberextendID);

  // climber PIDF
  private final ProfiledPIDController m_climbPID = new ProfiledPIDController(
      k_climberkP,
      k_climberkI,
      k_climberkD,
      k_climberConstraints);
  private final ElevatorFeedforward m_climbFF = new ElevatorFeedforward(
      k_climberkS,
      k_climberkG,
      k_climberkV);

  // persistent position values
  private double leftpos = 0;
  private double rightpos = 0;

  /** Climber subsystem. Includes deploying and climbing. */
  public Climber() {
    m_leftclimber.getConfigurator().apply(k_climberconfig);
    m_rightclimber.getConfigurator().apply(k_climberconfig);
    m_climberextend.getConfigurator().apply(k_climberextendconfig);
  }

  /** Deploys the climber assembly. */
  public void deployExtender() {
    m_climberextend.setVoltage(3);
  }

  /** Retracts the climber assembly. */
  public void retractExtender() {
    m_climberextend.setVoltage(-3);
  }

  /** Stops the climber extension motor. */
  public void stopExtender() {
    m_climberextend.stopMotor();
  }

  /** Extender current for sensing when at mechanical stops. */
  public double extenderCurrent() {
    return m_climberextend.getStatorCurrent().getValue().in(Units.Amps);
  }

  /** Retracts the climber with PID control. */
  public void climberRetractClosedLoop(double setpoint) {
    m_climbPID.setGoal(setpoint);
    double pid = m_climbPID.calculate((leftpos + rightpos) / 2);
    double ff = m_climbFF.calculate(m_climbPID.getSetpoint().velocity);
    m_leftclimber.setVoltage(ff + pid);
    m_rightclimber.setVoltage(ff + pid);
  }

  /** Retracts the climber using a FF only. */
  public void climberRetractOpenLoop(double velocitysp) {
    double ff = m_climbFF.calculate(velocitysp);
    m_leftclimber.setVoltage(ff);
    m_rightclimber.setVoltage(ff);
  }

  /** Extends the climbers out to their high limit. */
  public void climberExtend() {
    m_leftclimber.setVoltage(leftpos < k_highlimit ? 6 : 0);
    m_rightclimber.setVoltage(rightpos < k_highlimit ? 6 : 0);
  }

  public void leftClimberTest(double volts) {
    m_leftclimber.setVoltage(volts);
  }
  public void rightClimberTest(double volts) {
    m_rightclimber.setVoltage(volts);
  }

  /**
   * Re-synchronizes the climbers if they go out of sync. Currently set to extend
   * out until in tolerance.
   */
  public void synchronize() {
    if (!isRational()) {
      if (leftpos > rightpos)
        m_rightclimber.setVoltage(6);
      if (rightpos > leftpos)
        m_leftclimber.setVoltage(6);
    }
  }

  /**
   * Returns a boolean stating if left position and right position are within
   * tolerance of each other.
   */
  public boolean isRational() {
    return MathUtil.isNear(leftpos, rightpos, k_positiontolerance);
  }

  /** Returns a pair of doubles containing the positions. Left is first, right is second. */
  public Pair<Double,Double> climberPositions() {
    return Pair.of(leftpos, rightpos);
  }

  @Override
  public void periodic() {
    // update positions periodically
    leftpos = (m_leftclimber.getPosition().getValue().in(Units.Rotations) / k_climberratio) * k_climberwinchcirc;
    rightpos = (m_rightclimber.getPosition().getValue().in(Units.Rotations) / k_climberratio) * k_climberwinchcirc;
  }
}
