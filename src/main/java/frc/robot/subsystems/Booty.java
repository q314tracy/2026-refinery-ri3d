// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.IntakeConstants.*;

public class Booty extends SubsystemBase {

  private final TalonFX m_intakerollers = new TalonFX(k_intakerollerID);
  private final TalonFX m_intakeextend = new TalonFX(k_intakeextendID);
  private final TalonFXConfigurator m_intakerollersconfigurator = m_intakerollers.getConfigurator();
  private final TalonFXConfigurator m_intakeextendconfigurator = m_intakerollers.getConfigurator();

  public Booty() {
    m_intakerollersconfigurator.apply(k_intakerollerconfig);
    m_intakeextendconfigurator.apply(k_intakeextendconfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
