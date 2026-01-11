// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

  public class OIConstants {
    public static final double k_deadzone = 0.2;
    public static final double k_maxlinspeedteleop = 3;
    public static final double k_maxrotspeedteleop = 2 * Math.PI;
  }

  public class SwerveDriveConstants {
    public static final double k_wheelradius = Units.inchesToMeters(2);
    public static final double k_wheelcircumference = 2 * Math.PI * k_wheelradius;
    public static final double k_trackwidth = Units.inchesToMeters(24);
    public static final double k_drivegearratio = 5.27;
    public static final double k_turngearratio = 26.09;
    public static final double k_drivemotormaxRPM = 6000;
    public static final double k_maxlinspeed = (k_drivemotormaxRPM / k_drivegearratio) * k_wheelcircumference / 60; // meters/sec
    public static final double k_maxrotspeed = (2 * k_maxlinspeed) / k_trackwidth;
    public static final Pose2d k_initpose = new Pose2d(2, 2, new Rotation2d());
  }

  public class ShooterConstants {

    public static final int k_shootermotorID = 11;
    public static final int k_shooterintakeID = 12;
    public static final TalonFXConfiguration k_shooterconfig = new TalonFXConfiguration();
    static {
      k_shooterconfig.CurrentLimits.StatorCurrentLimit = 60;
      k_shooterconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_shooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_shooterconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }    
    public static final TalonFXConfiguration k_shooterintakeconfig = new TalonFXConfiguration();
    static {
      k_shooterintakeconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_shooterintakeconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_shooterintakeconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      k_shooterintakeconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public static final double k_shooterwheeldiam = Units.inchesToMeters(4); //meters
    public static final double k_shooterwheelcircum = Math.PI * k_shooterwheeldiam;

    public static final double k_velocitykP = 0.1;
    public static final double k_velocitykI = 0;
    public static final double k_velocitykD = 0;
    public static final double k_velocitykS = 0; //characterize pls
    public static final double k_velocitykV = 0; //characterize pls
    public static final TrapezoidProfile.Constraints k_velocityConstraints = //characterize pls
      new TrapezoidProfile.Constraints(
        k_velocitykD,
        k_shootermotorID);
  }

  public static final class PathfindingConstants {

  }
}
