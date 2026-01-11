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
    // may or may not be needed
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
    // motor configuration
    public static final int k_shooter1ID = 18;
    public static final int k_shooter2ID = 19;
    public static final TalonFXConfiguration k_shooterconfig = new TalonFXConfiguration();
    static {
      k_shooterconfig.CurrentLimits.StatorCurrentLimit = 60;
      k_shooterconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_shooterconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_shooterconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    // misc shooter constants
    public static final double k_shooterwheeldiam = Units.inchesToMeters(4); //meters
    public static final double k_shooterwheelcircum = Math.PI * k_shooterwheeldiam; //meters
    public static final double k_shooterratio = 1.333;
    public static final double k_shootermotormaxspeed = 6000.0; //rpm
    public static final double k_shootermaxvel = ((k_shootermotormaxspeed / k_shooterratio) * k_shooterwheelcircum) / 60; //m/s
    // PIDF constants, needs characterized
    public static final double k_velocitykP = 0.1;
    public static final double k_velocitykI = 0;
    public static final double k_velocitykD = 0;
    public static final double k_velocitykS = 0; // characterize pls
    public static final double k_velocitykV = 12 / k_shootermaxvel;
    public static final TrapezoidProfile.Constraints k_velocityConstraints =
        new TrapezoidProfile.Constraints(
            k_shootermotormaxspeed,
            k_shootermotormaxspeed);
  }

  public class FeederConstants {
    public static final int k_shooterfeed1ID = 20;
    public static final int k_shooterfeed2ID = 21;
    public static final TalonFXConfiguration k_shooterfeedconfig = new TalonFXConfiguration();
    static {
      k_shooterfeedconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_shooterfeedconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_shooterfeedconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      k_shooterfeedconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
  }

  public class IntakeConstants {
    // motor configuration
    public static final int k_intakerollerID = 13;
    public static final TalonFXConfiguration k_intakerollerconfig = new TalonFXConfiguration();
    static {
      k_intakerollerconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_intakerollerconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_intakerollerconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_intakerollerconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
  }

  public class BootyConstants {
    // motor configuration
    public static final int k_hopperextendID = 14;
    public static final TalonFXConfiguration k_hopperextendconfig = new TalonFXConfiguration();
    static {
      k_hopperextendconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_hopperextendconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_hopperextendconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      k_hopperextendconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
  }

  public class ClimberConstants {
    // climber lift motor configuration
    public static final int k_leftclimberID = 15;
    public static final int k_rightclimberID = 16;
    public static final TalonFXConfiguration k_climberconfig = new TalonFXConfiguration();
    static {
      k_climberconfig.CurrentLimits.StatorCurrentLimit = 60;
      k_climberconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_climberconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      k_climberconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    // climber extend motor configuration
    public static final int k_climberextendID = 17;
    public static final TalonFXConfiguration k_climberextendconfig = new TalonFXConfiguration();
    static {
      k_climberextendconfig.CurrentLimits.StatorCurrentLimit = 20;
      k_climberextendconfig.CurrentLimits.StatorCurrentLimitEnable = true;
      k_climberextendconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      k_climberextendconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    public static final double k_highlimit = Units.inchesToMeters(18); // meters, need to check
    public static final double k_lowlimit = 0; // meters
    public static final double k_positiontolerance = Units.inchesToMeters(1); //meters
    public static final double k_climberratio = 20; //TBD
    public static final double k_climberwinchdiam = 0.0; //meters
    public static final double k_climberwinchcirc = k_climberwinchdiam * Math.PI; //meters
    public static final double k_climbermotormaxspeed = 6000; //rpm, currently kraken x60
    public static final double k_climbermaxvel = ((k_climbermotormaxspeed / k_climberratio) * k_climberwinchcirc) / 60; //m/s
    // PIDF for climbing
    public static final double k_climberkP = 0.1;
    public static final double k_climberkI = 0;
    public static final double k_climberkD = 0;
    public static final double k_climberkS = 0; //tune me pls
    public static final double k_climberkG = 0; //tune me pls
    public static final double k_climberkV = 12 / k_climbermaxvel;
    public static final TrapezoidProfile.Constraints k_climberConstraints = 
        new TrapezoidProfile.Constraints(
            k_climbermaxvel, //m/s
            k_climbermaxvel); //m/s^2
  }

  public static final class PathfindingConstants {

  }
}
