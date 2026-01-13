// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.utils.Constants.ShooterConstants.k_shootermaxvel;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.mechanisms.Climber;
import frc.robot.subsystems.mechanisms.Feeder;
import frc.robot.subsystems.mechanisms.Hopper;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.Shooter;
import frc.robot.utils.SwerveTelemetry;
import frc.robot.utils.SwerveTunerConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.SwerveConstants;

public class RobotContainer {

  // joystick
  private final CommandGenericHID m_driverctlr = new CommandGenericHID(0);

  // other subsystems
  private final SwerveTelemetry m_swervetelemetry = new SwerveTelemetry(SwerveConstants.k_maxlinspeed);
  private final Swerve m_swerve = SwerveTunerConstants.createDrivetrain();
  private final Climber m_climber = new Climber();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();

  // misc vars
  private double shootervel = 0;
  private boolean hopperdeployed = false;

  public RobotContainer() {
    // pathfinding algo
    Pathfinding.setPathfinder(new LocalADStar());
    m_swerve.configureAutobuilder();
    // default commands
    m_shooter.setDefaultCommand(shooterDefault());
    m_swerve.setDefaultCommand(swerveDefault());
    // start telemtry for swerve
    m_swerve.registerTelemetry(m_swervetelemetry::telemeterize);
    // configure triggers
    configureBindings();
  }

  private void configureBindings() {
    // idles drivetrain when disabled
    RobotModeTriggers.disabled().whileTrue(
        m_swerve.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));
    // drivetrain bindings
    m_driverctlr.button(6).whileTrue(
      m_swerve.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
    );
    m_driverctlr.button(1).whileTrue(runEnd(() -> m_feeder.feed(), () -> m_feeder.stop(), m_feeder));
    m_driverctlr.button(2).whileTrue(runEnd(() -> m_shooter.openLoop(k_shootermaxvel), () -> m_shooter.stopShooter(), m_shooter));
    m_driverctlr.povLeft().whileTrue(runEnd(() -> m_hopper.retract(), () -> m_hopper.stop(), m_hopper));
    m_driverctlr.povRight().whileTrue(runEnd(() -> m_hopper.extend(), () -> m_hopper.stop(), m_hopper));
    m_driverctlr.button(3).whileTrue(runEnd(() -> m_intake.intake(), () -> m_intake.stop(), m_intake));
    m_driverctlr.povUp().whileTrue(runEnd(() -> m_climber.climberManual(3), () -> m_climber.climberStop(), m_climber));
    m_driverctlr.povDown().whileTrue(runEnd(() -> m_climber.climberManual(-3), () -> m_climber.climberStop(), m_climber));
    m_driverctlr.button(7).onTrue(run(() -> m_climber.climberExtend(), m_climber));
    m_driverctlr.button(8).onTrue(run(() -> m_climber.climberRetractClosedLoop(0)));
    // // shooter bindings
    // m_driverctlr.button(1).whileTrue(
    //     runEnd(() -> shootervel = 10, () -> shootervel = 0));
    // m_driverctlr.button(2).onTrue(
    //     run(() -> m_feeder.feed(), m_feeder)
    //         .withTimeout(1)
    //         .finallyDo(() -> m_feeder.stop()));
    // // hopper bindings
    // m_driverctlr.button(8).onTrue(
    //     runOnce(() -> m_hopper.extend(), m_hopper)
    //         .until(() -> m_hopper.bootyCurrent() > 15)
    //         .finallyDo(() -> runOnce(() -> m_hopper.stop(), m_hopper)));
  }

  /** Default command for the shooter subsytem. */
  public Command shooterDefault() {
    return run(() -> m_shooter.closedLoop(shootervel), m_shooter);
  }

  /** Default operator controlled method for swerve subsystem. */
  public Command swerveDefault() {
    return m_swerve.applyRequest(() -> new SwerveRequest.FieldCentric()
        .withRotationalDeadband(SwerveConstants.k_maxrotspeed * OIConstants.k_deadzone) // 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withVelocityX(MathUtil.applyDeadband(-m_driverctlr.getRawAxis(1), OIConstants.k_deadzone) * SwerveConstants.k_maxlinspeed * 0.1) // x velocity
        .withVelocityY(MathUtil.applyDeadband(-m_driverctlr.getRawAxis(0), OIConstants.k_deadzone) * SwerveConstants.k_maxlinspeed * 0.1) // y velocity
        .withRotationalRate(MathUtil.applyDeadband(-m_driverctlr.getRawAxis(4), OIConstants.k_deadzone) * SwerveConstants.k_maxrotspeed) // z rot velocity
    );
  }

  public Command getAutonomousCommand() {
    return print("no command configured");
  }
}
