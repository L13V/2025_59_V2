
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.EndevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.EndEvatorSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem.BallIntakeState;
import frc.robot.subsystems.EndEvatorSubsystem.EndEvatorState;
import java.io.File;
// import frc.robot.Robot.OverallRobotState;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final EndEvatorSubsystem m_endevator = new EndEvatorSubsystem();
  private final BallIntakeSubsystem m_ballintake = new BallIntakeSubsystem();
  // private final Robot m_robot = new Robot();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1 * drivebase.drivemultiplier,
      () -> driverXbox.getLeftX() * -1 * drivebase.drivemultiplier)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1)
      .scaleRotation(-1)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    // NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    if (Robot.isSimulation()) {
      // Pose2d target = new Pose2d(new Translation2d(1, 4),
      // Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    /*
     * L1
     */
    driverXbox.rightBumper().and(m_endevator.readyToRaiseWithCoralSupplier)
        .onTrue(m_endevator.setTo(EndEvatorState.L1));
    /*
     * L2
     */
    driverXbox.leftBumper().and(m_endevator.readyToRaiseWithCoralSupplier).and(m_endevator.isNotAt(EndEvatorState.L2))
        .and(m_endevator.isNotAt(EndEvatorState.L2_Score))
        .onTrue(m_endevator.setTo(EndEvatorState.L2));
    driverXbox.leftBumper().and(m_endevator.readyToRaiseWithCoralSupplier).and(m_endevator.isAt(EndEvatorState.L2))
        .and(m_endevator.elevatorAtTargetPosition(EndevatorConstants.L2_height))
        .onTrue(m_endevator.setTo(EndEvatorState.L2_Score));
    /*
     * L3
     */
    driverXbox.leftTrigger().and(m_endevator.readyToRaiseWithCoralSupplier).and(m_endevator.isNotAt(EndEvatorState.L3))
        .and(m_endevator.isNotAt(EndEvatorState.L3_Score))
        .onTrue(m_endevator.setTo(EndEvatorState.L3));
    driverXbox.leftTrigger().and(m_endevator.readyToRaiseWithCoralSupplier).and(m_endevator.isAt(EndEvatorState.L3))
        .and(m_endevator.elevatorAtTargetPosition(EndevatorConstants.L3_height))
        .onTrue(m_endevator.setTo(EndEvatorState.L3_Score));
    /*
     * L4
     */
    driverXbox.rightTrigger().and(m_endevator.readyToRaiseWithCoralSupplier).and(m_endevator.hasNoAlgaeSupplier)
        .and(m_endevator.isNotAt(EndEvatorState.L4))
        .and(m_endevator.notatElevatorTargetPosition(EndevatorConstants.L4_height))
        .onTrue(m_endevator.setTo(EndEvatorState.L4));
    driverXbox.rightTrigger().and(m_endevator.readyToRaiseWithCoralSupplier).and(m_endevator.isAt(EndEvatorState.L4))
        .and(m_endevator.elevatorAtTargetPosition(EndevatorConstants.L4_height))
        .onTrue(m_endevator.setTo(EndEvatorState.L4_Score));
    /*
     * Barge Stuff
     */
    driverXbox.rightTrigger().and(m_endevator.readyToRaiseWithAlgaeSupplier).and(m_endevator.hasNoCoralSupplier)
        .and(m_endevator.isNotAt(EndEvatorState.FLICK)).onTrue(m_endevator.setTo(EndEvatorState.BARGE));
    driverXbox.rightTrigger().and(m_endevator.readyToRaiseWithAlgaeSupplier).and(m_endevator.hasNoCoralSupplier)
        .and(m_endevator.isAt(EndEvatorState.BARGE))
        .and(m_endevator.elevatorAtTargetPosition(EndevatorConstants.barge_height))
        .onTrue(m_endevator.setTo(EndEvatorState.FLICK));
    /*
     * Floor Intakes
     */
    driverXbox.rightTrigger().and(m_endevator.notReadyToRaiseWithCoralSupplier).and(m_endevator.hasNoAlgaeSupplier)
        .onTrue(m_endevator.setTo(EndEvatorState.CORAL_FLOOR_INTAKE)).onFalse(m_endevator.setTo(EndEvatorState.STOW));
    driverXbox.leftTrigger().and(m_endevator.notReadyToRaiseWithAlgaeSupplier).and(m_endevator.hasNoCoralSupplier)
        .onTrue(m_endevator.setTo(EndEvatorState.ALGAE_FLOOR_INTAKE)).onFalse(m_endevator.setTo(EndEvatorState.STOW));
        
    /*
     * Stow
     */
    driverXbox.start().onTrue(m_endevator.setTo(EndEvatorState.STOW));
    /*
     * Ball Intake
     */
    driverXbox.a().onTrue(m_ballintake.setTo(BallIntakeState.INTAKE)).onFalse(m_ballintake.setTo(BallIntakeState.STOW));
    driverXbox.b().onTrue(m_ballintake.setTo(BallIntakeState.SCORE)).onFalse(m_ballintake.setTo(BallIntakeState.STOW));
    /*
     * Reef Intake
     */
    driverXbox.rightBumper().and(m_endevator.hasNoAlgaeSupplier).and(m_endevator.hasNoCoralSupplier)
        .onTrue(m_endevator.setTo(EndEvatorState.LOW_ALGAE_INTAKE));
    driverXbox.leftBumper().and(m_endevator.hasNoAlgaeSupplier).and(m_endevator.hasNoCoralSupplier)
        .onTrue(m_endevator.setTo(EndEvatorState.HIGH_ALGAE_INTAKE));
    // driverXbox.b().and(m_endevator.coralSupplier);

    // driverXbox.a().onTrue(m_endevator.moveElevatorTo(ElevatorState.L2));
    // driverXbox.b().and(() ->
    // m_endevator.readyToStow()).onTrue(m_endevator.setTo(ElevatorState.STOW));

    driverXbox.back().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
    // drivebase).repeatedly());
    // driverXbox.rightBumper().onTrue(drivebase.goSlow()).onFalse(drivebase.goFast());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Auto");
    return null;
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
