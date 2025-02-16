// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team7520.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.team7520.robot.subsystems.climber.ClimberSubsystem;
//import frc.team7520.robot.subsystems.LED;
//import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
//import frc.team7520.robot.subsystems.amp.AmpSubsystem;
//import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.*;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable table = inst.getTable("noteTable");

    // get a topic from a NetworkTableInstance
    // the topic name in this case is the full name

    //"Detections" supplies all notes detected, MaxConfObj gives only one
    public StringTopic StrTopic = inst.getStringTopic("/noteTable/Detections");
    public static boolean speakerRoutineActivateShooter = false;

    public final static Map map = new Map();

    private boolean notePathTrigger = false;
    private boolean chainingPathTrigger = false;
    private boolean dynamicStart = false;


    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/Swerve3Neo"));

    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    // private final XboxController driverController =
    // new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController = 
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> driverController.getLeftY() * -1,
        () -> driverController.getLeftX() * -1)
        .withControllerRotationAxis(driverController::getRightX)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(
        driverController::getRightX,
        driverController::getRightY)
        .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
        .robotRelative(true)
        .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX())
        .withControllerRotationAxis(() -> driverController.getRawAxis(2))
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
        .withControllerHeadingAxis(
        () -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2), 
        () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
        .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

        if (RobotBase.isSimulation()) {
                drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else {
                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        registerAutos();

        CameraServer.startAutomaticCapture();

        // Configure the trigger bindings
        configureBindings();
    }

    private void registerAutos(){

    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands() {

    }



    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      new JoystickButton(driverController, XboxController.Button.kA.value)
               .onTrue(Commands.runOnce(drivebase::zeroGyro));
        // X/Lock wheels
      new JoystickButton(driverController, XboxController.Button.kB.value)
               .whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock)));
      
      new Trigger(() -> driverController.getPOV() == 90)
            .onTrue(new InstantCommand(() -> {
                  var cmd = AutoBuilder.followPath(drivebase.GoRight());
                  cmd.schedule();}
            ));
      
      new Trigger(() -> driverController.getPOV() == 270)
            .onTrue(new InstantCommand(() -> {
                  var cmd = AutoBuilder.followPath(drivebase.GoLeft());
                  cmd.schedule();}
            ));

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

      if (RobotBase.isSimulation())
      {
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      } else
      {
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      }
    }

    /**
     * Path used in 15s auto chaining for returning shots to alliance. When there is a note inside the intake, the bot will
     * go to the chaining position and shoot. If no note was picked up, there is no recursion/looping.
     * @param noteInsideIntake a boolean indicating whether the intake has a note. Comes from intakeSubsystem.getSwitchVal()
     * @return
     */

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }

    public void teleOpInit() {
        //shooterSubsystem.setDefaultCommand(shooter);
        chainingPathTrigger = false;
        dynamicStart = false;
        notePathTrigger = false; 
    }
}
