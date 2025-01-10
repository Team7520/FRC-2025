// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team7520.robot.Constants.AmpConstants;
import frc.team7520.robot.Constants.IntakeConstants;
import frc.team7520.robot.Constants.IntakeConstants.Position;
import frc.team7520.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.team7520.robot.auto.AutoIntake;
//import frc.team7520.robot.auto.AutoNotePickUp;
//import frc.team7520.robot.auto.AutoNoteSearch;
//import frc.team7520.robot.auto.AutoShoot;
//import frc.team7520.robot.auto.AutoTurn;
//import frc.team7520.robot.auto.ShootSequence;
//import frc.team7520.robot.commands.AbsoluteDrive;
//import frc.team7520.robot.commands.Climber;
//import frc.team7520.robot.commands.Intake;
//import frc.team7520.robot.commands.Shooter;
import frc.team7520.robot.commands.AbsoluteDrive;
//import frc.team7520.robot.commands.Amp;
//import frc.team7520.robot.commands.AutoClimber;
import frc.team7520.robot.commands.TeleopDrive;
//import frc.team7520.robot.subsystems.climber.ClimberSubsystem;
//import frc.team7520.robot.subsystems.LED;
//import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
//import frc.team7520.robot.subsystems.amp.AmpSubsystem;
//import frc.team7520.robot.subsystems.shooter.ShooterSubsystem;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.*;

import java.io.File;
import java.util.function.BooleanSupplier;

import javax.management.InstanceAlreadyExistsException;

import org.photonvision.estimation.RotTrlTransform3d;

//Temporary Imports BY RObin
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import static frc.team7520.robot.subsystems.LED.candle;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
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
            "swerve/Swerve3Neo"), StrTopic);

    //private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    //private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

//    private final AmpSubsystem ampSubsystem = AmpSubsystem.getInstance();

    //private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    //private final LED LEDSubsystem = LED.getInstance();

    public final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final XboxController operatorController =
            new XboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

            /*
    private final Intake intake = new Intake(intakeSubsystem,
            operatorController::getRightBumper,
            operatorController::getYButton,
            operatorController::getAButton,
            operatorController::getBButton,
            operatorController::getXButton
        );
        */

//        private final Amp amp = new Amp(ampSubsystem,
//                operatorController::getPOV);

    //public Shooter shooter;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {

        registerAutos();

        CameraServer.startAutomaticCapture();

        // Configure the trigger bindings
        configureBindings();

        // Left joystick is the angle of the robot
        AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                () -> MathUtil.applyDeadband(-driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverController.getRightX(),
                () -> -driverController.getRightY(),
                driverController::getRightBumper,
                driverController::getLeftBumper,
                () -> driverController.getXButton()
        );

        /*
         shooter = new Shooter(shooterSubsystem,
                operatorController::getLeftTriggerAxis,
                operatorController::getRightTriggerAxis,
                operatorController::getLeftBumper
        );
        */


        // Climber climber = new Climber(climberSubsystem,
        //         driverController::getYButton,
        //         driverController::getYButton,
        //         operatorController::getStartButton, //Start button not used
        //         operatorController::getRightY,
        //         operatorController::getLeftY,
        //         operatorController::getBackButton
        // );

        /*
        AutoClimber climber = new AutoClimber(climberSubsystem, 
                driverController::getYButtonReleased, 
                operatorController::getLeftY, 
                operatorController::getRightY,
                operatorController::getBackButton
                );
        */

        // Intake intake = new Intake(intakeSubsystem,
        //         operatorController::getRightBumper,
        //         operatorController::getYButton,
        //         operatorController::getAButton,
        //         operatorController::getBButton,
        //         operatorController::getXButton,
        //         intakeSubsystem::getSwitchVal
        // );

        /* 
        Old drive method like in video games, Easier to learn, harder to control, Not tested not used
        TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(driverController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(driverController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> driverController.getRawAxis(2), () -> true);
        */
        drivebase.setDefaultCommand(closedAbsoluteDrive);
//        ampSubsystem.setDefaultCommand(amp);
        //shooterSubsystem.setDefaultCommand(shooter);
        //intakeSubsystem.setDefaultCommand(intake);
        //climberSubsystem.setDefaultCommand(climber);
        //LEDSubsystem.setDefaultCommand(LEDSubsystem.idle());

        //candle.animate(LEDSubsystem.idleAnimation);
    }

    private void registerAutos(){

        registerNamedCommands();

        autoChooser.setDefaultOption("2middleWithAt", drivebase.getPPAutoCommand("2MiddleWithAT", true));
        autoChooser.setDefaultOption("Safe auto", drivebase.getPPAutoCommand("safe", true));
        autoChooser.addOption("Amp", drivebase.getPPAutoCommand("Amp", true));
        autoChooser.addOption("Test", drivebase.getPPAutoCommand("test", true));
        autoChooser.addOption("BotToCentBot", drivebase.getPPAutoCommand("BotToCentBot", true));
        autoChooser.addOption("MidToCentTop", drivebase.getPPAutoCommand("MidToCentTop", true));
        autoChooser.addOption("TopToCentTop", drivebase.getPPAutoCommand("TopToCentTop", true));
        autoChooser.addOption("2Note", drivebase.getPPAutoCommand("2NoteMid", true));
        autoChooser.addOption("3NoteMid.Note1", drivebase.getPPAutoCommand("3NoteMid.Note1", true));
        autoChooser.addOption("3NoteMid.Note3", drivebase.getPPAutoCommand("3NoteMid.Note3", true));
        autoChooser.addOption("4Note", drivebase.getPPAutoCommand("4Note", true));
        autoChooser.addOption("4Note(StraightReturn)", drivebase.getPPAutoCommand("4Note(StraightReturn)", true));
        autoChooser.addOption("2NoteSpeakerS.Note3", drivebase.getPPAutoCommand("2NoteSpeakerS.Note3", true));
        autoChooser.addOption("3NoteSpeakerC.Note2.SpeakerC.Note5.SpeakerC", drivebase.getPPAutoCommand("3NoteSpeakerC.Note2.SpeakerC.Note5.SpeakerC", true));

        // 1note shoot and Speaker Source side to note8 parking
        autoChooser.addOption("SpeakerS.Note8", drivebase.getPPAutoCommand("SpeakerS.Note8", true));

        // 2note Ampside
        autoChooser.addOption("SpeakerA.Note1.SpeakerA", drivebase.getPPAutoCommand("SpeakerA.Note1.SpeakerA", true));
        // 2note Ampside with move to center
        autoChooser.addOption("SpeakerA.Note1.SpeakerA.Note4", drivebase.getPPAutoCommand("SpeakerA.Note1.SpeakerA.Note4", true));
        autoChooser.addOption("SpeakerS.Note8.SpeakerS", drivebase.getPPAutoCommand("SpeakerS.Note8.SpeakerS", true));
        autoChooser.addOption("4NoteButFaster", drivebase.getPPAutoCommand("4NoteButFaster", true));

        // Troll Auto
        autoChooser.addOption("TrollAuto3NoteFeed", drivebase.getPPAutoCommand("TrollAuto3NoteFeed", true));
        //autoChooser.addOption("Just shoot", new ShootSequence());
        SmartDashboard.putData(autoChooser);
    }

    /**
     * Use this method to define named commands for use in {@link PathPlannerAuto}
     *
     */
    private void registerNamedCommands()
    {
        // Example
        /*
        NamedCommands.registerCommand("shoot", new ShootSequence());
        NamedCommands.registerCommand("log", new InstantCommand(() -> System.out.println("eeeeeeeeeeeeeeeeeeeeeeeee")));
        NamedCommands.registerCommand("intakeOut", new AutoIntake(Position.INTAKE));
        NamedCommands.registerCommand("intake", new InstantCommand(() -> intakeSubsystem.setSpeed(Position.INTAKE.getSpeed())));
        NamedCommands.registerCommand("stopIntaking", new InstantCommand(() -> intakeSubsystem.setSpeed(0)));
        NamedCommands.registerCommand("intakeIn", new AutoIntake(Position.SHOOT));
        NamedCommands.registerCommand("stopShoot", new AutoShoot(0, false));
        NamedCommands.registerCommand("AutoNotePickUp", new AutoNotePickUp());
*/
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
    private void configureBindings()
    {
        // Zero gyro
        new JoystickButton(driverController, XboxController.Button.kA.value)
                .onTrue(new InstantCommand(drivebase::zeroGyro));
        // X/Lock wheels
        new JoystickButton(driverController, XboxController.Button.kX.value)
                .whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock)));

        /* // Item below is not to be used for a drive base using ABSOLUTE COORDINATES on field
        new JoystickButton(driverController, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(drivebase::resetOdometry));
        */

        /* Green when note is in, yellow when note detected, rainbow for neither */
        /*
        new Trigger(intakeSubsystem::getSwitchVal)
               .whileFalse(new RepeatCommand(LEDSubsystem.noteIn()))
                .onTrue(LEDSubsystem.idle());


        new Trigger(drivebase::getNoteAvailable).and(intakeSubsystem::getSwitchVal)
                .whileTrue(new RepeatCommand(LEDSubsystem.noteAvailable()));

*/
        
        /* OTF Path Note using sensor feedback */
        /*
        new JoystickButton(driverController, XboxController.Button.kB.value).and(intakeSubsystem::getSwitchVal)
                .onTrue(notePickUp(false));
*/

        /* If joysticks are moved while a path is in session, the path is overrided */
        /*
        new Trigger(() -> SwerveSubsystem.pathActive)
                .and(() -> (Math.abs(driverController.getLeftX()) > 0.2 || Math.abs(driverController.getLeftY()) > 0.2 || Math.abs(driverController.getRightX()) > 0.2 || Math.abs(driverController.getRightY()) > 0.2))
                .onTrue(new InstantCommand(() -> {
                                var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(-1, new Pose2d(), new Rotation2d(), new Rotation2d()));
                                cmd.schedule();

                        }));
*/

        /* OTF Path Shooter timed */
        /*
        new Trigger(() -> driverController.getPOV() == 0)
                .onTrue(centralSpeakerShot());

        new Trigger(() -> driverController.getPOV() == 270)
                .onTrue(leftSpeakerShot());
        
        new Trigger(() -> driverController.getPOV() == 90)
                .onTrue(rightSpeakerShot());
        new Trigger(() -> driverController.getPOV() == 180)
                .onTrue(ampShot());

        new Trigger(() -> speakerRoutineActivateShooter)
                .onTrue(new ParallelCommandGroup(
                        new ShootSequence(),
                        new InstantCommand(() -> {speakerRoutineActivateShooter = false;})
                ));

        new Trigger(() -> dynamicStart)
                .onTrue(new InstantCommand(() -> {
                        dynamicStart = false;
                }).andThen(notePickUp(true)));

                */
        
        /* For chaining OTF Note Auto */
        /*
        new Trigger(() -> !SwerveSubsystem.pathActive && notePathTrigger)
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> {notePathTrigger = false;}),
                        // new AutoIntake(Position.SHOOT),
                        //new InstantCommand(() -> {intakeSubsystem.setSpeed(0);}),
                        new AutoNoteSearch(drivebase).onlyIf(intakeSubsystem::getSwitchVal),
                        notePickUp(true).onlyIf(intakeSubsystem::getSwitchVal),
                        // new AutoIntake(Position.SHOOT),
                        // new InstantCommand(() -> {intakeSubsystem.setSpeed(0);}),
                        autoChaining(!intakeSubsystem.getSwitchVal())
                )
                // .finallyDo((boolean interrupted) -> {
                //         System.out.println("NOTE END INTERRUPTION: " + interrupted);
                //         intakeSubsystem.setSpeed(0);
                //         intakeSubsystem.setPosition(Position.SHOOT);
                // })
                );

        new Trigger(() -> !SwerveSubsystem.pathActive && chainingPathTrigger)
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> {chainingPathTrigger = false;}),
                        new WaitCommand(0.5),
                        notePickUp(true)
                )
                // .finallyDo((boolean interrupted) -> {
                //         System.out.println("CHAIN END INTERRUPTED: " + interrupted);
                //         intakeSubsystem.setSpeed(0);
                //         intakeSubsystem.setPosition(Position.SHOOT);
                // })
                );
                */
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*
    public Command getAutonomousCommand() {
        if (autoChooser.getSelected().getName().equals("2MiddleWithAT") || autoChooser.getSelected().getName().equals("2middleWithAt")) { //The named command is not what is displayed on sendable chooser, but rather the name of the auto as it is written in path planner GUI
                return new SequentialCommandGroup(
                                autoChooser.getSelected()
                                //notePickUp(true)
                
                ).finallyDo((boolean interupted) -> {
                        shooterSubsystem.setDefaultCommand(shooter);
                        dynamicStart = true;
                        // intakeSubsystem.setSpeed(0);
                        // intakeSubsystem.setPosition(Position.SHOOT);
                });     
        } else {
                return new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                autoChooser.getSelected()
                                //new InstantCommand(() -> shooterSubsystem.setDefaultCommand(new AutoShoot(0.7, false)))
                                
                        ),
                        new InstantCommand(() -> new AutoShoot(0, false)),
                        new InstantCommand(() -> shooterSubsystem.setDefaultCommand(shooter))
                ).finallyDo((boolean interupted) -> {
                        shooterSubsystem.setDefaultCommand(shooter);
                });
        }
        
        
        /* 
        //Robin's 1m Path Test 
        PathPlannerPath path = PathPlannerPath.fromPathFile("OneMeterByRobinTest");
        return AutoBuilder.followPath(path);
        * /
        
    }
    */

    public void activateSpeakerRoutine() {
        speakerRoutineActivateShooter = true;
    }

    /**
     * Runs OTF path to note and full intake sequence using sensor in parallel
     * @param chaining a boolean indicating whether the command is used for 15s auto or not
     * @return the command for autoNotePickUp
     */
    /*
    public Command notePickUp(boolean chaining) {
        if (chaining) {
                return new AutoTurn(drivebase, 0, null)
                .andThen(new WaitCommand(0.2)) //to give tpu time to get accurate average of note
                .andThen(new InstantCommand(() -> {   
                        System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa");                     
                        SwerveSubsystem.pathActive = true;
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(0, new Pose2d(), new Rotation2d(), new Rotation2d()));
                        cmd.schedule();
                }).finallyDo((boolean interrupted) -> {
                        notePathTrigger = true; 
                }));
        } else {
                return new AutoTurn(drivebase,  0, null)
                .andThen(new WaitCommand(0.2))
                .andThen(new InstantCommand(() -> {                        
                        SwerveSubsystem.pathActive = true;
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(0, new Pose2d(), new Rotation2d(), new Rotation2d()));
                        cmd.schedule();
                        
                }));
        }
         
    }
        */

    /**
     * Runs OTF path to center speaker position and shoot sequence, timed
     * @return
     */
    /*
    public Command centralSpeakerShot() {
                return new InstantCommand(() -> {
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(
                                1, 
                                map.getSpeakerCenter(), 
                                new Rotation2d(map.getSpeakerCenter().getRotation().getRadians() + Math.PI), 
                                new Rotation2d(map.getSpeakerCenter().getRotation().getRadians() + Math.PI))
                        );
                        cmd.schedule();                
                });
    }
                */

    /**
     * Runs OTF path to left side speaker position and shoot sequence, timed
     * @return
     */
    /*
    public Command leftSpeakerShot() {
                return new InstantCommand(() -> {
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(
                                1, 
                                map.getSpeakerLeftSide(), 
                                new Rotation2d(map.getSpeakerLeftSide().getRotation().getRadians() + Math.PI), 
                                new Rotation2d(map.getSpeakerLeftSide().getRotation().getRadians() + Math.PI))
                        );
                        cmd.schedule();                
                });
    }
*/

    /**
     * Runs OTF path to right side speaker position and shoot sequence, timed
     * @return
     */
    /*
    public Command rightSpeakerShot() {
                return new InstantCommand(() -> {
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(
                                1, 
                                map.getSpeakerRightSide(), 
                                new Rotation2d(map.getSpeakerRightSide().getRotation().getRadians() + Math.PI), 
                                new Rotation2d(map.getSpeakerRightSide().getRotation().getRadians() + Math.PI))
                        );
                        cmd.schedule();                
                });
    }
*/

    /**
     * Runs the OTF path to the amp
     * @return
     */
    /*
    public Command ampShot() {
        return new InstantCommand(() -> {
                        var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(
                                1, 
                                map.getAmp(), 
                                new Rotation2d(map.getAmp().getRotation().getRadians() + Math.PI), 
                                new Rotation2d(map.getAmp().getRotation().getRadians() + Math.PI))
                        );
                        cmd.schedule();                
                });
    }
*/

    /**
     * Path used in 15s auto chaining for returning shots to alliance. When there is a note inside the intake, the bot will
     * go to the chaining position and shoot. If no note was picked up, there is no recursion/looping.
     * @param noteInsideIntake a boolean indicating whether the intake has a note. Comes from intakeSubsystem.getSwitchVal()
     * @return
     */
    /*
    public Command autoChaining(boolean noteInsideIntake) {
        if (noteInsideIntake) {
                return new SequentialCommandGroup(
                        new AutoIntake(Position.SHOOT),
                        new InstantCommand(() -> {intakeSubsystem.setSpeed(0);}),
                        new InstantCommand(() -> {
                        
                                SwerveSubsystem.pathActive = true;
                                var cmd = AutoBuilder.followPath(drivebase.sophisticatedOTFPath(
                                        1, 
                                        map.getAutoChaining(), 
                                        new Rotation2d(map.getAutoChaining().getRotation().getRadians() + Math.PI), 
                                        new Rotation2d(map.getAutoChaining().getRotation().getRadians() + Math.PI))
                                        // map.getSpeakerCenter(), 
                                        // new Rotation2d(map.getSpeakerCenter().getRotation().getRadians() + Math.PI), 
                                        // new Rotation2d(map.getSpeakerCenter().getRotation().getRadians() + Math.PI))
                                );
                                cmd.schedule();                
                        })
                ).finallyDo((boolean interrupted) -> {
                        //chainingPathTrigger = true;
                });
        } else {
                return new InstantCommand(() -> {
                        intakeSubsystem.setSpeed(0);
                        intakeSubsystem.setPosition(Position.SHOOT);
                });
        }
                
    }
*/

    public void teleOpInit() {
        //shooterSubsystem.setDefaultCommand(shooter);
        chainingPathTrigger = false;
        dynamicStart = false;
        notePathTrigger = false;
    }

}
