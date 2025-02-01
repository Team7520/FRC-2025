// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import frc.team7520.robot.Constants;
//import frc.team7520.robot.auto.AutoIntake;
//import frc.team7520.robot.auto.AutoNotePickUp;
//import frc.team7520.robot.auto.AutoShoot;
//import frc.team7520.robot.auto.ShootSequence;
//import frc.team7520.robot.subsystems.intake.IntakeSubsystem;
import frc.team7520.robot.util.AprilTagSystem;
import frc.team7520.robot.util.LimelightHelpers;
import frc.team7520.robot.util.TpuSystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.Meter;


import static frc.team7520.robot.Constants.Telemetry.SWERVE_VERBOSITY;

public class SwerveSubsystem extends SubsystemBase {

    /**
     * Swerve drive object.
     */
    private final SwerveDrive swerveDrive;
    /**
     * Maximum speed of the robot in meters per second, used to limit acceleration.
     */
    public double maximumSpeed = Units.feetToMeters(14.5); //14.5
    /**
     * AprilTagSystem
     */
    public AprilTagSystem aprilTagSystem = new AprilTagSystem("AprilCam");
    int counter = 0;
    
    /**
     * TpuSystem
     */
    public Translation2d notePose = new Translation2d();
    public TpuSystem tpuSystem;
    public boolean noteAvailable = false;

    /**
     * ABSOLUTE COORDINATE variables. Used by other classes.
     */
    public static boolean isBlueAlliance = false;
    private boolean driverStationReady = false;

    public static boolean pathActive = false;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
                try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
//             swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d(new Translation2d(Meter.of(1),
//             Meter.of(4)),
// Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1);
                                                swerveDrive.setMotorIdleMode(true);

    }

    public boolean getNoteAvailable() {
        return noteAvailable;
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */

     
    public void setupPathPlanner() {
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();   
            AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName       PathPlanner path name.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    /*
    public Command getPathCommand(String pathName, boolean setOdomToStart) {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
    */
    
    /**
     * Get the autonomous command for the robot.
     * @param autoName       Name of the auto file.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link PathPlannerAuto} command.
     */
    public Command getPPAutoCommand(String autoName, boolean setOdomToStart) {
        if (setOdomToStart) {
            SmartDashboard.putNumber("HeadingFromFile", -1);
//            resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
        }
        return new PathPlannerAuto(autoName);
    }



    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {      
        Pose2d startingPose = new Pose2d();  
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed, startingPose);
    }

    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative,false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    @Override
    public void periodic() {

        if(true) {
            return;
        }
        
        if (!driverStationReady) {
            driverStationReady = DriverStation.getAlliance().isPresent();
        } else {
            isBlueAlliance = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
        }
        
        /** Pose Update */
        Pose2d updatedPose = LimelightHelpers.getBotPose2d_wpiBlue("");
        if (updatedPose != null && counter > 0) {
            counter = 0;
            resetOdometry(updatedPose);
            //SmartDashboard.putNumber("Estimated Pose Angle",updatedPose.getRotation().getDegrees());
        } else {
            //System.out.println(counter);
            counter++;
        }
        SmartDashboard.putNumber("ROBOT POSE X", getPose().getX());
        SmartDashboard.putNumber("ROBOT POSE Y", getPose().getY());
        SmartDashboard.putNumber("Current Pose Angle", getPose().getRotation().getDegrees());
        aprilTagSystem.periodic(getPose());
        //System.out.println(pathActive);
        //System.out.println("Angle: " + bestAngleToApproachNote().getDegrees());

        /** Note Detection Stuff */
        tpuSystem.periodic();
        SmartDashboard.putBoolean("CAN PRESS NOTE BUTTON?", noteAvailable);
        // Be AWARE that xdistanceNote and ydistanceNote MAY BE USED FOR APRIL TAGS
        Translation2d relativeNoteLocation = tpuSystem.getBestNoteLocation();
        vectorCalculatedDistanceNote(relativeNoteLocation);
        SmartDashboard.putNumber("X Distance To Note", relativeNoteLocation.getX());
        SmartDashboard.putNumber("Y Distance To Note", relativeNoteLocation.getY());

        /** For april tag detection */
        //SmartDashboard.putNumber("Rotation", swerveDrive.getPose().getRotation().getDegrees());
        //SmartDashboard.putBoolean("Detected!", hasTargets);
        //SmartDashboard.putNumber("Yaw", yaw);
    }


    /**
     * Calculates the coordinates for the robot to move to an april tag, field relative. This method attempts to take the heading into account.
     * This method updates the coordinates for the robot to move towards when On-The-Fly pathplanner is called. By using the heading angle, theta,
     * it breaks down the relative X and Y paths into absolute/field oriented X and Y componenents. The abs components from each relative component
     * is added together to create a total path that pathplanner uses.
     * 
     * <p> The problem here is that the heading is in range of +-180 degrees, and trig ratios change signs (+/-) in different quadrants. Therefore
     * when you attempt to calculate components based off of just the heading angle, sometimes it doesn't add up properly. For example, when the heading is
     * -135 degrees and the target is abs +x direction, the calculated abs x component is actually opposite to the april tag, so it goes backwards.
     * There is a flaw in this math, and a new method is to replace this one.
     * 
     * <p> By Robin
     * @param measurement the target percieved in 3 dimensions
     * @see #vectorCalculatedDistanceNote(Transform3d) 
     * @deprecated
     */
    public @Deprecated void trigonometricCalculatedDistance(Transform3d measurement) {
        //ydistanceNote = measurement.getX() * Math.sin((getHeading().getRadians())) + measurement.getY() * Math.cos((getHeading().getRadians()));
        //xdistanceNote = - measurement.getY() * Math.sin(Math.abs(getHeading().getRadians())) + measurement.getX() * Math.cos((getHeading().getRadians()));
    }


    /**
     * Calculates the abs x and y distance from the current position to a target position. This method makes use of literal 2d vectors through
     * the Translation2d library. In theory, find a position vector relative to the robot, find it's angle to the forward facing vector, and 
     * adding that to the heading find the abs vector with its direction, and therefore the abs x and y components. Very easy!
     * 
     * <p> By Robin
     * @param measurement the target percieved in 2 dimensions, birds eye view.
     */
    public void vectorCalculatedDistanceNote(Translation2d relativeVector) {
        // What if we analyzed component paths as 2d vectors using translation2d?
        Translation2d absoluteVector = new Translation2d(relativeVector.getNorm(), new Rotation2d(getHeading().getRadians() + relativeVector.getAngle().getRadians()));
        //SmartDashboard.putNumber("Absolute Distance to Note", relativeVector.getNorm());
        notePose = absoluteVector;
        noteAvailable = !(notePose.getX() == 0 && notePose.getY() == 0);
    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.setGyro(new Rotation3d(0, 0, initialHolonomicPose.getRotation().getRadians()));
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Resets odometry pose and heading to 0.
     */
    public void resetOdometry() {
        swerveDrive.setGyro(new Rotation3d(0, 0, 0));
        swerveDrive.resetOdometry(new Pose2d());
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                headingX,
                headingY,
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     *  Set the target heading of the robot.
     *
     * @param angle The angle in degrees.
     */

    public void setTargetHeading(Rotation2d angle){

        swerveDrive.swerveController.lastAngleScalar = angle.getRadians();

    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);

        swerveDrive.swerveController.lastAngleScalar = angle.getRadians();
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput,
                angle.getRadians(),
                getHeading().getRadians(),
                maximumSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Add a vision reading for updating odometry. This method is kind of buggy and makes the robot lose its heading for some reason...?
     */
    public void addVisionReading(Pose2d newPose) {
        swerveDrive.addVisionMeasurement(newPose, Timer.getFPGATimestamp());
    }

    public void setGyro(Rotation2d yaw){
        swerveDrive.setGyro(new Rotation3d(0, 0, yaw.getRadians()));
    }

    /**
     * Creates an on-the-fly path with a set curvature (route) starting from the position of the
     * robot when the path is initiated. The curvature is field relative, always in the same direction
     * and magnitude (think of the path as a free moving vector with a dynamic starting position). 
     * 
     * <p>By Robin.
     * @return a path
     */
    /*
    public PathPlannerPath robinPath() {
        double x = getPose().getX();
        double y = getPose().getY();
        double direction = getHeading().getDegrees();
        
        /*
         * Robin here to explain on-the-fly path. The path is created from several Pose2d objects. Each Pose2d
         * has an X and Y coordinate representing the location of a position WAYPOINT. The rotational aspect of each Pose2d
         * represents the angle of the tangent which the path (bezier curve of the route) enters the waypoint. 
         * For example, if point A is at (0,0) and point B (1,1) and both have a rotational component of 0 degrees, 
         * the robot will travel in an S shaped curve. If point A is 0 degrees but point B is 90 degrees, the path from A to B will look like a quarter
         * of a circle. If point A is 0 degrees, but B is 180 degrees, the path will look like an upside down parabola.
         * The difference with the last Pose2d of the path vs the GoalEndState is that the GES acts as the rotational waypoint. GES
         * does not affect the shape of the path because it is not a position waypoint and therefore has no tangets,
         * but only affects if the robot will spin as it traverses. Of course, GES only exists at the
         * end of the path - to add more rotational waypoints you must use smth else...
         * /
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            getPose(), //The Path starts at the position of the robot currently, but first move towards the direction it was facing before it curves to end point. As such, different diretions will give different curves to end point
            new Pose2d(x + 1, y + 1, Rotation2d.fromDegrees(direction)) //The end point +1 meter in the x direction and +1 meter in the y direction. Enter the end point with THE PATH FACING 0 degrees
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(1.0, 1.0, 2 * Math.PI, 2 * Math.PI), //Global constraints
            new GoalEndState(0.0, Rotation2d.fromDegrees(direction)) //End with in the same direction as when the robot was facing when the path started
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        return path;
    }
    */

    /**
     * Creates an OTF path containing events, rotation targets, and specific constraint zones where required. Used for autonomous actions. If no note
     * is detected, or the wanted position is not available for any reason, the path will only be 0.1m forward.
     * @param mode 0 for note pickup, 1 for shooter sequence.
     * @return a Path
     */
    /*
    public PathPlannerPath sophisticatedOTFPath(int mode, Pose2d destination, Rotation2d startBezier, Rotation2d endBezier) {
        double x = getPose().getX();
        double y = getPose().getY();
        double direction = getHeading().getDegrees();

        IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

        if (mode == 0) {
            /* Note sequence * /
            if (noteAvailable) {
                pathActive = true;
                
                double globalVelocity = 1;
                //Rotation2d endDirection = (bestAngleToApproachNote());
                Rotation2d endDirection = getHeading();
                Pose2d startPose = new Pose2d(getPose().getTranslation(), endDirection);
                //int poseNumOfExtra = 0;
                //System.out.println(Math.abs(notePose.getDistance(getPose().getTranslation())));
                if (Math.abs(notePose.getNorm()) < 0.9-Note.XDISTANCE_OFFSET) {
                    startPose = new Pose2d(x - notePose.getX()/4, y - notePose.getY()/4, endDirection.plus(Rotation2d.fromDegrees(180)));
                    globalVelocity = 0.8;
                } else if (Math.abs(notePose.getNorm()) < 1.3-Note.XDISTANCE_OFFSET) {
                    startPose = new Pose2d(getPose().getTranslation(), endDirection.plus(Rotation2d.fromDegrees(180)));
                    globalVelocity = 0.7;
                }

                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    startPose,
                    new Pose2d(x + notePose.getX(), y + notePose.getY(), endDirection)
                );
                
                EventMarker em = new EventMarker(0, new AutoNotePickUp());
                EventMarker em3 = new EventMarker(1, new AutoIntake(Position.SHOOT));
                EventMarker em4 = new EventMarker(1, new InstantCommand(() -> intakeSubsystem.setSpeed(0)));
                EventMarker signalEnd = new EventMarker(0.95, new InstantCommand(() -> {pathActive = false;}));
                List<EventMarker> lst_em = Arrays.asList(em, em3, em4, signalEnd);

                RotationTarget rt = new RotationTarget(0, endDirection);
                List<RotationTarget> lst_rt = Arrays.asList();
                
                //ConstraintsZone cz = new ConstraintsZone(0.3, 0.6, new PathConstraints(0.05, 0.5, 0.5 * Math.PI, 0.5 * Math.PI));
                List<ConstraintsZone> lst_cz = Arrays.asList();

                PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    lst_rt,
                    lst_cz,
                    lst_em,
                    new PathConstraints(1.5, globalVelocity, 2 * Math.PI, 2 * Math.PI),
                    new GoalEndState(0.0, endDirection),
                    false
                );

                path.preventFlipping = true;
                return path;
            }

        } else if (mode == 1) {
            pathActive = true;
            /* Shooting sequence * /
            if (true) { // Change the argument to whether you are in range for the position using Map
                //getPose().getTranslation().getDistance(destination.getTranslation()) < 4
                List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                    new Pose2d(getPose().getTranslation(), startBezier), 
                    new Pose2d(destination.getTranslation(), endBezier) 
                );

                EventMarker em3 = new EventMarker(0.7, new InstantCommand(() -> {RobotContainer.speakerRoutineActivateShooter = true;})); // THIS COMMAND IS TERMINATED WHEN THE PATH ENDS
                EventMarker em2 = new EventMarker(0, new AutoIntake(Position.SHOOT));
                EventMarker em1 = new EventMarker(0, new InstantCommand(() -> intakeSubsystem.setSpeed(0)));
                EventMarker signalEnd = new EventMarker(0.97, new InstantCommand(() -> {pathActive = false;})); // THIS COMMAND IS TERMINATED WHEN THE PATH ENDS
                List<EventMarker> lst_em = Arrays.asList(em1, em2, em3, signalEnd);
            
                //RotationTarget rt = new RotationTarget(0.5, Rotation2d.fromDegrees(direction + tpuSystem.getBestNoteAngleToApproach()));
                List<RotationTarget> lst_rt = Arrays.asList();
                
                //ConstraintsZone cz = new ConstraintsZone(0.3, 0.6, new PathConstraints(0.05, 0.5, 0.5 * Math.PI, 0.5 * Math.PI));
                List<ConstraintsZone> lst_cz = Arrays.asList();

                PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    lst_rt,
                    lst_cz,
                    lst_em,
                    new PathConstraints(1.5, 1.5, 2 * Math.PI, 2 * Math.PI),
                    new GoalEndState(0.0, destination.getRotation()), // change direction to rotation of pose of april tag
                    false
                );

                path.preventFlipping = true;
                return path;
            }
        }

        /* If no notes detected or no path to go to * /
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            getPose(), //The Path starts at the position of the robot currently, but first move towards the direction it was facing before it curves to end point. As such, different diretions will give different curves to end point
            new Pose2d(x + 0.01, y, Rotation2d.fromDegrees(direction)) //The end point +1 meter in the x direction and +1 meter in the y direction. Enter the end point with THE PATH FACING 0 degrees
        );

        // Create the path using the bezier points created above
        EventMarker em = new EventMarker(0, new AutoIntake(Position.SHOOT));
        EventMarker em2 = new EventMarker(0, new InstantCommand(() -> intakeSubsystem.setSpeed(0))); 
        List<EventMarker> lst_em = Arrays.asList(em, em2);
       
        List<RotationTarget> lst_rt = Arrays.asList();
        List<ConstraintsZone> lst_cz = Arrays.asList();
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            lst_rt,
            lst_cz,
            lst_em,
            new PathConstraints(1.0, 1.0, 2 * Math.PI, 2 * Math.PI), //Global constraints
            new GoalEndState(0.0, Rotation2d.fromDegrees(direction)), //End with in the same direction as when the robot was facing when the path started
            false
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;
        pathActive = false;
        return path;
    }
    */

    /* 
    public PathPlannerPath testpath() {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(getPose().getX(), getPose().getY(), getPose().getRotation()),
            new Pose2d(4.073906, 3.306318, Rotation2d.fromDegrees(240))
        );

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(240)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return path;
    }
    */

    public Rotation2d bestAngleToApproachNote() {
        return Rotation2d.fromDegrees(getHeading().getDegrees() + tpuSystem.getBestNoteAngleToApproach());
    }
}
