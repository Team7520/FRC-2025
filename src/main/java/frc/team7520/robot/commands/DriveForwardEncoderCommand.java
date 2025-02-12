package frc.team7520.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team7520.robot.subsystems.swerve.SwerveSubsystem;
import frc.team7520.robot.util.AprilTagSystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.SwerveModule;

public class DriveForwardEncoderCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final double targetDistance; // in meters
    private double initialEncoderDistance;
    private boolean done = false;
    public AprilTagSystem aprilTagSystem = new AprilTagSystem("PhotonCamera");
    private double startYpose;
    private Pose2d lastSeen;

    public DriveForwardEncoderCommand(SwerveSubsystem swerveSubsystem, double targetDistance) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetDistance = targetDistance;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        done = false;
        // Get the initial encoder distance (average of all module positions)
        initialEncoderDistance = getAverageEncoderDistance();
        SmartDashboard.putNumber("ERORR", -1);
        SmartDashboard.putNumber("RobotX_POSE", -1);
        SmartDashboard.putNumber("RobotY_POSE", -1);

        Pose2d updatedPose = aprilTagSystem.getCurrentRobotFieldPose();
            if (updatedPose != null){
                SmartDashboard.putNumber("Start_X_Pose", updatedPose.getX());
                SmartDashboard.putNumber("Start_Y_Pose", updatedPose.getY());
                startYpose = updatedPose.getY();
            } else {
                SmartDashboard.putNumber("Start_X_Pose", -1);
                SmartDashboard.putNumber("Start_Y_Pose", -1);
            }
    }

    @Override
    public void execute() {
        // Drive the robot forward at a constant speed
        swerveSubsystem.drive(new Translation2d(0, 3), 0, false); // 0.5 m/s forward, no rotation
    }

    @Override
    public boolean isFinished() {
        // Check if the robot has traveled the target distance
        double currentDistance = Math.abs(getAverageEncoderDistance() - initialEncoderDistance);
        SmartDashboard.putNumber("currentDistance",currentDistance);
        SmartDashboard.putNumber("targetDistance", targetDistance);
        Pose2d updatedPose = aprilTagSystem.getCurrentRobotFieldPose();
        if (updatedPose!=null){
            lastSeen = updatedPose;
        }
        if((currentDistance > (targetDistance/2)) && (done == false)){
            if (updatedPose != null){
                SmartDashboard.putNumber("RobotX_POSE", updatedPose.getX());
                SmartDashboard.putNumber("RobotY_POSE", updatedPose.getY());
                
            } else {
                SmartDashboard.putNumber("RobotX_POSE", -1);
                SmartDashboard.putNumber("RobotY_POSE", -1);
            
            }
            SmartDashboard.putNumber("RealRobotPose", currentDistance);
            SmartDashboard.putNumber("ERORR", currentDistance-(lastSeen.getY()-startYpose));
            
            done = true;
        }
        return currentDistance >= targetDistance; 
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
        
    }

    /**
     * Calculate the average distance traveled by all swerve modules.
     * This assumes each module's encoder tracks distance in meters.
     */
    private double getAverageEncoderDistance() {
        SwerveModule[] modules = swerveSubsystem.getSwerveDrive().getModules();
    double totalDistance = 0;

    // Constants for conversion
    double wheelDiameter = 0.1016; // Replace with your wheel diameter in meters
    double gearRatio = 6.7; // Replace with your gear ratio (e.g., 6.75:1)
    double wheelCircumference = Math.PI * wheelDiameter; // Circumference in meters

        // Get the drive motor's encoder position in degrees
        double encoderDegrees = modules[0].getDriveMotor().getPosition();
        SmartDashboard.putNumber("encoderDegrees",encoderDegrees);


        // Convert degrees to meters
        // double distanceMeters = (encoderDegrees / 360.0) * (wheelCircumference / gearRatio);
        totalDistance += encoderDegrees;

    return totalDistance;// / modules.length;
    }
}