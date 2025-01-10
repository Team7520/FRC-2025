// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team7520.robot.util;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

//import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;

import com.fasterxml.jackson.databind.util.JSONPObject;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team7520.robot.Constants;

//import frc.team7520.robot.Constants;
import java.io.IOException;

/** Add your docs here. */
public class AprilTagSystem {

    private PhotonCamera camera;
    private final PipeLineType TYPE = PipeLineType.APRIL_TAG;
    private boolean isOpen = false;
    private boolean facingTarget = false;
    private String cameraName;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private List<AprilTag> apriltags;
    private boolean aprilTagLayoutLoaded = false;
    private final double MAX_RANGE = 2; //In meters, anything beyond 2 meters should not be used
    
    private Pose2d robotPose;
    private AprilTag closestTag;

    public enum PipeLineType {
        APRIL_TAG,
        COLORED_SHAPE,
        REFLECTIVE
    }

    public class PhotonVisionData {
        public boolean is_vaild;
        // 2D Mode
        public double yaw;
        public double pitch;
        public double area;
        public double skew;
        public double x_dist_2d;
        public double y_dist_2d;
        // 3D Mode
        public int april_tag_id;
        public double x_distance;
        public double y_distance;
        public double z_distance;
        public double x_rotate;
        public double y_rotate;
        public double z_rotate;
        public double angle_rotate;
        public double ambiguity;
    }

    public AprilTagSystem(String cameraName) {
        this.cameraName = cameraName;        
        camera = new PhotonCamera(cameraName);
        if(camera.isConnected()) {
            isOpen = true;
        } else {
            System.out.printf("Failed to open camera: %s \n", cameraName);
        }
        periodic(robotPose);
    }

    public void periodic(Pose2d robotPose) {
        this.robotPose = robotPose;
        isOpen = camera.isConnected();
        SmartDashboard.putBoolean("PHOTONVISION CONNECTED?", isOpen);

    }

    /** 
     * Loads field layout of april tags. The aprilTagFieldLayout field is initiated here because the layout takes a while to load - often crashes
     * if called too soon (such as within class constructor). 
     * @return a boolean indicating whether the layout was loaded
     */
    public boolean initiateAprilTagLayout() {
        for (int i = 0; i < 5 && !aprilTagLayoutLoaded; i++) {
            try {            
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                apriltags = aprilTagFieldLayout.getTags();
                aprilTagLayoutLoaded = true;
            } catch (IOException e) {
                e.printStackTrace();
                System.out.println("Load April Tag Layout Error");
            }
        }
        return aprilTagLayoutLoaded;
    }

    /**
     * Estimates the current robot position based on the april tag it sees. April tags farther than {@link #MAX_RANGE}
     * are not considered.
     * @return a Pose2d
     */
    public Pose2d getCurrentRobotFieldPose() {
        var result = camera.getLatestResult();
        /* 
         * To avoid confusion regarding whether rotation is applied first or translation, and whether the rotation/translational
         * axes are transformed along with the object, we'll assume translation is considered first before rotation, where each
         * component of the transformation is considered in the order as it is labeled as a parameter.
         * 
         * According to WPILIB Documentation, the related object/class, Transform2d, consists of a translation and a rotation.
         * In Transform2d, the rotation is applied TO THE TRANSLATION, then the rotation is applied to the object.
         * This is mathematically equivalent to applying an unrotated translation, then applying the rotation to the object.
         * Both seqences transform the object to the same destination in space.
         * 
         * For simplicity reasons, we should base the camera's transformation relative to the robot center, and then simply inverse
         * the transformtation - instead of having robot relative to camera.
         * 
         * -Robin
         */
        double CAMERA_POS_FOR_ROBOT_X = -0.26; // Meters
        double CAMERA_POS_FOR_ROBOT_Y = -0.06;
        double CAMERA_POS_FOR_ROBOT_Z = 0.6858;
        double CAMERA_POS_FOR_ROBOT_ROLL = 0;
        double CAMERA_POS_FOR_ROBOT_PITCH = -Math.toRadians(20); // Radians
        double CAMERA_POS_FOR_ROBOT_YAW = Math.PI;

        Transform3d robotToCamera = new Transform3d(CAMERA_POS_FOR_ROBOT_X, 
                                                    CAMERA_POS_FOR_ROBOT_Y, 
                                                    CAMERA_POS_FOR_ROBOT_Z, 
                                                    new Rotation3d(CAMERA_POS_FOR_ROBOT_ROLL,
                                                    CAMERA_POS_FOR_ROBOT_PITCH,
                                                    CAMERA_POS_FOR_ROBOT_YAW));
        
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            if (target.getBestCameraToTarget().getX() > MAX_RANGE) {
                return null;
            }
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(), 
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), 
                robotToCamera.inverse());
            
            //SmartDashboard.putNumber("Tag X", target.getBestCameraToTarget().getX());
            //SmartDashboard.putNumber("Tag Y", target.getBestCameraToTarget().getY());
            return robotPose.toPose2d();            
        } 
        return null;
    }

    /**
     * Checks which aprilTag is the closest to the robot. Loops through all 16
     */
    private void closestTag() {
        closestTag = apriltags.get(0);
        for (int i = 0; i < apriltags.size(); i++) {
            closestTag = compareTag(closestTag, apriltags.get(i));
        } 
        
    }

    /**
     * Calculates the distance between the given aprilTag and robot pose
     * @param aprilTag
     * @return
     */
    private double checkDistance(AprilTag aprilTag) {
        return aprilTag.pose.toPose2d().getTranslation().getDistance(robotPose.getTranslation());
    }

    /**
     * Compares which of the two given aprilTags have a smaller distance the robot
     * @param aprilTag1 an AprilTag
     * @param aprilTag2 another AprilTag
     * @return a Pose2d representing an AprilTag
     */
    private AprilTag compareTag(AprilTag aprilTag1, AprilTag  aprilTag2) {
        if (checkDistance(aprilTag1) <= checkDistance(aprilTag2)) {
            return aprilTag1;
        }
        return aprilTag2;
    }

    /**
     * @return a PhotonVisionData object containing information about a detected april tag
     */
    public PhotonVisionData getPVTargetData() {
        PhotonVisionData target_data = new PhotonVisionData();
        target_data.is_vaild = false;
        if(!isOpen) {
            System.out.printf("Check PhotonVison Camera %s, which is NOT open\n", cameraName);
            camera = new PhotonCamera(cameraName);
            if(camera.isConnected()) {
                isOpen = true;
                System.out.printf("PhotonVison Camera %s, opened now\n", cameraName);
            } else {    
                return target_data;
            }
        }

        var result = camera.getLatestResult();
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            target_data.yaw = target.getYaw(); // The yaw of the target in degrees (positive right).
            target_data.pitch = target.getPitch(); // The pitch of the target in degrees (positive up).
            target_data.area = target.getArea(); // The area (how much of the camera feed the bounding box takes up) as a percent (0-100).   
            target_data.april_tag_id = target.getFiducialId(); // The ID of the detected fiducial marker.
            target_data.ambiguity = target.getPoseAmbiguity(); // How ambiguous the pose of the target is must less than 0.2 

            /* Get the transform that maps camera space 
                (X = forward, Y = left, Z = up) to object/fiducial tag space 
                (X forward, Y left, Z up) with the lowest reprojection error.
            */
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            target_data.x_distance = bestCameraToTarget.getX();
            target_data.y_distance = bestCameraToTarget.getY();
            target_data.z_distance = bestCameraToTarget.getZ();
            Rotation3d trans_3d_rotate = bestCameraToTarget.getRotation();
            target_data.x_rotate = trans_3d_rotate.getX();
            target_data.y_rotate = trans_3d_rotate.getY();
            target_data.z_rotate = trans_3d_rotate.getZ();
            target_data.angle_rotate = trans_3d_rotate.getAngle();

            target_data.is_vaild = true;
            System.out.printf("AprilTag Raw Data: yaw:%f, pitch:%f, area:%f, 3D: X:%f, Y:%f, Z:%f, RX:%f, RY:%f, RZ:%f, RW:%f, ID:%d, ambiguity:%f\n",
                target_data.yaw, 
                target_data.pitch, 
                target_data.area, 
                target_data.x_distance,
                target_data.y_distance,
                target_data.z_distance,
                target_data.x_rotate,
                target_data.y_rotate,
                target_data.z_rotate,
                target_data.angle_rotate,
                target_data.april_tag_id, 
                target_data.ambiguity);
        } 
        SmartDashboard.putBoolean("APRIL TAG FOUND", result.hasTargets());

        return target_data;
    }
}
