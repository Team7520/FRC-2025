/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Robin Yan
 * 08/07/2024
 * A class that represents a note. It contains the basic a information of the note translated from NetworkTables.
 * Using the provided information and some math, it calculates an estimated distance to the note in terms of X and Y (following FRC coordinates)
 * Each note is equipped with a "score" indicating whether or not the note in question is the best option to go for when multiple notes are detected.
 * A score of note is dependent on their area (major effect) and confidence level (minor effect).
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

package frc.team7520.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import frc.team7520.robot.Constants;

import java.io.File;
import java.util.List;

public class Note {
    /*
     * Terminology being used here:
     * Bottom - very bottom of the screen/stream being processed. Angle is 47.2° from horizontal
     * Top - very top of the stream whose angle is near 0° to the horizontal, parallel to the ground
     * Center - refered to both in terms right to left and top to bottom. In terms of top to bottom, center is located 23.2° from horizontal
     */
    final static private double SCREEN_WIDTH  = 640; // A stream is constructed were (0,0) is the top left corner, and (640,480) is the bottom right
    final static private double SCREEN_HEIGHT = 480;
    final static private double X_DPP = 58 / SCREEN_WIDTH; // Degrees per pixel. Measured by 29° from center of screen to side
    final static private double Y_DPP = 0.1; // Degrees per pixel. Measured as bottom being 47.2° from the top horizontal. Top angle is therefore 0.8° above horiztonal
    final static private double CAM_HEIGHT = 0.6858; // In meters off the floor, AKA 27 inches. We assume the camera is located directly above robot center
    final static private double DISTANCE_AT_BOTTOM = 0.508; // In meters forward of center of robot, AKA 20 inches
    final static private double X_CENTER = SCREEN_WIDTH/2; // 320 pixel from the left side
    final static private double YDISTANCE_OFFSET = 0.15; // In meters to the left. What is actually calculated as relativeYDistance is 0.15m off physical measurements
    final static public double XDISTANCE_OFFSET = 0.5; // To prevent robot from running over note's location, stopping it in front of note in position for pick up
    final static private double MAXIMUM_ACCEPTED_RANGE = 3; // In meters (3)
    final static private double CONFIDENCE_WEIGHT_FACTOR = 10;
    final static private double AREA_WEIGHT_FACTOR = 0.001;
    final static private double RANGE_WEIGHT_FACTOR = 0; // Add points if inside accepted range

    /* Information from NT */
    private double xPos;
    private double yPos;
    private double width;
    private double height;
    private double confidence;

    /* Estimated details */
    private double relativeXDistance = 0; // Position of a note estimated from data receieved from NetworkTables. Follows FRC coordinates system
    private double relativeYDistance = 0; // In meters
    private Translation2d noteLocation;
    private double angleToApproach = 0; // CHange in degrees for which way the robot heading should be when approaching
    private double score = 0;

    /**
     * Create a new Note object
     * @param stringInfo the String containing a note's coordinates, dimensions, and confidence level. Must be in the form: "[{'x':0,'y':0,'h':0,'w':0,'conf':0}]"
     */
    public Note(String stringInfo) {
        periodic(stringInfo);
    }

    /**
     * Updates the note object with up-to-date information
     * @param stringInfo
     */
    public void periodic(String stringInfo) {
        if (translateInfo(stringInfo)) {
            noteLocation = trigonemtricCalculatedDistance();
            updateScore();
        }
        
        
    }

    /**
     * Reads the value of a string containg information of a note and breaking it into isolated values. This method assumes that the given string has no errors and
     * comes in the following format: "[{'x':0,'y':0,'h':0,'w':0,'conf':0}]"
     * @param value the String information
     * @return <code>true</code> if info update was successful
     */
    private boolean translateInfo(String value) {
        try {
            boolean firstInd = false;
            int[] indexes = new int[10]; 
            int count = 0;

            for (int i = 0; i < value.length(); i++) {
                char c = value.charAt(i);
                if (c >= '0' && c <= '9' || c == '.') {
                    if (!firstInd) {
                        firstInd = true;
                        indexes[count] = i;
                        count++;
                    }
                } else {
                    if (firstInd) {
                        firstInd = false;
                        indexes[count] = i;
                        count++;
                    }
                }
            }

            xPos = Double.parseDouble(value.substring(indexes[0], indexes[1]));
            yPos = Double.parseDouble(value.substring(indexes[2], indexes[3]));
            height = Double.parseDouble(value.substring(indexes[4], indexes[5]));
            width = Double.parseDouble(value.substring(indexes[6], indexes[7]));
            confidence = Double.parseDouble(value.substring(indexes[8], indexes[9]));
            return true;

        } catch (NumberFormatException e) {
            System.out.println("NOTE: NUMBER FORMAT EXCEPTION");
            return false;
        } catch (IndexOutOfBoundsException e) {
            System.out.println("NOTE: INDEX OUT OF BOUNDS");
            return false;
        }
    }

    /**
     * Uses physical and retrieved measurements and trigonometry to estimate the location of a note relative to the robot center.
     * View the team drive for details about the math: https://drive.google.com/drive/folders/1zDrigwM__K0zThnreSJkRGCgeL3lPyoZ
     * @return a Translation2d of the note detected
     */
    private Translation2d trigonemtricCalculatedDistance() {
        angleToApproach = (X_CENTER-xPos)*(X_DPP);
        relativeXDistance = CAM_HEIGHT/(Math.tan(Math.toRadians(yPos*Y_DPP)));
        relativeYDistance = Math.hypot(relativeXDistance, CAM_HEIGHT)*Math.tan(Math.toRadians(angleToApproach));
        return new Translation2d(relativeXDistance - XDISTANCE_OFFSET, relativeYDistance + YDISTANCE_OFFSET); //YDISTANCE_OFFSET
    }

    /**
     * Updates the score of the note. The score, in terms of how it is used here, is to determine whether this note is the most optimal note to go to, as compared to others.
     */
    private void updateScore() {
        score = confidence*CONFIDENCE_WEIGHT_FACTOR + width*height*AREA_WEIGHT_FACTOR;
        if (relativeXDistance > MAXIMUM_ACCEPTED_RANGE) {
            score *= RANGE_WEIGHT_FACTOR;
        }
    }

    /**
     * Compares the score of two notes.
     * @param other the other Note
     * @return the note with the larger score
     */
    public Note compareNoteScore(Note other) {
        if (other == null) {
            return this;
        }
        if (this.score >= other.score) {
            return this;
        } else {
            return other;
        }
    }

    public Translation2d getLocation() {
        if (noteLocation == null) {
        return new Translation2d(0,0);
        }
        return noteLocation;
    }

    public double getArea() {
        return height*width;
    }

    public double getConfidence() {
        return confidence;
    }

    public String toString() {
        return "Note Confidence: " + confidence;
    }

    public double getScore() {
        return score;
    }

    public double getAngleToApporach() {
        return angleToApproach*1.2;
    }
    
}
