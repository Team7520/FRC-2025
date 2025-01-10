/* ------------------------------------------------------------------------------------------------------------------------------------
 * TpuSystem
 * Robin Yan
 * 08/02/2024
 * A class used to represent a list of detections from the NetworkTables published by the Raspberry Pi + TPU.
 * The TpuSystem reads a StringTopic and divides it into individual note information (also as a string).
 * A Note object is created when a new detection is found, and vice versa for removal. Note objects are continously updated if detected.
 * Using the compareScore method of the Note class, TpuSystem determines the most optimal note and its location.
 * ------------------------------------------------------------------------------------------------------------------------------------ */

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

public class TpuSystem {
  // the subscriber is an instance variable so its lifetime matches that of the class
  private final StringSubscriber NOTE_INFO;
  private final int MAXIMUM_CAPACITY = 10;
  private int numNotes = 0;
  private Note[] notes = new Note[MAXIMUM_CAPACITY];
  private Note bestNote = null;

  /**
   * Makes a new TpuSystem. A TpuSystem can be understood as a list of notes/detections, as determined by information foudn in a the String passed through NT
   * @param detection the StringTopic that gives a details of ONE note. Does not currently support multiple note detection.
   */
  public TpuSystem(StringTopic detection) {
    NOTE_INFO = detection.subscribe("[]"); //[{'x':None,'y':None,'w':None,'h':None,'conf':0}]
  }

  /**
   * Periodically checks if notes are detected
   */
  public void periodic() {
    // simple get of most recent value; if no value has been published,
    // returns the default value passed to the subscribe() function
    String val = NOTE_INFO.get();
    translateInfo(val);
    if (!isEmpty()) {
      determineBestNote();
    } else {
      bestNote = null;
    }
    printAllScore();
  }

  /**
   * Closes the subscriber, though not always necessary needed
   */
  public void close() {
    // stop subscribing
    NOTE_INFO.close();
  }

  /**
   * Reads a big string value, breaking it down into individual notes.
   * @param value the String value from a topic
   */
  private void translateInfo(String value) {
    try{
      /* Remove all notes in list if no detections */
      if (value.equals("\"[]\"")) {
        while (removeLastNote());
        return;           
      }

      /* If detections, seperate individual info of notes and update/make notes */
      int[][] indexes = new int[MAXIMUM_CAPACITY][2]; // Each row is a note, a row containing a start and end index for a string
      int row = -1;
      for (int i = 0; i < value.length(); i++) {
        char c = value.charAt(i);
        if (c == '{') {
          row++;
          indexes[row][0] = i;
        } else if (c == '}') {
          indexes[row][1] = i;     
        }
      }

      /* if list has more notes that actually detected this current cycle... */
      if (numNotes > row + 1) {
        removeLastNote();
      }

      /* update or make a new note for each information set */
      for (int i = 0; i < row+1; i++) {
        String subValue = value.substring(indexes[i][0], indexes[i][1]+1);
        if (notes[i] == null) {
          boolean added = addNote(i, subValue);
          if (!added) {
            System.out.println("TPU SYSTEM: THE NOTE LIST IS FULL!");
          }
        } else {
          notes[i].periodic(subValue);
        }
      }

    } catch (NumberFormatException e) {
      System.out.println("TPU SYSTEM: NUMBER FORMAT EXCEPTION");
    } catch (IndexOutOfBoundsException e) {
      System.out.println("TPU SYSTEM: INDEX OUT OF BOUNDS");
    }
  }

  /**
   * Sorts notes[] so that all null objects (AKA non existant) are at the back of list.
   * Not currently required since insertion/removal of objects always occurs at end of list
   */
  private void sortNull() {
    for (int i = 0; i < numNotes; i++) {
      if (notes[i] == null) {
        notes[i] = notes[numNotes-1];
        notes[numNotes-1] = null;
      }
    }
  }

  /**
   * Adds a note to the bottom of the list.
   * @return <code>false</code> if the list capacity is full
   */
  private boolean addNote(int index, String segregatedInfo) {
    if (numNotes == MAXIMUM_CAPACITY) {
      return false;
    }
    notes[index] = new Note(segregatedInfo);
    numNotes++;
    return true;
  }

  /**
   * Removes the note at the bottom of the list when that note is no longer detected. If the note that disappeared was not
   * at the bottom of the list, udpate another note with required information as substitute.
   * @return a boolean indicating whether the removal was successful
   */
  private boolean removeLastNote() {
    if (isEmpty()) {
      return false;
    }
    notes[numNotes-1] = null;
    numNotes--;
    return true;
  }

  /**
   * Determines the best note according to which note has the best score
   */
  private void determineBestNote() {
    if (isEmpty()) {
      bestNote = null;
      return;
    }
    bestNote = notes[0];
    for (int i = 0; i < numNotes; i++) {
      bestNote = bestNote.compareNoteScore(notes[i]);
    }
    if (bestNote.getScore() < 1) {
      bestNote = null;
    }
  }

  /**
   * Checks if the list has any notes or not
   * @return if the list is empty
   */
  private boolean isEmpty() {
    return numNotes == 0;
  }
  
  /**
   * @return the Translation2d location of the best note
   */
  public Translation2d getBestNoteLocation() {
    if (bestNote == null) {
      return new Translation2d(0,0);
    }
    return bestNote.getLocation();
  }

  public double getBestNoteAngleToApproach() {
    if (bestNote == null) {
      return 0;
    }
    return bestNote.getAngleToApporach();
  }

  /**
   * Prints all scores of all currently detected notes. Does not identify which score is for which note
   */
  private void printAllScore() {
    String s = "";
    for (int i = 0; i < numNotes; i++) {
      s += notes[i].getScore() + ", ";       
    }
    System.out.println(s);
  }
}