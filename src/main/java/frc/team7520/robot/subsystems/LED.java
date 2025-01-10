package frc.team7520.robot.subsystems;


import java.time.Instant;

import org.ejml.dense.block.decomposition.chol.CholeskyOuterForm_DDRB;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this LED. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static LED INSTANCE = new LED();

    public final static CANdle candle = new CANdle(17);

    public final Animation idleAnimation = new RainbowAnimation(255, 0.75, 100);
    private final Animation intakingAnimation = new ColorFlowAnimation(255, 165, 0, 0, 0.75, 100, Direction.Forward);
    private final Animation noteIn = new ColorFlowAnimation(0, 255, 0, 0, 0.9, 100, Direction.Forward);
    private final Animation noteAvailable = new ColorFlowAnimation(255, 165, 0, 0, 0.95, 100, Direction.Forward);
    private final Animation idle2 = new ColorFlowAnimation(0, 0, 255, 200, 0.8, 100, Direction.Backward);

    /**
     * Returns the Singleton instance of this LED. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LED.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LED getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this LED. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LED() {

    }

    public Command idle() {
        return run(
            () -> {
                candle.animate(idle2);
            }
        );
    }

    public InstantCommand idle2() {
        return new InstantCommand(
            () -> {
                candle.animate(idle2);
            }
        );
    }

    public InstantCommand intaking() {
        return new InstantCommand(
            () -> {
                candle.animate(intakingAnimation);
            }
        );
    }

    public InstantCommand noteIn() {
        return new InstantCommand(
            () -> {
                candle.animate(noteIn);
            }
        );
    }

    public InstantCommand noteAvailable() {
        return new InstantCommand(
            () -> {
                candle.animate(noteAvailable);
            }
        );
    }

    public InstantCommand clear() {
        return new InstantCommand(
            () -> {
                candle.clearAnimation(0);
            }
        );
    }
}

