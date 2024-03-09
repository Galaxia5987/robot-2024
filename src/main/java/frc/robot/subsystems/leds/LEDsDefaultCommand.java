package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.math.differential.BooleanTrigger;
import frc.robot.subsystems.gripper.Gripper;

public class LEDsDefaultCommand extends Command {

    private final LEDs leds;
    private final Gripper gripper = Gripper.getInstance();

    private Color primaryColor = Color.kBlue;
    private Color secondaryColor = Color.kBlack;
    private boolean primary = true;
    private boolean rainbow;
    private double blinkTime;
    private final Timer timer = new Timer();
    private final BooleanTrigger noteTrigger = new BooleanTrigger();
    private final Timer noteTimer = new Timer();

    public LEDsDefaultCommand(LEDs leds) {
        this.leds = leds;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();

        noteTimer.start();
        noteTimer.reset();
    }

    @Override
    public void execute() {
        if (noteTimer.hasElapsed(2)) {
            switch (RobotContainer.getInstance().getState()) {
                case SHOOT:
                    primaryColor = Color.kBlue;
                    blinkTime = 0.5;
                    rainbow = false;
                    break;
                case AMP:
                    primaryColor = Color.kGreen;
                    blinkTime = 0.5;
                    rainbow = false;
                    break;
                case CLIMB:
                    rainbow = true;
                    break;
            }

            if (gripper.hasNote()) {
                secondaryColor = Color.kDarkOrange;
            } else {
                secondaryColor = primaryColor;
            }

            noteTrigger.update(gripper.hasNote());
            if (noteTrigger.triggered()) {
                blinkTime = 0.1;
                primaryColor = Color.kDarkOrange;
                secondaryColor = Color.kBlack;

                noteTimer.reset();
            }
        }

        if (rainbow) {
            leds.setRainbow();
        } else {
            if (timer.advanceIfElapsed(blinkTime)) {
                primary = !primary;
            }

            leds.setSolidColor(primary ? primaryColor : secondaryColor);
        }
    }
}
