package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.math.differential.BooleanTrigger;
import frc.robot.subsystems.ShootingManager;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.vision.Vision;

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
                    if (ShootingManager.getInstance().getDistanceToSpeaker() < 5
                            && !Vision.getInstance().getScoreParameters().isEmpty()
                            && Vision.getInstance().getScoreParameters().stream()
                                    .allMatch((param) -> param.yaw().isPresent())) {
                        primaryColor = Color.kGreen;
                        secondaryColor = Color.kGreen;
                        blinkTime = 0.3;
                    } else {
                        primaryColor = Color.kRed;
                        secondaryColor = Color.kRed;
                        blinkTime = 0.5;
                    }
                    rainbow = false;
                    break;
                case AMP:
                    primaryColor = Color.kBlue;
                    secondaryColor = Color.kBlue;
                    blinkTime = 0.5;
                    rainbow = false;
                    break;
                case CLIMB:
                    rainbow = true;
                    break;
            }

            noteTrigger.update(gripper.hasNote());
            if (noteTrigger.triggered()) {
                blinkTime = 0.1;
                secondaryColor = primaryColor;
                primaryColor = Color.kWhiteSmoke;

                noteTimer.reset();
            }
        }

        if (rainbow) {
            leds.setRainbow();
        } else {
            if (timer.advanceIfElapsed(blinkTime)) {
                primary = !primary;
            }

            if (!DriverStation.isDisabled()) {
                leds.setSolidColor(primary ? primaryColor : secondaryColor);
            } else {
                leds.setSolidColor(Color.kBlack);
            }
        }

        SmartDashboard.putString(
                "LED Color", (primary ? primaryColor : secondaryColor).toHexString());
    }
}
