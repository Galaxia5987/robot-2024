package frc.robot.subsystems.leds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
    private final Debouncer hasTarget = new Debouncer(1.0, Debouncer.DebounceType.kFalling);

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
        switch (RobotContainer.getInstance().getState()) {
            case SHOOT:
                if (ShootingManager.getInstance().getDistanceToSpeaker() < 5
                        && hasTarget.calculate(
                                !Vision.getInstance().getScoreParameters().isEmpty()
                                        && Vision.getInstance().getScoreParameters().stream()
                                                .allMatch((param) -> param.yaw().isPresent()))) {
                    primaryColor = Color.kGreen;
                    secondaryColor = Color.kGreen;
                } else {
                    primaryColor = Color.kRed;
                    secondaryColor = Color.kRed;
                }
                rainbow = false;
                break;
            case AMP:
                primaryColor = Color.kBlue;
                secondaryColor = Color.kBlue;
                rainbow = false;
                break;
            case CLIMB:
                rainbow = true;
                break;
        }

        noteTrigger.update(gripper.hasNote());
        if (noteTrigger.triggered()) {
            noteTimer.reset();
        }
        if (!noteTimer.hasElapsed(2)) {
            blinkTime = 0.1;
            secondaryColor = Color.kWhiteSmoke;
        }

        if (DriverStation.isDisabled()) {
            if (DriverStation.isFMSAttached()) {
                rainbow = true;
            } else if (RobotController.getBatteryVoltage() < 11.8) {
                primaryColor = Color.kRed;
                secondaryColor = Color.kBlack;
                blinkTime = 0.5;
            } else {
                primaryColor = Color.kBlack;
                secondaryColor = Color.kBlack;
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

        SmartDashboard.putString(
                "LED Color", (primary ? primaryColor : secondaryColor).toHexString());
    }
}
