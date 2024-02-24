package frc.robot.subsystems.leds;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.IntSupplier;

public class LEDs extends SubsystemBase {
    private final AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    private Color primary = Color.kBlack;
    private Color secondary = Color.kBlack;
    private Color currentColor = primary;
    private Color fadeColor = primary;

    private double fadeTime = 1;
    private final Timer timer = new Timer();

    public LEDs(int port, int length) {
        ledStrip = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        ledStrip.setLength(length);
        ledStrip.setData(ledBuffer);
        ledStrip.start();

        timer.start();
        timer.reset();
    }

    private void setSolidColor(Color color, int start, int end) {
        for (int i = start - 1; i < end; i++) {
            ledBuffer.setLED(i, color);
        }
        ledStrip.setData(ledBuffer);
    }

    /**
     * Updates fadeColor to the correct color for a fade effect at the current time by interpolating
     * the two given colors.
     *
     * @param initial Initial color.
     * @param goal Final Color.
     */
    private void updateFade(Color initial, Color goal) {
        Translation3d initialPoint = new Translation3d(initial.red, initial.green, initial.blue);
        Translation3d goalPoint = new Translation3d(goal.red, goal.green, goal.blue);

        double t = timer.get() / fadeTime;

        Translation3d solution = initialPoint.interpolate(goalPoint, t);
        fadeColor = new Color((int) solution.getX(), (int) solution.getY(), (int) solution.getZ());
    }

    private void setRainbow(int start, int end) {
        int rainbowHue = 0;
        for (int i = start - 1; i < end; i++) {
            ledBuffer.setHSV(i, rainbowHue, 255, 180);
            rainbowHue += (180 / (end - start + 1));
            ledStrip.setData(ledBuffer);
            rainbowHue %= 180;
        }
    }

    public Color getPrimary() {
        return primary;
    }

    /**
     * Sets the primary color of the LEDs that will be used in the solid and percentage modes and as
     * the first color in the blink and fade modes.
     *
     * @param primary Color to set the primary.
     */
    public void setPrimary(Color primary) {
        this.primary = primary;
    }

    public Color getSecondary() {
        return secondary;
    }

    /**
     * Sets the secondary color of the LEDs that will be used as the second color in the blink and
     * fade modes.
     *
     * @param secondary Color to set the secondary.
     */
    public void setSecondary(Color secondary) {
        this.secondary = secondary;
    }

    public double getFadeTime() {
        return fadeTime;
    }

    /**
     * Sets the duration for the fade effect.
     *
     * @param duration Duration of the fade effect. [sec]
     */
    public void setFadeTime(double duration) {
        this.fadeTime = duration;
    }

    /**
     * Sets a solid color that won't be saved as the primary color.
     *
     * @param optionalColor A color to set the LEDs to.
     * @param start The starting led number of the segment.
     * @param end The ending led number of the segment.
     * @return A command that sets the LEDs to a solid color.
     */
    public Command solid(Color optionalColor, int start, int end) {
        return this.run(() -> setSolidColor(optionalColor, start, end));
    }

    public Command solid(int start, int end) {
        return solid(primary, start, end);
    }

    /**
     * Sets a segment to a percentage of the segment length.
     *
     * @param percentage The percentage of the segment length to light up.
     * @param start The starting led number of the segment.
     * @param end The ending led number of the segment.
     * @return A command that sets a segment to a percentage of the segment length.
     */
    public Command percentage(IntSupplier percentage, int start, int end) {
        return this.run(
                () -> {
                    setSolidColor(primary, start, (end - start + 1) * percentage.getAsInt() / 100);
                });
    }

    /**
     * Sets a segment to blink between the primary and secondary colors.
     *
     * @param blinkTime The time it takes to change the color. [sec]
     * @param start The starting led number of the segment.
     * @param end The ending led number of the segment.
     * @return A command that sets a segment to blink between the primary and secondary colors.
     */
    public Command blink(double blinkTime, int start, int end) {
        return this.run(
                () -> {
                    currentColor = currentColor == primary ? secondary : primary;

                    if (timer.advanceIfElapsed(blinkTime)
                            && blinkTime > LedConstants.MINIMAL_BLINK_TIME) {
                        setSolidColor(currentColor, start, end);
                    }
                });
    }

    /**
     * Sets a segment to fade between the primary and secondary colors.
     *
     * @param fadeTime The duration of the fade effect. [sec]
     * @param start The starting led number of the segment.
     * @param end The ending led number of the segment.
     * @return A command that sets a segment to fade between the primary and secondary colors.
     */
    public Command fade(double fadeTime, int start, int end) {
        return this.run(
                () -> {
                    setFadeTime(fadeTime);
                    updateFade(primary, secondary);
                    setSolidColor(fadeColor, start, end);
                });
    }

    public Command fade(int start, int end) {
        return fade(fadeTime, start, end);
    }

    /**
     * Sets a segment of the strip to a rainbow pattern.
     *
     * @param start The starting led number of the segment.
     * @param end The ending led number of the segment.
     * @return A command that sets a segment to a rainbow pattern.
     */
    public Command rainbow(int start, int end) {
        return this.run(() -> setRainbow(start, end));
    }
}
