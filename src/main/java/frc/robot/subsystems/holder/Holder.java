package frc.robot.subsystems.holder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class Holder extends SubsystemBase {
    private static Holder INSTANCE = null;
    private final TalonSRX spinMotor;

    public Holder(){
        spinMotor = new TalonSRX(Ports.Gripper.ROLLER_ID);
        spinMotor.enableVoltageCompensation(true);
        spinMotor.enableCurrentLimit(true);
        spinMotor.configVoltageCompSaturation(12);
        spinMotor.configPeakCurrentLimit(40);
        spinMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
        spinMotor.setInverted(false);
    }

    public static Holder getInstance(){
        if (INSTANCE==null){
            INSTANCE = new Holder();
        }
        return INSTANCE;
    }

    public Command setPower(double power){
        return Commands.run(()->spinMotor.set(ControlMode.PercentOutput, power));
    }
}
