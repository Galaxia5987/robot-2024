package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase{
    private HoodInputsAutoLogged inputs = HoodIO.inputs;
    private static Hood INSTANCE = null;
    private final HoodIO io;

    private Hood(HoodIO hoodIO) {
        this.io = hoodIO;
    }

    public static Hood getInstance(HoodIO io) {
        if (INSTANCE == null) {
            INSTANCE = new Hood(io);
        }
        return INSTANCE;
    }

    public Command setAngle (double angle){
        return Commands.run(()->{
            io.setAngle(angle);
            inputs.angleSetPoint = angle;
        }).withName("set hood angle");
    }
    public double getAngle(){
       return inputs.angle;
    }


    @Override
    public void periodic() {
        io.updateInputs();
    }
}
