
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOReal
import java.util.function.Consumer

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val elevator: Climb
    private val driverController = CommandXboxController(0)

    init {
        val climbIO = ClimbIOReal()
        Climb.initialize(climbIO)
        elevator = Climb.getInstance()
        registerAutoCommands()
        configureDefaultCommands()
        configureButtonBindings()
    }

    private fun configureDefaultCommands() {
        elevator.defaultCommand = elevator.setPower {
            MathUtil.applyDeadband(
                +(driverController.leftTriggerAxis + 1) / 2
                        - (driverController.rightTriggerAxis + 1) / 2,
                0.15
            )
        }
    }

    private fun configureButtonBindings() {
        driverController.y().onTrue(elevator.reset())
        driverController.x().onTrue(elevator.setPosition(10.0))
        driverController.a().onTrue(elevator.setPosition(20.0))
        driverController.b().onTrue(elevator.setPosition(30.0))
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
