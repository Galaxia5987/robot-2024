# Command groups
This file contains [Mermaid](https://mermaid.js.org/) graphs that show the command groups used on the robot. Information about specific subsystems can be found in their respective directories.

## Intake
```mermaid
flowchart TD
intake[Intake] --> fold_gripper[Fold Gripper]
    fold_gripper --> set_gripper_intake_power & deploy_intake & power_intake
    subgraph Gripper
        set_gripper_intake_power[Set gripper Intake power]
        stop_gripper[Stop gripper]
    end
    subgraph intake_angle[Intake angle]
        deploy_intake[Deploy]
        fold_intake[Fold]
    end
    subgraph intake_power[Intake power]
        power_intake[Power intake]
        stop_intake[Stop intake]
    end
    set_gripper_intake_power & deploy_intake & power_intake --> wait_for_intake_sensor["<i>Wait for intake sensor</i>"]
    wait_for_intake_sensor --> stop_gripper & fold_intake & stop_intake
```
