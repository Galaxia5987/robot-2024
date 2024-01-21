# Command groups
This file contains [Mermaid](https://mermaid.js.org/) graphs that show the command groups used on the robot. <i>Italicized nodes</i> can be overridden. Information about specific subsystems can be found in their respective directories.

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
## Score
```mermaid
flowchart TD
    score[Score] -->  check_objective{Check Objective}
    check_objective --> set_trap_angle & drive_speaker_amp["<i>Drive to Speaker/Amp pose</i>"]
    drive_speaker_amp --> check_speaker_amp{Check Speaker/Amp}
    check_speaker_amp --> wait_shoot
    subgraph Speaker
        wait_shoot[Wait for shooter] --> feed[Feed]
    end
    
    check_speaker_amp --> Amplify
    subgraph Amplify
        set_amp_height[Set elevator to Amp height]
        set_amp_angle[Set gripper to Amp angle]
        set_amp_height & set_amp_angle --> set_gripper_amp_power[Set gripper Amp power]
    end

    subgraph Trap
        set_trap_angle[Set gripper to Trap angle] -->
        set_gripper_trap_power[Set gripper Trap power]
    end
```
