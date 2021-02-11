# Baltimore Bolts 2021 Robot Code
- 2021 Infinite Recharge robot code base

### Software Architecture
```
Start -> Main.java
             |
         Robot.java
             |
     RobotContainer.java
     |                 |
<Command>.java    <XboxController>
     |
<Subsystem>.java
```

### Drive Mapping
| Controller | Button | Assignment | Notes |
| ---------- | ------ | ---------- | ----- |
| Driver | Joystick | Drive robot | Arcade style drive system, drive type to be made into code option |
| Driver | Right Bumper | Fire powercell |  |
| Driver | Y | Flip robot drive direction | Not yet implemented, only for certain drive types |
| Operator | Joystick | B | Run harvester |
| Operator | Joystick | A | Move indexer updward one rotation |
| Operator | Joystick | Y | Move indexer downward one rotation |
| Operator | DPAD Up | Expand the elevator |  |
| Operator | DPAD Down | Contract the elevator |  |
| Operator | Right Bumper | Fire powercell |  |

### Developer Notes
- subsystems are single threaded
    - this essentially means you can only run one command on a particular subsystem at a time
    - if you want to run multiple commands on a subsystem, it must be broken into multiple subsystems 
        - front and back harvester for instance
