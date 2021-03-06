# Baltimore Bolts 2021 Robot Code
- 2021 Infinite Recharge robot code base
- Robot Name: "Lazar Beam"
- Code Name: "Beans"

### Software Architecture
```
Start -> Main.java
             |
         Robot.java
             |
     RobotContainer.java -> <Controllers>
                                 |
                            <Command>.java
                                 |
                            <Subsystem>.java
```

### Driver Button Mapping
| Button | Assignment | Notes |
| ------ | ---------- | ----- |
| Joystick | Drive robot | Drive type can be changed in `RobotContainer.java` with enum selection|
| Y | Flip robot drive direction | Not yet implemented, only for certain drive types |

### Operator Button Mapping
| Button | Assignment | Notes |
| ------ | ---------- | ----- |
| A | Intake | Runs both harvesting and indexing subsystems to create a one button solution for intaking powercells |
| B | Indexer shooting | Shift all powercells to the top and then shift one up to shoot if shooter is running |
| X | Purge | Remove balls from robot in case they are stuck |
| Y |  |  |
| DPAD Up | Move indexer up one rotation |
| DPAD Down | Move indexer down one rotation |
| DPAD Left | Harvester purge |
| DPAD Right | Harvester intake |
| Left Trigger | Run shooter in reverese | Should never need this |
| Right Trigger | Run shooter | Hold this when desiring to shoot |
| Left Bumper | Contract the elevator |  |
| Right Bumper | Expand the elevator |  |

> In an ideal scenario, the only buttons needed for the operator are `A`, `B`, and `Right Trigger`, the rest of the operator button mappings are for manual control if things aren't working

### Developer Notes
- subsystems are single threaded
    - this essentially means you can only run one command on a particular subsystem at a time
    - if you want to run multiple commands on a subsystem, it must be broken into multiple subsystems
        - front and back harvester for instance

### Easter Egg
```
Nkc:;,,;:ldONMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
o,;:;:::::,,lx0XNWMMMMMWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
,;:;;:::::;;;;;:cloodxk0XWMWMMMMMMMMMMMMMMMMMMMWXOxoooxOXWMM
.;:;;;::::;;:;;;,,,;,,,,;cxXWWWWMMMMMMMMMMMWMNkc,.......,l0W
',:;::;;:::::::::::::::;;'.;OWWMMMMMMMMMMWWXd:',;;;;;;,'..'k
l',::;;::::::::::::::::::;'.:KWWWMMMMMMMMXd;';:::::;;:;,''.'
No'';;;:::::::::::::::::;,'.,0MWWMMMWWN0d:.':::::;::;::;,'..
MNO:'',;;;::::::::;;;::;,'..dNWWNKOkdol;,,,;::;:::;;::;;,''.
WWMNkl,'',,;;;;;;:;;;;,,..,xX0xdc:;,;;;;;;:::;;:::;;::;;,'..
MMMWMWXko:'...''''''...,cxK0l;,,;:::::;::::;;:::::::::;;'..:
MMMMMMMMMNKkdolccccloxMMMMO:;:;;;;:::;;;;:::::::;;;:::;'..,0
MMMMMMMMMMWXOxddxk0NMMWMMNo,:;:;;:::::;;;:::::::::::;,'..;0M
MMMMMMMMNkl;,,,,,,;coMMMMWd,;:::::;;:::;;:::;;:::;;,'..'oXWW
MWMMMMW0c,;:;:c::;::;;lMMMKc,::::::::::::::::::;;,'..'lKWMMM
MMMMMWO,';;:c::::c::::,,dXWKd:;,;:::::::::::;;;,..,cdKWMMMMM
MMMMM0;.;;;:c::::::::::;,lKWWKOdc;'',,,,,'''',:ldOXWMMMMMMMM
MMMMMd.';:;;:;;::::::::,;:cxNMWMWXOxdooooodxOKNWMMMMMMMMMMMM
MMMMMo.';:;;:;;::::::::;,;;;cdOKWMMMMMMWMMMMMMMMMMMMMMMMMMMM
MMMMMx.';::;:;;:::::::::;,,,'..':okXWMMWMMMMMMMMMMMMMMMMMMMM
MMMMM0,.,;;;:;::::::::::::;;;,,,;'';cdKWMMMMMMMMMMMMMMMMMMMM
MMMMMWo.',;:::;;::::::::::::::::::::,.'cOWMMMMMMMMMMMMMMMMMM
MMMMMMXc.',;;;:::::::::::::::::::::;:;,..lXMMMMMMMMMMMMMMMMM
MMMMMMMK:..';:::::::::::::::::::;:::::;,..:KMMMMMMMMMMMMMMMM
MMMMMMMMXl..',;:;;::::;;:::;;:::;;::::;,'..oMMWMMMMMMMMMMMMM
MMMMMMMMWNk;..',;::;;;;:::::;;;::::;::;,'..lWMMMMMMMMMMMMMMM
MMMMMMMMMMMXd,..',,;;::::::::;;;;:;:;;,''..kWMMMMMMMMMMMMMMM
MMMMMMMMMMMMWKd,...'',,;;;;;;;;;;;;,,'....dNWMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMNkc,....'',,,,,,,'''''...:ONMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMWNOoc;,...........',:d0WMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMWNNNNNXXXKOkdooolllodkO0XXXXXXXXNWMMMMMMMMMMMMM
```