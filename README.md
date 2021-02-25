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
| X |  |  |
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
WNOo:;,,,,;:ldOXMMMMMWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
Kl,;;:;:;;:::,,cdONMMWWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
c,::;::::;;::;:;,:ldk0XWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
',:;;;:::;;::;:;;;;,,,;:clllodk0NWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMN0kolcclokKWMMMM
.;;;;;:::::::::;;:;,,,,,;;;,,''.,cd0WWMMMMMMMMMMMMMMMMMMMMMMMXkl,..........;dXMM
.,;::;:::::::::::;:::;::::::::;;,...c0WMMMMMMMMMMMMMMMMMMWWKo;'.',;,,,,,''...,kW
,.,;:::;::::::::;;:::;:::::::::;;;''.,0MMMMMMMMMMMMMMMMMWKo,',;:::;:::;:;,''...x
k,.,;;::::::::;;:::;::;;:::::::;::,'..cNMMMMMMMMMMMMMMMXd,.;:;:::::;;;:::;,'...'
WO,.,;;;:::;::;;:::::::::::::::::;,''.:XMMMMMMMMMWMMNOo;..;::;;:::::::::;:;,''.
MMKl'.,,;::;::;;::::::::::::::;:;;''..oWMMMMMMWNKOkdc;,'',;::;;:::::::::::;,''.
MMMWOc'.',,;;;:::::::::::;;:::;;,''..lXMMWN0xolc;,;;;;,,;::::;;:;:::::::;;;,''..
MMMMMW0o;..',,,,;;;;;;;;;;;;;,'''..,xNNOdo:,,,;:;;;,,;;;;::::;::::;;::::;:;,''..
MMMMMMMMNOo:,....'',',,,,,''....':xXNOc,,,::;;::;:::::::::::::::::;:::;::;,''..c
MMMMMMMMMMMWXkoc:;'.......',;cokXWWXo,,::::::;;:::::::::::::::::;;:::;;:;,''..;K
MMMMMMMMMMMMMMMMWNK0OOkOO0KXWWWMWMWd';:;::;:::;;;::::::::::::::::::;;::;,''..;0M
MMMMMMMMMMMMMMWKkdllllodkKWMWWWMMMXc':;;;::;;:::;::::::::::::::;::::;;;,''..cKMM
MMMMMMMMMMMWXd:,,,,;;;,,',co0NWMMMNl';::::::::::::;;;:;;;::;;;:::::;;,''..,xNMMM
MMMMMMMMWMNd,',;::::::::::;',lONMWW0;'::;;:::::;:;;::;;::;::::;::;,''...'oKWMMMW
MMMMMMMMMK:.,:;:::cc:c::::;::;,c0WWWOc,;:::::::;::;:::::::::::;;,''...;dKWMMMMMM
MMMMMMMM0;.,:;::lc:;;:ccc:::::;';kNWNXx:,,';;:::::;:::::::;,;;,'..';lONMMMMMMMMM
MMMMMMMX:.';::::c::::::cc:::::::,,dXNWWXOxc,'',,,;,,;;;,,'...',coxKWMMMMMMMMMMMM
MMMMMMMk..,;;;:::;;:::;;:;;;:::;,;,lKWMWWWWXOdlc::;,,;;::clok0XWMMMMMMMMMMMMMMMM
MMMMMMMd..,;;::::::;;;:;;::;;::;,;c;,oKWMMMMMMMMWWNXXXNWWWMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMo..,;;::;::::;::::::;;;::;',::,,lokXWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMx..,;:::;:::::::::::::::;::;,,,,...':dOXWMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMM0,.',;;:::::::::::::::::;;;:::;,'',,,'',cxKNMMWMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMWo..';::::::::::::::::::;;;::::::::::;:;,.':xXWWWMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMK;.'';::;;:::::::::::::;;;;;:::;:::::;;::;..'oKWMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMWk'.'',::;::::::::::::::::::::::::::::::::;,..'xNWWMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMWk..'',;:::::;;::::::::::::::::::::::::;;:;,'..lNMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMWO,..'',;;::::::::::::::::::::::::::::::;:,''..dWMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMKc..'',,;:;::::;;::::::::::::::;;;;::;::,''. ;XMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMNx,..''',;;:::::;:::::::::::;;:::::::;;,''. ;XMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMXo...''',;;;:::::::::::::::::::::;:;,'.'. lNMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMW0c....''',,;;;:::::::::::::::;;;,'.''. ,0MMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMW0o,...'''',,,;;;;;;;;;;;;;,,'''''...;0WMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMNkc,.....'''''''''''''''''''....,xXMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMNOd:,.....................,ckXMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMMMMMMMMMMWNKkdl:;;,,''''',;;cox0NWWMMMMMMMMMMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMXKXXKXKK0KKXKK0KKK000OOOO0XWWXKKK000KKK00KXWMMMMMMMMMMMMMMMMMM
MMMMMMMMMMMMMMMMMMX0KX0OO0OOOOOOKK0OO0K0OOO0XWN0kkkOOO0XKO0KXNMMMMMMMMMMMMMMMMMM
