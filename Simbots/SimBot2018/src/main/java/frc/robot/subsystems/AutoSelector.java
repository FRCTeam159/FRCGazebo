package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.PhysicalConstants;
import frc.robot.Robot;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetGrabberState;
import frc.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


/**
 *
 */
public class AutoSelector extends SubsystemBase implements Constants, PhysicalConstants {
  public static int targetObject = OBJECT_SCALE;
  public static int targetSide = POSITION_LEFT;
  
  //strategy options

  
  public static final int SAME_SIDE_SWITCH = 0;
  public static final int SAME_SIDE_SCALE = 1;
  public static final int OTHER_SIDE_SCALE = 2;
  public static final int TWO_CUBE_AUTO = 3;

  public static int autoTarget = SAME_SCALE;

  SendableChooser<Integer> strategyChooser= new SendableChooser<Integer>();

  public static Integer strategyOption = SAME_SIDE_SCALE;

  edu.wpi.first.wpilibj2.command.Command autonomous;
  public AutoSelector(){
    super();
    // 4-position switch options
    strategyChooser.setDefaultOption("Same Side Scale", SAME_SIDE_SCALE);
    strategyChooser.addOption("Same Side Switch", SAME_SIDE_SWITCH);
    strategyChooser.addOption("Other Side Scale", OTHER_SIDE_SCALE);
    strategyChooser.addOption("Two Cube Auto", TWO_CUBE_AUTO);
    SmartDashboard.putData("Strategy", strategyChooser);
  }
  int getSoftStrategy() {
    return strategyChooser.getSelected();
  }
  int getHardStrategy() {
    // replace this with getHardwareSwitches in real robot code
    return strategyChooser.getSelected();
  }

  void getStrategy() {  
    strategyOption = getSoftStrategy();
    setAutoTarget();
    showAutoTarget();
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Command getAutonomous() {
    getStrategy();
    return getAutoCommand(autoTarget);
  }

  void setAutoTarget() {
    String gameMessage = Robot.fms_string;
    boolean allgood = false;
    boolean allbad = false;
    targetObject = OBJECT_NONE;

    if (Robot.robotPosition == POSITION_CENTER) {
      targetObject = OBJECT_SWITCH;
      if (gameMessage.charAt(0) == 'R')
        targetSide = POSITION_RIGHT;
      else
        targetSide = POSITION_LEFT;
      if(strategyOption==TWO_CUBE_AUTO)
        autoTarget=TWO_CUBE_CENTER;
      else
        autoTarget=CENTER_SWITCH;
      return;
    } else if (Robot.robotPosition == POSITION_RIGHT) {
      targetSide = POSITION_RIGHT;
      if (gameMessage.charAt(1) == 'R' && gameMessage.charAt(0) == 'R')
        allgood = true;
      else if (gameMessage.charAt(1) == 'L' && gameMessage.charAt(0) == 'L')
        allbad = true;
      else if (gameMessage.charAt(1) == 'R' && gameMessage.charAt(0) == 'L')
        targetObject = OBJECT_SCALE;
      else 
        targetObject = OBJECT_SWITCH;
    } else { // left position
      targetSide = POSITION_LEFT;
      if (gameMessage.charAt(1) == 'L' && gameMessage.charAt(0) == 'L')
        allgood = true;
      else if (gameMessage.charAt(1) == 'R' && gameMessage.charAt(0) == 'R')
        allbad = true;
      else if (gameMessage.charAt(1) == 'L' && gameMessage.charAt(0) == 'R')
        targetObject = OBJECT_SCALE;
      else 
        targetObject = OBJECT_SWITCH;
    }
    // select the highest level action allowed by FMS pattern and strategy selection
    switch (strategyOption) {
    case TWO_CUBE_AUTO:
      if(allgood) {
        autoTarget=TWO_CUBE_SIDE;
        targetObject = OBJECT_SCALE;
        break;
      } // intentional fall-thru
    case OTHER_SIDE_SCALE:
      if(allbad) {
        autoTarget=OTHER_SCALE;
        targetObject = OBJECT_SCALE;
        targetSide = (Robot.robotPosition == POSITION_RIGHT)?POSITION_LEFT:POSITION_RIGHT;
        break;
      }// intentional fall-thru
    case SAME_SIDE_SCALE:
      if(allgood || targetObject == OBJECT_SCALE) {
        autoTarget=SAME_SCALE;
        targetObject = OBJECT_SCALE;
        break;
      }// intentional fall-thru
    case SAME_SIDE_SWITCH:
      if(allgood || targetObject == OBJECT_SWITCH){
        autoTarget=SAME_SWITCH;
        targetObject = OBJECT_SWITCH;
      }
      else {
        autoTarget=GO_STRAIGHT;
        targetObject = OBJECT_NONE;
      }
    }
  }

  public static void showAutoTarget() {
    String which_object[] = { "Switch", "Scale", "Straight" };
    String which_side[] = { "Center", "Left", "Right" };
    String targetString = which_side[targetSide] + "-" + which_object[targetObject];
    SmartDashboard.putString("Target", targetString);
  }

  private SequentialCommandGroup getAutoCommand(int target) {
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    boolean mirror=(Robot.robotPosition==POSITION_LEFT);
    autoCommand.addCommands(new SetGrabberState(HOLD, 0.1));
    switch(target) {
    case GO_STRAIGHT:
      System.out.println("Go Straight");
      autoCommand.addCommands(new DrivePath(GO_STRAIGHT,false,false));
      break;
    case CENTER_SWITCH:
      mirror=(targetSide==POSITION_RIGHT);
      System.out.println("Center Switch");
      autoCommand.addCommands(new ParallelCommandGroup(
        new SetElevator(SWITCH_DROP_HEIGHT, 2.0),
        new DrivePath(CENTER_SWITCH,mirror,false)
      ));
      autoCommand.addCommands(new SetGrabberState(PUSH, 1.0));
      break;
    case SAME_SWITCH:
      System.out.println("Same Switch");
      autoCommand.addCommands(new ParallelCommandGroup(
        new SetElevator(SWITCH_DROP_HEIGHT, 4.0),
        new DrivePath(SAME_SWITCH,mirror,false)
      ));
      autoCommand.addCommands(new SetGrabberState(PUSH, 1.0));
      break;
    case SAME_SCALE:
      System.out.println("Same Scale"); 
      autoCommand.addCommands(
      new ParallelCommandGroup(
          new SetElevator(SWITCH_DROP_HEIGHT, 4.0),
          new DrivePath(SAME_SCALE,mirror,false)
      )
      ,
      new SetElevator(SCALE_DROP_HEIGHT, 5.0),
      new SetGrabberState(PUSH, 1.0)
      );
      
      break;
    case OTHER_SCALE:
      System.out.println("Other Scale");
      autoCommand.addCommands(new ParallelCommandGroup(
        new SetElevator(SWITCH_DROP_HEIGHT, 2.0),
        new DrivePath(OTHER_SCALE,mirror,false)
      ));
      autoCommand.addCommands(new SetElevator(SCALE_DROP_HEIGHT, 2.0));
      autoCommand.addCommands(new SetGrabberState(PUSH, 1.0));
      break;
    case TWO_CUBE_SIDE:
      System.out.println("Two Cube Side");
      autoCommand = getAutoCommand(SAME_SCALE); // reentrant call !
      autoCommand.addCommands(new SetElevator(0, 2.0)); // drop elevator and prepare to grab
      if (Robot.robotPosition == POSITION_RIGHT) // note: pathfinder can't turn in place or drive in reverse
        autoCommand.addCommands(new TurnToAngle(125, 3.0));
      else
        autoCommand.addCommands(new TurnToAngle(-125.0, 3.0));
      autoCommand.addCommands(new SetGrabberState(OPEN, 0.5));
      autoCommand.addCommands(new SetGrabberState(GRAB, 0.5));
      autoCommand.addCommands(new DrivePath(TWO_CUBE_SIDE,mirror,false));
      autoCommand.addCommands(new SetGrabberState(CLOSE, 0.5));
      autoCommand.addCommands(new SetElevator(SWITCH_DROP_HEIGHT, 4.0));
      if (Robot.robotPosition == POSITION_RIGHT) // turn more toward center of switch
        autoCommand.addCommands(new TurnToAngle(20.0, 3.0));
      else
        autoCommand.addCommands(new TurnToAngle(-25.0, 3.0));
      autoCommand.addCommands(new SetGrabberState(PUSH, 1.0));
      break;
    case TWO_CUBE_CENTER:
      System.out.println("Two Cube Center");
      autoCommand = getAutoCommand(CENTER_SWITCH);  // place first cube (reentrant call !)
      mirror=(targetSide==POSITION_LEFT); // inverted for backwards travel
      autoCommand.addCommands(new DrivePath(TWO_CUBE_CENTER,mirror,true)); // reverse s-turn from switch
      autoCommand.addCommands(new ParallelCommandGroup(
        new SetElevator(0, 2.0),   // set intake to grab cube
        new SetGrabberState(OPEN, 0.5),
        new SetGrabberState(GRAB, 0.5)
      ));
      autoCommand.addCommands(new DriveStraight(28, 0.4,2.0,0)); // drive forward and grab end cube
      autoCommand.addCommands(new SetGrabberState(CLOSE, 0.5));
      autoCommand.addCommands(new DriveStraight(-24, 0.4,2.0,0)); // back up
      autoCommand.addCommands(new ParallelCommandGroup(
        new SetElevator(SWITCH_DROP_HEIGHT, 2.0),
        new DrivePath(TWO_CUBE_CENTER,!mirror,false)
      ));
      autoCommand.addCommands(new SetGrabberState(PUSH, 1.0));
      break;
    }
    return autoCommand;
  }
}
