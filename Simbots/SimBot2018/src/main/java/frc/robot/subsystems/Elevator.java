package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommands;
import gazebo.SimEncMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Elevator extends SubsystemBase {
  static double inches_per_meter = 100 / 2.54;

  static public double max_outer_travel = 1.0 * inches_per_meter; // outer
  static public double max_inner_travel = 1.05 * inches_per_meter; // inner

  public static final double MAX_HEIGHT = 78;
  private static final double MIN_HEIGHT = 0;

  public static final double SWITCH_HEIGHT = 24;
  public static final double SCALE_HEIGHT = MAX_HEIGHT;
  public static final double START_HEIGHT = 1;
  double tolerance = 6;

  public static double travel_ratio = max_inner_travel / max_outer_travel;
  double setpoint = 0;

  Inner inner;
  Outer outer;

  public Elevator() {
    //super();
    outer = new Outer(RobotMap.ELEVATORMOTOR);
    inner = new Inner(RobotMap.ELEVATORMOTOR + 1);
    SmartDashboard.putNumber("Elevator Set", Math.round(setpoint));
    SmartDashboard.putNumber("Elevator Get", 0);
    reset();
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorCommands());
  }

  public void setPosition(double pos) {
    pos /= 2;
    double value = pos > max_outer_travel ? max_outer_travel : pos;
    value = value < 0 ? 0 : value;
    setpoint = value * 2;
    outer.setPosition(value);
    inner.setPosition(travel_ratio * value);
    // System.out.println("Elevator set:"+value);
    //SmartDashboard.putNumber("Elevator Set", Math.round(setpoint));
    //SmartDashboard.putNumber("Elevator Get", Math.round(getPosition()));
  }

  @Override
  public void simulationPeriodic() {
    outer.setPosition();
    inner.setPosition();
    SmartDashboard.putNumber("Elevator Set", Math.round(setpoint));
    SmartDashboard.putNumber("Elevator Get", Math.round(getPosition()));
  }
  public void reset() {
    //System.out.println("Elevator reset");
    outer.reset();
    inner.reset();
    setpoint = 0;
  }

  public void enable() {
    //System.out.println("Elevator enable");
    inner.enable();
    outer.enable();
  }

  public void disable() {
    //System.out.println("Elevator disable");
    outer.disable();
    inner.disable();
  }

  public double getPosition() {
    return inner.getPosition() * 2; // return inches
  }

  public double getSetpoint() {
    return setpoint; // return inches
  }

  public boolean atTarget() {
    if (Math.abs(getPosition() - setpoint) < tolerance)
      return true;
    return false;
  }

  // pid controller for elevator
  public class Outer {
    double P = 0.1;
    double I = 0.0;
    double D = 0.0;
    private SimEncMotor motor;
    PIDController pid;
    double setpoint = 0;
    boolean debug_in = false;
    boolean debug_out = false;
    int cycle_count = 0;

    public Outer(int i) {
      motor = new SimEncMotor(i);
      pid = new PIDController(P, I, D);
      enable();
    }

    public void disable() {
      //motor.disable();
    }

    public void enable() {
      motor.enable();
    }

    public void setPosition() {
      double p=pidGet();
      double c=pid.calculate(p);
      pidWrite(c);
    }
    public void setPosition(double d) {
      setpoint = d;
      pid.setSetpoint(setpoint);
      setPosition();
    }

    public double getPosition() {
      return inches_per_meter * motor.getDistance(); // return inches
    }

    public void reset() {
      setpoint = 0;
      pid.reset();
      pid.setSetpoint(setpoint);
    }

    public double pidGet() {
      if (debug_in)
        System.out.println("Outer::pidGet(" + getPosition() + ")");
      return getPosition();
    }

    public void pidWrite(double output) {
      cycle_count++;
      if (Double.isNaN(output)) {
        System.out.println("Outer::pidWrite(NAN) - aborting ! " + cycle_count);
        return;
      }
      if (debug_out)
        System.out.println("Outer::pidWrite(" + output + ")");
      motor.set(output);
    }
  }

  // pid controller for carriage
  class Inner {
    double P = 0.1;
    double I = 0.0;
    double D = 0.0;
    private SimEncMotor motor;
    PIDController pid;
    double setpoint = 0;
    boolean debug_in = false;
    boolean debug_out = false;
    int cycle_count = 0;

    public Inner(int i) {
      motor = new SimEncMotor(i);
      //motor.configEncoderCodesPerRev(1); // deprecated in new CTRE TalonSRX
      pid = new PIDController(P, I, D);
      enable();
    }

    public void disable() {
      //motor.disable();
    }

    public void enable() {
      motor.enable();
    }

    public void setPosition(double d) {
      setpoint = d;
      pid.setSetpoint(setpoint);
      setPosition();
    }

    public void setPosition() {
      double p=pidGet();
      double c=pid.calculate(p);
      pidWrite(c);
    }
    public double getPosition() {
      return inches_per_meter * motor.getDistance(); // return inches
    }

    public void reset() {
      setpoint = 0;
      pid.reset();
      pid.setSetpoint(setpoint);
    }

    public double pidGet() {
      if (debug_in)
        System.out.println("Inner::pidGet(" + getPosition() + ")");
      return getPosition();
    }

    public void pidWrite(double output) {
      cycle_count++;
      if (Double.isNaN(output)) {
        System.out.println("Inner::pidWrite(NAN) - aborting ! " + cycle_count);
        return;
      }
      if (debug_out)
        System.out.println("Inner::pidWrite(" + output + ")");
      motor.set(output);
    }
  }
}
