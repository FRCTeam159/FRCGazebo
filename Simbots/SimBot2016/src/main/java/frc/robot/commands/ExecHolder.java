package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ExecHolder extends CommandBase {

    enum State {
        FIND_ZERO,
        WAIT_FOR_BALL_TO_ENTER,
        GO_TO_FORWARD_LIMIT,
        WAIT_FOR_PUSH_REQUEST,
        WAIT_FOR_BALL_TO_LEAVE,
        GO_TO_REVERSE_LIMIT,
        REMOVE_BALL,
        PUSH_ERROR
    }

    State state = State.FIND_ZERO;
    static final double BALLDETECTIONDELAY = 0.5;
    static final double BALLREMOVEDELAY = 2.0;

    double elapsed;
    double start_time;
    double mark_time;

    boolean timing = false;
    boolean debug = true;

    //Timer timer = new Timer();
    //Timer delta_timer = new Timer();
    double delta_time;

    public ExecHolder() {
        state = State.FIND_ZERO;
        elapsed = 0;
        addRequirements(Robot.holder);
        //timer.start();
        //delta_timer.start();
    }

    void debugPrint(String msg) {
        if (debug)
            System.out.format("Holder %1.2f %s\n",elapsed,msg);
    }

    void setDeltaTimeout(double t) {
        timing = true;
        delta_time = t;
        mark_time=Robot.getTime()-start_time;
        //delta_timer.reset();
    }

    boolean checkTimeout() {
        if (elapsed-mark_time >= delta_time) {
            timing = false;
            return true;
        }
        return false;
    }

    @Override
    public void initialize() {
        start_time = Robot.getTime();
        if (!Robot.holder.isInitialized()) {
            debugPrint("Initializing Holder");
            Robot.holder.initialize();
        }
    }

    @Override
    public void execute() {
        //if(Robot.inAuto())
         //   return;
        elapsed=Robot.getTime()-start_time;
        switch (state) {
            case FIND_ZERO:
                findZero();
                break;
            case WAIT_FOR_BALL_TO_ENTER:
                waitForBallToEnter();
                break;
            case GO_TO_FORWARD_LIMIT:
                goToForwardLimit();
                break;
            case WAIT_FOR_PUSH_REQUEST:
                waitForPushRequest();
                break;
            case WAIT_FOR_BALL_TO_LEAVE:
                waitForBallToLeave();
                break;
            case GO_TO_REVERSE_LIMIT:
                goToReverseLimit();
                break;
            case REMOVE_BALL:
                removeBall();
                break;
            case PUSH_ERROR:
                pushError();
                break;
        }
    }

    void findZero() {
        if (Robot.holder.findZero()) {
            debugPrint("Zero Limit switch found");
            state = State.WAIT_FOR_BALL_TO_ENTER;
        }
    }

    // ==========================================================================================
    // ExecHolder::WaitForBallToEnter()
    // ==========================================================================================
    // - Wait for ball to enter
    // - Then delay to give the ball time to settle
    // - Then pinch the ball (goto forward limit)
    // ==========================================================================================
    void waitForBallToEnter() {
        // Robot.holder.CloseGate();
        if (Robot.holder.isBallPresent()) {
            if (!timing) {
                setDeltaTimeout(BALLDETECTIONDELAY);
                debugPrint("New Ball Detected - Waiting Settling Period..");
            } else if (checkTimeout()) {
                debugPrint("Opening Gate ..");
                state = State.GO_TO_FORWARD_LIMIT;
            }
        } else
            Robot.holder.closeGate();
    }

    // ==========================================================================================
    // ExecHolder::GoToForwardLimit()
    // ==========================================================================================
    // - Pinch the ball
    // ==========================================================================================
    void goToForwardLimit() {
        if (!Robot.holder.isBallPresent()) {
            state = State.GO_TO_REVERSE_LIMIT;
        } else {
            Robot.holder.openGate();
            if (Robot.holder.isGateOpen()) {
                state = State.WAIT_FOR_PUSH_REQUEST;
            }
        }
    }

    // ==========================================================================================
    // ExecHolder::waitForPushRequest()
    // ==========================================================================================
    // - Hold the ball while waiting for a push request
    // ==========================================================================================
    void waitForPushRequest() {
        if (!Robot.holder.isBallPresent()) {
            debugPrint("Ball not present");
            state = State.WAIT_FOR_BALL_TO_ENTER;
        } else {
            Robot.holder.holdBall();
            if (Robot.holder.pushRequested()) {
                debugPrint("Push Requested - Waiting for ball to leave..");
                state = State.WAIT_FOR_BALL_TO_LEAVE;
            }
        }
    }

    // ==========================================================================================
    // ExecHolder::WaitForBallToLeave()
    // ==========================================================================================
    // - After a push request, enable the push wheel (forward push)
    // - Start a timer to detect when the ball leaves
    // - If the ball is still present after the timeout, assume it is stuck and try
    // to remove it
    // - Else close the gate and wait for a new ball to enter
    // ==========================================================================================
    void waitForBallToLeave() {
        Robot.holder.pushBall();
        if (!timing)
            setDeltaTimeout(BALLDETECTIONDELAY);
        else if (checkTimeout()) {
            if (Robot.holder.isBallPresent()) {
                debugPrint("Push Failed - Attempting to remove the ball");
                state = State.REMOVE_BALL;
            } else {
                debugPrint("Push Succeeded - Closing Gate");
                state = State.GO_TO_REVERSE_LIMIT;
            }
        }
    }

    // ==========================================================================================
    // ExecHolder::GoToReverseLimit()
    // ==========================================================================================
    // - Move the gate backwards until it closes
    // ==========================================================================================
    void goToReverseLimit() {
        Robot.holder.closeGate();
        if (Robot.holder.isGateClosed()) {
            debugPrint("Gate Closed - Waiting for ball to enter");
            state = State.WAIT_FOR_BALL_TO_ENTER;
        }
    }

    // ==========================================================================================
    // ExecHolder::RemoveBall()
    // ==========================================================================================
    // - Try to remove a stuck ball by reversing the push and gate motors
    // - Start a timer
    // - If the timeout expires and the ball is still stuck go to an error state
    // - Else close the gate
    // ==========================================================================================
    void removeBall() {
        Robot.holder.removeBall();
        if (!timing)
            setDeltaTimeout(BALLDETECTIONDELAY);
        else if (checkTimeout()) {
            if (Robot.holder.isBallPresent()) {
                debugPrint("Remove Ball Failed - Entering Error State");
                state = State.PUSH_ERROR;
            } else {
                debugPrint("Remove Ball Succeeded - Closing Gate");
                state = State.GO_TO_REVERSE_LIMIT;
            }
        }
    }

    // ==========================================================================================
    // ExecHolder::PushError()
    // ==========================================================================================
    // - The ball is stuck
    // - Keep trying to close the gate
    // - Wait until the ball gets removed somehow
    // ==========================================================================================
    void pushError() {
        Robot.holder.closeGate();
        if (!Robot.holder.isBallPresent() && Robot.holder.isGateClosed())
            state = State.WAIT_FOR_BALL_TO_ENTER;
    }
}
