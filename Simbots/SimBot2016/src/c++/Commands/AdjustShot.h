/*
 * AdjustShot.h
 *
 *  Created on: Jun 20, 2016
 *      Author: dean
 */

#ifndef SRC_COMMANDS_ADJUSTSHOT_H_
#define SRC_COMMANDS_ADJUSTSHOT_H_

#include <WPILib.h>
#include <Commands/CommandGroup.h>

class AdjustShot: public Command {
    int ntargets=0;
    bool at_vtarget=false;
    bool at_htarget=false;
    bool on_target;

public:
    AdjustShot();
    void Initialize();
    bool IsFinished();
    void Execute();
    void End();
    void Interrupted() { End();}

    class AdjustVAngle: public PIDSource, public PIDOutput{
    private:
        PIDController pid;
    public:
        AdjustVAngle(double P, double I, double D);
        double PIDGet();
        void PIDWrite(double d);
        bool AtTarget();
        void Initialize();
        void End();
    };
    class AdjustHAngle: public PIDSource, public PIDOutput{
    private:
        PIDController pid;
    public:
        AdjustHAngle(double P, double I, double D);
        double PIDGet();
        void PIDWrite(double d);
        bool AtTarget();
        void Initialize();
        void End();
    };
    AdjustHAngle Hcntrl;
    AdjustVAngle Vcntrl;
};

#endif /* SRC_COMMANDS_ADJUSTSHOT_H_ */
