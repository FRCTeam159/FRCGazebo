/*
 * Vision.h
 *
 *  Created on: Jun 20, 2016
 *      Author: dean
 */

#ifndef SRC_SUBSYSTEMS_VISION_H_
#define SRC_SUBSYSTEMS_VISION_H_

#include "WPILib.h"
#include <Commands/Subsystem.h>

#define MAX_TARGETS 4
struct TargetInfo {
    double width;
    double height;
};
struct TargetData {
    double centerX;
    double centerY;
    double width;
    double height;
    double area;
};
struct CameraInfo {
    int screen_width;
    int screen_height;
    double fov;
    double fov_factor;
};

class Vision: public Subsystem {
protected:

    CameraInfo camera;
    TargetData targets[MAX_TARGETS];
    TargetInfo target;
    TargetData closest;
    int ntargets=0;
    double camera_hoffset;
    double camera_voffset;
    void ClearTargets();
    double calcTargetHOffset(double range);
    double calcTargetVOffset(double range);
    void Log();
    bool auto_targeting=false;
    bool on_target=false;
    bool adjusting=false;

public:
    std::shared_ptr<NetworkTable> table;

    Vision();
    virtual ~Vision();
    void Init();
    void Reset();
    void InitDefaultCommand();
    void Update();
    int GetTargets();
    bool FindClosestTarget();

    void SetTargetSpecs(double w, double h);
    void SetCameraSpecs(int w, int h, double f);
    void SetCameraOffsets(double h, double v);
    double GetTargetDistance();
    double GetTargetVerticalAngle();
    double GetTargetHorizontalAngle();
    void PrintRawTargetData();
    void PrintTargetOffsets();
    CameraInfo GetCameraInfo() { return camera;}
    TargetInfo GetTargetInfo() { return target;}

    bool GetAutoTargeting() {
        return auto_targeting;
    }

    void SetAutoTargeting(bool b) {
        auto_targeting=b;
    }

    bool OnTarget() {
        return on_target;
    }

    void SetOnTarget(bool b) {
        on_target=b;
    }
    bool Adjusting() {
        return adjusting;
    }

    void SetAdjusting(bool b) {
        adjusting=b;
    }
    void AutonomousInit();
    void TeleopInit();
    void DisabledInit();

};

#endif /* SRC_SUBSYSTEMS_VISION_H_ */
