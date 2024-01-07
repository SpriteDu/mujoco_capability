#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include <mujoco/mujoco.h> // Assuming mujoco.h is the header for mjModel and mjData

class ControlSystem
{
public:
    ControlSystem(const mjModel *model);
    int update(mjData *d, double ref[], int stage);

private:
    const mjModel *m;
    float kp;
    double threshold;
    int n_joints;
};

#endif // CONTROLSYSTEM_H
