#include "ControlSystem.h"
#include <cmath>

/**
  @brief ControlSystem Constructor
  This constructor initializes the ControlSystem class with a given mjModel.
  It sets the proportional control constant (kp) and the threshold value for error checking.


  @param model - Pointer to an mjModel object representing the MuJoCo model.
  This object contains information about the physical model to be controlled.

  Member Variables Initialization:
  @param m Pointer to the MuJoCo model (mjModel).
  @param kp - Proportional control constant, set to 0.07f. Used in the control algorithm
  to determine the control response based on the error value.
  @param threshold - Error threshold, set to 0.01. Used to determine when a joint is
  considered to be at the desired position.
  @param n_joints - Extracts and stores the number of actuator (control) inputs in the model.
*/
ControlSystem::ControlSystem(const mjModel *model) : m(model), kp(0.005f), threshold(0.01)
{
    n_joints = m->nu;
}

int ControlSystem::update(mjData *d, double ref[], int stage)
{
    double err = 0.0;
    int joint_at_position = 0;

    switch (stage)
    {
    case 0:
    {
        d->ctrl[10] = 1.5;
        for (int i = 0; i < 9; i++)
        { // the time of loop could affect the result
            err = ref[i] - d->qpos[i];
            d->ctrl[i] = d->ctrl[i] + kp * err;
            if (std::fabs(err) <= threshold)
            {
                joint_at_position++;
            }
        }
        for (int i = 11; i < n_joints; i++)
        {
            d->ctrl[i] = 0;
        }
        if (joint_at_position == 7)
        {
            stage = 1;
        }
        break;
    }
    case 1:
    { // reaches the position, grab object
        int finger_joint_at_position = 0;
        for (int i = 11; i < n_joints; i += 3)
        {
            err = 0.9 - d->qpos[i];
            d->ctrl[i] = d->ctrl[i] + kp * err;
            if (std::fabs(err) < 0.02)
            {
                finger_joint_at_position++;
            }
        }
        if (finger_joint_at_position >= 2)
        {
            for (int i = 11; i < n_joints; i += 3)
            {
                err = 1 - d->qpos[i];
                d->ctrl[i] = 0.9;
            }
            stage = 2;
        }
        break;
    }
    case 2:
    {
        int i = 5;
        err = 3 - d->qpos[i];
        d->ctrl[i] = d->ctrl[i] + kp * err;
        err = 3 - d->qpos[6];
        d->qpos[6] = d->ctrl[6] + kp * 0.01 * err;
        break;
    }
    default:
        break;
    }

    return stage;
}
