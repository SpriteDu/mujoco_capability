#pragma once
#include <string>
#include <mujoco/mujoco.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ostream>
/**
 * Class DataHandler
 *
 * This class is responsible for handling data recording for a MuJoCo simulation.
 */
class DataHandler
{
public:
    /**
     * Constructor for DataHandler.
     *
     * @param file_name The name of the file to which data will be recorded.
     *                  Defaults to "New_Record".
     * @param current_time Pointer to a double representing the current simulation time.
     *                     This can be used to timestamp the recorded data.
     */
    DataHandler(const std::string &file_name = "New_Record", double *current_time = nullptr);

    /**
     * Opens the data file for recording.
     * This method should be called before any data recording begins.
     */
    void openData();

    /**
     * Records general data from the simulation.
     *
     * @param m Pointer to the MuJoCo model (mjModel).
     * @param d Pointer to the MuJoCo data (mjData), used for getting general simulation data.
     * @param current_time Pointer to the current simulation time.
     */
    void record_data(const mjModel *m, mjData *d, double *current_time);

    /**
     * Records contact force data for a specific contact ID.
     *
     * @param m Pointer to the MuJoCo model (mjModel).
     * @param d Pointer to the MuJoCo data (mjData), used for getting contact force data.
     * @param contactF Pointer to an array containing contact force data.
     * @param ID The ID of the contact point to record data for.
     */
    void record_contact(const mjModel *m, mjData *d, const mjtNum *contactF, int ID);

private:
    std::string file_name;
    double *current_time;
    std::unique_ptr<double[]> tip_index_force_ptr;
};