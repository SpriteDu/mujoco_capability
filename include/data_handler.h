#pragma once
#include <string>
#include <mujoco/mujoco.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ostream>


class DataHandler {
public:
    DataHandler(const std::string& file_name = "New_Record", double* current_time = nullptr);
    void openData();
    void record_data(const mjModel* m, mjData* d, double* current_time);
    void record_contact(const mjModel* m, mjData* d);
private:
    std::string file_name;
    double* current_time;
};