#include "../include/data_handler.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <chrono>

DataHandler::DataHandler(const std::string &file_name, double *current_time) : file_name(file_name), current_time(current_time)
{
    std::cout << "DataHandler object constructed, data file name: " << this->file_name << ".csv" << std::endl;
}

void DataHandler::openData()
{
    std::ofstream outputFile(file_name + ".csv", std::ios::app);
    if (!outputFile.is_open())
    {
        std::cerr << "Failed to open the file." << std::endl;
        return; // Return early if the file couldn't be opened.
    }
    // Get the current local time
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    // Convert to a readable format
    std::stringstream timeStream;
    timeStream << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    // // Write the local time to the file
    outputFile << "Experiment conducted at: ";
    if (timeStream)
    {
        outputFile << timeStream.str() << "\n";
    }
    else
    {
        outputFile << "N/A\n"; // Provide a default if current_time is not provided.
    }
    // outputFile << " joint" << std::endl;

    outputFile << "Body Name,Time,normal,tangent1,tangent2\n"; 

    outputFile.close();
}

void DataHandler::record_data(const mjModel *m, mjData *d, double *current_time)
{
    std::ofstream outputFile(file_name + ".csv", std::ios::app);
    if (!outputFile.is_open())
    {
        std::cerr << "Failed to open the file." << std::endl;
        return; // Return early if the file couldn't be opened.
    }

    if (current_time)
    {
        outputFile << "current_time: " << *current_time << std::endl;
    }
    else
    {
        outputFile << "current_time: N/A" << std::endl;
    }

    // Write d->ctrl array
    outputFile << "d_ctrl: ";
    for (int i = 0; i < sizeof(d->ctrl); i++)
    {
        outputFile << std::setprecision(3) << d->ctrl[i] << ' ';
    }
    outputFile << std::endl;

    // Write d->qpos array
    outputFile << "d_qpos: ";
    for (int i = 0; i < sizeof(d->ctrl); i++)
    {
        outputFile << std::setprecision(3) << d->qpos[i] << ' ';
    }
    outputFile << std::endl;
    outputFile << "contact: " << d->ncon << " joints: ";

    outputFile << std::endl;
    outputFile.close();
}

void DataHandler::record_contact(const mjModel *m, mjData *d, const mjtNum *contactF, int ID)
{
    std::ofstream outputFile(file_name + ".csv", std::ios::app);

    if (!outputFile.is_open())
    {
        std::cerr << "Failed to open the file." << std::endl;
    }

    outputFile << mj_id2name(m, mjOBJ_BODY, ID) << "," << d->time << ",";
    for (int i = 0; i < 3; i++) // maximum to 6 as the contact force returns force and torque.
    {
        outputFile << std::abs(contactF[i]);
        if (i < 2)
        { // Add comma
            outputFile << ",";
        }
    }

    outputFile << "\n";
}