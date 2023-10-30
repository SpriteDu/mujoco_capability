#include "include/data_handler.h"
#include <iostream>

DataHandler::DataHandler(const std::string& file_name, double* current_time) : file_name(file_name), current_time(current_time) {}

void DataHandler::openData() {
    std::ofstream outputFile(file_name + ".txt", std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return; // Return early if the file couldn't be opened.
    }

    outputFile << "time: ";
    if (current_time) {
        outputFile << *current_time;
    } else {
        outputFile << "N/A"; // Provide a default if current_time is not provided.
    }
    outputFile << " joint" << std::endl;

    outputFile.close();
}

void DataHandler::record_data(const mjModel* m, mjData* d, const std::string& file_name, double* current_time) {
    std::ofstream outputFile(file_name + ".txt", std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return; // Return early if the file couldn't be opened.
    }

    if (current_time) {
        outputFile << "current_time: " << *current_time << std::endl;
    } else {
        outputFile << "current_time: N/A" << std::endl;
    }

    // Write d->ctrl array
    outputFile << "d_ctrl: ";
    for (int i = 0; i < sizeof(d->ctrl); i++) {
        outputFile << std::setprecision(3) << d->ctrl[i] << ' ';
    }
    outputFile << std::endl;

    // Write d->qpos array
    outputFile << "d_qpos: ";
    for (int i = 0; i < sizeof(d->ctrl); i++) {
        outputFile << std::setprecision(3) << d->qpos[i] << ' ';
    }
    outputFile << std::endl;

    outputFile.close();
}