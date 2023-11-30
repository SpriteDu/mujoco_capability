#include "data_handler.h"
#include <iostream>

DataHandler::DataHandler(const std::string& file_name, double* current_time) : file_name(file_name), current_time(current_time) {
        std::cout << "DataHandler object constructed, data file name: " << this->file_name << ".csv"<< std::endl;
        
}

void DataHandler::openData() {
    std::ofstream outputFile(file_name + ".csv", std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return; // Return early if the file couldn't be opened.
    }

    outputFile << "time: ";
    if (current_time) {
        outputFile << *current_time;
    } else {
        outputFile << "N/A\n"; // Provide a default if current_time is not provided.
    }
    // outputFile << " joint" << std::endl;

    outputFile << "Index,Contact_thumb,index,middle,ring,little\n";

    outputFile.close();
}

void DataHandler::record_data(const mjModel* m, mjData* d, double* current_time) {
    std::ofstream outputFile(file_name + ".csv", std::ios::app);
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
    outputFile << "contact: "<< d->ncon << " joints: " ;

    outputFile << std::endl;
    outputFile.close();
}


void DataHandler::record_contact(const mjModel* m, mjData* d){
    std::ofstream outputFile(file_name + ".csv", std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return; // Return early if the file couldn't be opened.
    }

    // Write d->ctrl array
    // outputFile << "Index,Contact geom1,geom2,PosX,PosY,PosZ,tangent1, 2, spin, roll1, 2,Mu\n";
    // outputFile << "Index,Contact_thumb,index,middle,ring,little\n";
    // Iterate over the contacts and write their data





    for (int i = 0; i < m->nsensordata; ++i) {
    outputFile << d->sensordata[i];
    if (i < m->nsensordata - 1) {
        outputFile << ";";
    } else {
        outputFile << "\n";
    }
        }

        //  mjContact* contact = &d->contact[i];
        // //record index
        // outputFile << i <<"," ;
        // // Write Geom IDs
        // outputFile  << contact->geom1<< "," << contact->geom2<< ",";
        // // Write position friction of regularized cone
        // outputFile << contact->pos[0] << "," << contact->pos[1] << "," << contact->pos[2] << ",";
        
        // // Write frictions
        // for (int i = 0; i < 5; ++i) {
        //     outputFile << d->sensordata[i] << ",";
            
        // }
        // // Write mu
        // outputFile << contact->mu<<"\n";
         

        // 
    


}



#include <fstream>
#include <vector>

void saveContactsToCSV(const std::vector<mjContact>& contacts, const std::string& filename) {
    // Open an output file stream
    std::ofstream file(filename);

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // Write the header
    // file << "PosX,PosY,PosZ,Mu,Friction1,Friction2,Friction3,Friction4,Friction5,Geom1,Geom2\n";
    // file << "Index,Sensordata1,2,3,4,5";
    // Iterate over the contacts and write their data
    // for (int i=0;i<= m->nsensordata;i++ ){
    //     file<<d->sensordata[i];
    //     if i<m->nsensordata;{
    //         file<<"," ;
    //     }
    // }
    for (const auto& contact : contacts) {
        // Write position
        file << contact.pos[0] << "," << contact.pos[1] << "," << contact.pos[2] << ",";
        // Write mu
        file << contact.mu << ",";
        // Write frictions
        for (int i = 0; i < 5; ++i) {
            file << contact.friction[i];
            if (i < 4) file << ",";
        }
        // Write Geom IDs
        file << "," << contact.geom1 << "," << contact.geom2 << "\n";
    }

    // Close the file
    file.close();
}
