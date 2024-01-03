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

/**
 * @brief Record contact data to a CSV file.
 * 
 * The data includes simulation time, contact name, and contact force.
 * 
 * @param m Pointer to the MuJoCo model (mjModel).
 * @param d Pointer to the MuJoCo data (mjData), used for getting the simulation time.
 * @param contactF Pointer to an array containing contact force data.
 * @param name Name of the contact point (obtained from MuJoCo).
 */
void DataHandler::record_contact(const mjModel* m, mjData* d,const mjtNum* contactF,int ID){
    std::ofstream outputFile(file_name + ".csv", std::ios::app);
    
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
    }
    // for (int i = 0; i <=6; ++i) {
        
    //     if (i < 5) { // Avoid adding a comma after the last element
    //     outputFile << contactF[i];
    //     outputFile << ",";
    //     }
    //     else outputFile << "\n";
    //     return; // Return early if the file couldn't be opened.
    // }

outputFile << d->time << "," << mj_id2name(m,mjOBJ_BODY,ID) << ",";
for (int i = 0; i < 6; ++i) {
    outputFile << contactF[i];
    if (i < 5) { // Add a comma after each element except the last
        outputFile << ",";
    }
}

outputFile << "\n"; // Add a newline after writing all elements

    // Write d->ctrl array
    // outputFile << "Index,Contact geom1,geom2,PosX,PosY,PosZ,tangent1, 2, spin, roll1, 2,Mu\n";
    // outputFile << "Index,Contact_thumb,index,middle,ring,little\n";
    // Iterate over the contacts and write their data

        // for (int i = 0; i < d->ncon; i++) { //loop over all contacts. 
        // int body1 = m->geom_bodyid[d->contact[i].geom1];
        // int body2 = m->geom_bodyid[d->contact[i].geom2];
        // std::cout <<" body 1:  " << body1 << ". body 2:  " <<body2 << " objectID " << objectID << " indexTipID " << indexTipID << std::endl;

        // if (body1 == objectID || body2 == objectID)
        // {
        //     body1 == objectID ? fingerID = body2 : fingerID = body1;
        //     if (fingerID = indexTipID) { 
        //         // std::cout << "index" << std::endl;
        //         mj_contactForce(m, d, i, tip_index_force);
        //         tip_index_ncon++;
        //         std::cout << "Contact Force as a Vector: ["
        //             << tip_index_force[0] << ", "
        //             << tip_index_force[1] << ", "
        //             << tip_index_force[2] << "]" << " Number " << tip_index_ncon << std::endl;        
    
        //     }

        // }
            
        // }

    // for (int i = 0; i < m->nsensordata; ++i) {
    // outputFile << d->sensordata[i];
    // if (i < m->nsensordata - 1) {
    //     outputFile << ";";
    // } else {
    //     outputFile << "\n";
    // }
    //     }

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

double DataHandler::getElement(size_t index) const {
    if (index < 6) {
        return tip_index_force_ptr[index];
    }
    throw std::out_of_range("Index out of range");
}

void DataHandler::setElement(size_t index, double value) {
    if (index < 6) {
        tip_index_force_ptr[index] = value;
        return;
    }
    throw std::out_of_range("Index out of range");
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
