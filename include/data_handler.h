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
    void record_contact(const mjModel* m, mjData* d,const mjtNum* contactF, int ID);
    double getElement(size_t index) const;
    void setElement(size_t index, double value);
private:
    std::string file_name;
    double* current_time;
    std::unique_ptr<double[]> tip_index_force_ptr;
    // const char* objectName = "Multi_shaped_object";
    // int objectID = mj_name2id(m, mjOBJ_BODY, objectName);
    // const char* fingerTipName = "Index_Distal";
    // const int indexTipID = mj_name2id(m, mjOBJ_BODY, fingerTipName);
    // mjtNum tip_index_force[6];
    // int tip_index_ncon{0};
    // int object_ncon{0};
    // int bodyID{-1};
    // int fingerID{-1};
    
    // const char* tableName = "Table";
    // const int tableID = mj_name2id(m, mjOBJ_BODY, tableName);
    // int table_ncon{0};
};