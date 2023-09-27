#include <iostream>
#include <string>
#include <array>
#include "tool_kits.h"
#include "franka_ik_He.hpp"

using namespace std;

void Log(const char* text)
{
    cout << text << endl;
}

void Log(int var)
{
    cout << var << endl;
}

void Log(double var)
{
    cout << var << endl;
}

double random_generator(){
    return (double)rand() / RAND_MAX; // for generating random points between 0 to 1
}

Ik_solution::Ik_solution(std::array<double, 16> des_EE, std::array<double, 7> actual_q, double des_q7){
    O_T_EE = des_EE;
    q_actual_array = actual_q;
    q7 = des_q7;
    std::cout << "Inverse kinematics solution created" << std::endl;
}

Ik_solution::Ik_solution(){
    std::cout << "Inverse kinematics solution created" << std::endl;
}

void Ik_solution::define_sol_par(std::array<double, 16> des_EE, std::array<double, 7> actual_q, double des_q7){
    O_T_EE = des_EE;
    q_actual_array = actual_q;
    q7 = des_q7;
    std::cout << "Parameters updated" << std::endl;
}

void Ik_solution::get_Solution(){
    q_out_cc = franka_IK_EE_CC(O_T_EE, q7, q_actual_array);
    int cnt[4] = {0,0,0,0};
    double dist[4] = {0};
    for (auto i = 0; i < 7; i++){
        if (q_out_cc[i] != q_out_cc[i]){ // NaN values have the odd property that comparisons involving them are always false
            flag_sol = true;
            break;
        }
    }
    if (flag_sol){
        q_out = franka_IK_EE(O_T_EE, q7, q_actual_array);
        for (auto i = 0; i < 4; i++)
            for (auto j = 0; j < 7; j++){
                if (q_out[i][j] != q_out[i][j]){ // NaN values have the odd property that comparisons involving them are always false
                    cnt[i] = 1;
                    break;
                }
                else
                    dist[i] += (q_actual_array[j] - q_out[i][j])*(q_actual_array[j] - q_out[i][j]); // Getting the distance of each solution
            }
        if ((cnt[0]+cnt[1]+cnt[2]+cnt[3]) == 4)
            std::cout << "No solution satisfies the requested EE frame \n";
        else{
            auto sol = Ik_solution::get_closest_sol(dist, 4);
            for (auto i = 0; i < 7; i++)
                q_sol[i] = q_out[sol][i];
        }
    }
    else
        for (auto i = 0; i < 7; i++)
            q_sol[i] = q_out_cc[i];
    std::cout << cnt[0] << ", " << cnt[1] << ", " << cnt[2] << ", " << cnt[3] << ", " << std::endl;
}

void Ik_solution::print_sol(){
    for (int i = 0; i < q_sol.size(); i++)
        std::cout << q_sol[i] << ", ";
    std::cout << "\n";
}

int Ik_solution::get_closest_sol(double* dist, int n_sol){
    double min_val = 1000;
    int min;
    for (auto i = 0; i < n_sol ; i++)
        if (dist[i] < min_val && dist[i] != 0){
            min_val = dist[i];
            min = i;
        }
    return min;
}