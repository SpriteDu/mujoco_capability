#pragma once

void Log(const char* text);

void Log(int var);

void Log(double var);

double random_generator();

void Log(std::array<double, 7> q);

class Ik_solution{
    public:
        std::array<double, 16> O_T_EE = {0};
        std::array<double, 7> q_actual_array = {0};
        std::array<double, 7> q_out_cc = {0};
        std::array<double, 7> q_sol = {0};
        std::array< std::array<double, 7>, 4 > q_out = {0};
        double q7 = 0;
        bool flag_sol = false;

        Ik_solution(std::array<double, 16> des_EE, std::array<double, 7> actual_q, double des_q7);
        Ik_solution();
        void define_sol_par(std::array<double, 16> des_EE, std::array<double, 7> actual_q, double des_q7);        
        void get_Solution();
        void print_sol();        

    static int get_closest_sol(double* dist, int* cnt, int n_sol);
};