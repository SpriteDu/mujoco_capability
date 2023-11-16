// Adapted by Diego Hidalgo, 2023
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <iostream>
#include <Eigen/Dense>
#include <time.h>
#include <thread>
#include <iostream>
#include <random>
#include <string>
#include "tool_kits.h"

#define PI 3.14159265358979323846

using namespace std;
using Eigen::MatrixXd;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

char filename[] = "/home/jididu/Documents/mujoco-2.3.7/Project/mujoco-capability/franka_panda_RH8D_R.xml";

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void controlSystem(const mjModel* m, mjData* d, double ref[])
{
    int n_joints = m->nu;
    float kp = 0.013f;
    double err = 0.0;
    for(int i = 0; i < n_joints; i++)
    {
        err = ref[i] - d->qpos[i];
        d->ctrl[i] = d->ctrl[i] + kp*(err);
    }
}

void pcontroller(const mjModel* m, mjData* d)
{
  cout << m->nu << m->nv <<endl;
  if( m->nu==m->nv )
  std::cout << "equal"<<endl;
  double ref[] = {0.470887, 0.978366, 2.6895, -2.15875, 2.75034, 1.88727, 0.707};

    mju_scl3(d->ctrl, d->qvel, -0.1);

}

void printSensorData(const mjModel* m, mjData* d, bool posvel)
{
    if (posvel)
        for (int i = 0; i < 5; i++)
            std::cout << d->sensordata[i*6] << d->sensordata[i*6 + 1] << d->sensordata[i*6 + 2] << std::endl;
    else
        for (int i = 0; i < 5; i++)
            std::cout << d->sensordata[i*6 + 3] << d->sensordata[i*6 + 4] << d->sensordata[i*6 + 5] << std::endl;
}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" No arguments passed. Loading model...\n");
        //return 0;
    }

    // load and compile model
    char error[1000] = "Could not load binary model";

    if(argc < 2) {m = mj_loadXML(filename, 0, error, 1000);}
    else
    {
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    }
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);



    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1400, 900, "Robo arm hand simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    double arr_view[] = {86.68,-9.65863,2.54165,0.171193,0.0378619,0.61732};
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    //cout << "About to plot your results" << endl;
    Log("Loading worked correctly, plotting simulation now");

    int n_joints = 26;
    for(int i = 0; i<n_joints; i++)
        d->qpos[i] = 0;    

    /*
    d->qpos[0] = -2.5038;
    d->qpos[1] = -0.415952;
    d->qpos[2] = 1.34734;
    d->qpos[3] = -0.860236;
    d->qpos[4] = 0.555381;
    d->qpos[5] = 0.843954;
    d->qpos[6] = 0;*/


    ////////////////////////////////////////////////////////////////////////////////////////////
    double q7 = 0.707;
    std::array<double, 16> O_T_EE_array;
    std::array<double, 7> q_actual_array = {0};
    for (auto i = 0; i < 7; i++)
        q_actual_array[i] = d->qpos[i];

    O_T_EE_array[0] = -1; O_T_EE_array[4] = 0; O_T_EE_array[8] = 0;    O_T_EE_array[12] = -0.25;
    O_T_EE_array[1] = 0; O_T_EE_array[5] = -1; O_T_EE_array[9] = 0;    O_T_EE_array[13] = 0.13;
    O_T_EE_array[2] = 0; O_T_EE_array[6] = 0; O_T_EE_array[10] = 1;  O_T_EE_array[14] = 1.2;
    O_T_EE_array[3] = 0; O_T_EE_array[7] = 0; O_T_EE_array[11] = 0;   O_T_EE_array[15] = 1;      

    Ik_solution ik_sol;
    ik_sol.define_sol_par(O_T_EE_array, q_actual_array, q7);
    ik_sol.get_Solution();
    ik_sol.print_sol();
    /////////////////////////////////////////////////////////////////////////////////////////////


    int ncon = d->ncon; // Number of contacts

    double ref[n_joints] = {-1.42172, -1.18835, 0.977646, -2.34855, 0.884779, 1.46295, 0, // Panda joints
            0, // Forearm --> Should be kept in 0 as it is not actuated
            0, // Wrist adduction/abduction
            0.50, // Wrist Flexion/Extension
            1.5, // Thumb MCP Adduction
            0.5, // Thumb MCP Flexion
            0.70, // Thumb PIP Flexion
            0.70, // Thumn DIP Flexion
            1.5, // Index MCP Flexion
            0.70, // Index PIP Flexion
            0.70, // Index DIP Flexion
            1.5, // Middle MCP Flexion
            0.70, // MIddle PIP Flexion
            0.70, // MIddle DIP Flexion
            1.50, // Ring MCP flexion
            0.70, // Ring PIP Flexion
            0.70, // Ring DIP Flexion
            1.50, // Little MCP Flexion
            0.70, // Little PIP Flexion
            0.70}; // Little DIP Flexion

    for (int i = 0; i < 7; i++)
        ref[i] = ik_sol.q_sol[i];

    // run main loop, target real-time simulation and 60 fps rendering
    /*
    MatrixXd m2(2,2);
    m2(0,0) = 3;
    m2(1,0) = 2.5;
    m2(0,1) = -1;
    m2(1,1) = 4;
    std::cout << m2 << std::endl;*/

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    //mjcb_control = controlLaw;              // initialize a control law    

    double desired_real_time_factor = 1.0;  // Adjust this factor as needed
    double normal_dt = 1.0 / 60.0;  // 60 Hz
    double dt = normal_dt / desired_real_time_factor;
    // m->opt.timestep = dt;  // the integral time in each step. should align with the step size defined earlier. 
    cout<<"the intergal time is " << m->opt.timestep<< endl;
    

    // for P controller the calling frequency influence the controled behavior
    double controlSystem_dt = 1.0 / 60.0;  // 60 Hz for control system
    double lastControlUpdateTime = 0.0;
    // mjcb_control = pcontroller;
    mjtNum start_time = d->time;;
    while( !glfwWindowShouldClose(window) )
    {        

        mjtNum simstart = d->time;
        
        // while last step is finished and a frame is rendered,, keep do the simulation without re-rendering.
        while (d->time - simstart < dt) {
        mj_step(m, d);
        }
        
        mjtNum elapsed_time = d->time - start_time;//set after the while loop to decrease the influence from.
        //steady the calling frequency of controllSystem.
        if (elapsed_time - lastControlUpdateTime >= controlSystem_dt){
            
            // mj_step(m,d);
            // mj_step1(m, d);
            controlSystem(m, d, ref);
            // mj_step2(m,d);
             lastControlUpdateTime = elapsed_time;
        }
       
        


        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    
    } // end simulation

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
    //delete[] corr_sys;
    return 1;
}

//         // // Maintain a consistent frame rate using sleep
//         // double sleep_time = dt - (glfwGetTime() - start_time);
//         // if (sleep_time > 0)
//         // {
//         //     // Sleep to control the frame rate
//         //     // Platform-specific sleep or delay function
//         //     std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
//         // }

//         /*ncon = d->ncon; // Number of contacts
//         std::cout << ncon << "/ " << d->contact[0].pos[0] << "/ " << d->contact[0].pos[1] << "/ " << d->contact[0].pos[2] <<
//                              "/ " << d->contact[0].frame[0] << "/ " << d->contact[0].frame[1] << "/ " << d->contact[0].frame[2] <<
//                              "/ " << d->contact[0].mu << "/ " << d->contact[0].geom1 <<  "/ " << d->contact[0].geom2
//                 << std::endl;*/




//         //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

