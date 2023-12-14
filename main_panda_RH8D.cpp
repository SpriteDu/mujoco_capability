// Adapted by Diego Hidalgo, 2023
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <iostream>
#include <Eigen/Dense>
#include <time.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <random>
#include <string>
#include "tool_kits.h"
// #include "ui.cpp"
#include "include/data_handler.h"

#define PI 3.14159265358979323846

using namespace std;
using Eigen::MatrixXd;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvCamera cam2;
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
mjrContext con_hint;

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
        // stage = 0;
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

int controlSystem(const mjModel* m, mjData* d, double ref[], int stage)
{
    int n_joints = m->nu;
    float kp = 0.013f;
    double err = 0.0;
    double threshold = 0.01;
    int joint_at_position = 0;

    switch (stage)
    {
    case 0:{ // reach teh desired position 
        d->ctrl[10] =1.5;
        for(int i = 0; i < 9; i++) // the time of loop could affect the result 
        {
            
            err = ref[i] - d->qpos[i];
            d->ctrl[i] = d->ctrl[i] + kp*(err);
            // std::cout<< err << "<=" <<threshold<< std::endl;
            if(std::fabs(err) <= threshold){
                joint_at_position++;
                // std::cout<< "reached 1 " << std::endl;
            }

        }
        for (int i = 11; i < n_joints; i++){
            d->ctrl[i] = 0;
        }
        if ( joint_at_position ==7){
                stage = 1;
        }
        // std::cout<< "not yet" << std::endl;
    }
    break;

    case 1:{ // catch the object
        // std::cout <<"stage 1 " <<std::endl;
        int finger_joint_at_position = 0;
        for (int i = 11; i < n_joints; i++){
            err = 0.7 - d->qpos[i];
            d->ctrl[i] = d->ctrl[i] + kp*(err);

            // std::cout<< err << std::endl;

            if (std::fabs(err)<0.02){
                // cout<<"smaller"<<endl;
                finger_joint_at_position ++;
            }

        }
        if (finger_joint_at_position >= 5){
                stage = 2;
            }
        break;
    }
   

    case 2: { // lift the object.
        // std::cout <<"stage 2 " <<std::endl;
        int i = 5; 
        err = 3 - d->qpos[i];
        d->ctrl[i] = d->ctrl[i] + kp*(err);

        err = 3 - d->qpos[6];
         d->qpos[6] = d->ctrl[6] + kp*(err);
        break;
    }
        
    
    default:
        break;
    } //switch 



return stage;
} // control system

void pcontroller(const mjModel* m, mjData* d)
{
//   cout << m->nu << m->nv <<endl;
//   if( m->nu==m->nv )
//   std::cout << "equal"<<endl;
  double ref[] = {0.470887, 0.978366, 2.6895, -2.15875, 2.75034, 1.88727, 0.707};
  // only one position due to the constriant from callback
    // d->ctrl[0] = -10*(d->qpos[0]-0)-1*d->qvel[0];

   int n_joints = m->nu;
    float kp = 0.003f;
    float kv = 0.01f;
    double err = 0.0;
    for(int i = 0; i < n_joints; i++)
    {
        err = ref[i] - d->qpos[i];
        d->ctrl[i] = d->ctrl[i] + kp*(err) -kv * d->qvel[i];
    }

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

void simulation (mjModel* model, mjData* data, int argc, const char** argv)
{
// create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1400, 900, "Robo arm hand simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    opt.flags[mjVIS_CONTACTFORCE] = 1;
    // opt.flags[mjVIS_CONTACTSPLIT] = true;
    std::cout<<opt.flags[mjVIS_CONTACTFORCE]<<"true"<<std::endl;
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

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
    double q7 = 1.207;
    std::array<double, 16> O_T_EE_array;
    std::array<double, 7> q_actual_array = {0};
    for (auto i = 0; i < 7; i++)
        q_actual_array[i] = d->qpos[i];

    O_T_EE_array[0] = 0; O_T_EE_array[4] = -0.5; O_T_EE_array[8] = 0.867;    O_T_EE_array[12] = 0.75;
    O_T_EE_array[1] = 0; O_T_EE_array[5] = 0.867; O_T_EE_array[9] = 0.5;    O_T_EE_array[13] = -0.1;
    O_T_EE_array[2] = 1; O_T_EE_array[6] = 0; O_T_EE_array[10] = 0;  O_T_EE_array[14] = 0.65;
    O_T_EE_array[3] = 0; O_T_EE_array[7] = 0; O_T_EE_array[11] = 0;   O_T_EE_array[15] = 1;      


    Ik_solution ik_sol;
    ik_sol.define_sol_par(O_T_EE_array, q_actual_array, q7);
    ik_sol.get_Solution();
    ik_sol.print_sol();
    /////////////////////////////////////////////////////////////////////////////////////////////


    int ncon = data->ncon; // Number of contacts

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


    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    //mjcb_control = controlLaw;              // initialize a control law    

    double desired_real_time_factor = 1.0;  // Adjust this factor as needed
    double normal_dt = 1.0 / 60.0;  // 
    double dt = normal_dt / desired_real_time_factor; // 60 Hz for control system and rendering
    // m->opt.timestep = dt;  // the integral time in each step. should align with the step size defined earlier. 
    cout<<"the intergal time is " << model->opt.timestep<< endl;
    

    // for P controller the calling frequency influence the controled behavior
    double controlSystem_dt = 1.0 / 60.0;  
    double lastControlUpdateTime = 0.0;
    // mjcb_control = pcontroller;
    mjtNum start_time = data->time;;
    int stage = 0; // flag to tell which stage it is right now.
    DataHandler datahandler;
    datahandler.openData();

    const char* objectName = "Small_Proximal";
    int objectID = mj_name2id(m, mjOBJ_BODY, objectName);
    const char* fingerTipName = "Middle_Distal--Middle_Middle";
    const int indexTipID = mj_name2id(m, mjOBJ_JOINT, fingerTipName);
    mjtNum tip_index_force[6];
    int tip_index_ncon{0};
    int object_ncon{0};
    int bodyID{-1};
    
    const char* tableName = "Table";
    const int tableID = mj_name2id(m, mjOBJ_BODY tableName);
    int table_ncon{0};
    

    while( !glfwWindowShouldClose(window) )
    {        
        
        
        

        mjtNum simstart = data->time;
        
        // while last step is finished and a frame is rendered,, keep do the simulation without re-rendering.
        while (data->time - simstart < dt) {
        mj_step(model, data);
        }

        for (int i = 0; i < d->ncon; i++) {//loop over all contacts. 
            // std::cout<< "index:" <<i << "\n";
                int body1 = m->geom_bodyid[d->contact[i].geom1];
                int body2 = m->geom_bodyid[d->contact[i].geom2];
            // std::cout<< objectID << " ";
            // std::cout<< body1 <<" ";
            // std::cout<< body2 << "\n";


            if (body1 == objectID || body2 == objectID) { 
                
                object_ncon++;

                body1 == objectID ? bodyID = body2 : bodyID = body1; // Corrected assignment operator
                // First, try only focus on the finger tip, also count the number of contacts on each body
                // Index finger 
                // std::cout<< "bodyID is " <<bodyID <<"and indexTipID is " << indexTipID <<std::endl;
                if (bodyID == indexTipID) {
                    mj_contactForce(m, d, i, tip_index_force);
                    tip_index_ncon++;
                    std::cout << "Contact Force as a Vector: ["
                        << tip_index_force[0] << ", "
                        << tip_index_force[1] << ", "
                        << tip_index_force[2] << "]" << " Number " << tip_index_ncon << std::endl;
                }

                std::cout<<"ID of table is " << tableID << " body is" << bodyID <<std::endl;
                if (bodyID == tableID){
                    mj_contactForce(m, d, i, tip_index_force);
                    table_ncon++;
                    std::cout << "Contact Force with the table as a Vector: ["
                        << tip_index_force[0] << ", "
                        << tip_index_force[1] << ", "
                        << tip_index_force[2] << "]" << " Number " << table_ncon << std::endl;
                }
            }
        }



        //         switch (bodyID) {
        //             case indexTipID:
        //                 mj_contactForce(m, d, i, tip_index_force);
        //                 tip_index_ncon++;
        //                 std::cout << "Contact Force as a Vector: ["
        //                         << tip_index_force[0] << ", "
        //                         << tip_index_force[1] << ", "
        //                         << tip_index_force[2] << "]" << " Number " << tip_index_ncon << std::endl;
        //                 break;
        //             default:
        //                 break;
        //         }
        //     }
        // }


        

        mjtNum elapsed_time = data->time - start_time;//set after the while loop to decrease the influence from.
        //steady the calling frequency of controllSystem.
        if (elapsed_time - lastControlUpdateTime >= controlSystem_dt){
            
            // mj_step(m,d);
            // mj_step1(m, d);
            stage = controlSystem(model, data, ref, stage);
            datahandler.record_contact(model,data);
            // std::cout<<m->nsensordata<<std::endl;

            // for (int i = 0; i < m->nsensordata; ++i) {
            //     std::cout<<data->sensordata[i]<<std::endl;
            //     }


    //         for (int i = 0; i < d->ncon; ++i) {
    // mjContact* contact = &d->contact[i];
    
    // Example of printing contact position (assuming mjContact has these fields)
    // std::cout << "Contact " << i << ": Position = ("
    //           << contact->pos[0] << ", "
    //           << contact->pos[1] << ", "
    //           << contact->pos[2] << ")\n"
    //           <<  contact->friction[0] << endl;

             
 


       
        


        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
        }
    
    } // end simulation

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);


}
void depth_show (mjModel* model, mjData* data, int argc, const char** argv)
{
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(1200, 800, "Camera", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

    // mjv_defaultCamera(&cam2);
  mjvOption sensor_option;
  mjvPerturb sensor_perturb;
  mjvScene sensor_scene;
  mjrContext sensor_context;

    mjvCamera rgbd_camera;
  rgbd_camera.type = mjCAMERA_FIXED;
  rgbd_camera.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "camera");


  mjv_defaultOption(&sensor_option);
//   sensor_option.flags[mjVIS_CONTACTFORCE] = 1;
// std::cout << "mjVIS_CONTACTFORCE flag is: " << (opt.flags[mjVIS_CONTACTFORCE] ? "true" : "false") << std::endl;

  mjv_defaultScene(&sensor_scene);
  mjr_defaultContext(&sensor_context);

  // create scene and context
  mjv_makeScene(model, &sensor_scene, 1000);
  mjr_makeContext(model, &sensor_context, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

//   RGBD_mujoco mj_RGBD;

  while (!glfwWindowShouldClose(window))
  {
    // get framebuffer viewport
    mjrRect viewport = {0,0,0,0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    

    mjv_updateScene(model, data, &sensor_option, NULL, &rgbd_camera, mjCAT_ALL, &sensor_scene);
    mjr_render(viewport, &sensor_scene, &sensor_context);

   int total = viewport.width*viewport.height;
    std::unique_ptr<float []> depth(new float[total]);
    
  mjr_readPixels(nullptr, depth.get(), viewport, &sensor_context);
  // convert to meters
  float extent = model->stat.extent;

float near = model->vis.map.znear * extent;
  float far = model->vis.map.zfar * extent;
 for (int i=0; i<total; ++i) {
    depth[i] = near / (1.0f - depth[i] * (1.0f - near/far));
  }

  // convert to a 3-channel 8-bit image
  std::unique_ptr<unsigned char[]> depth8(new unsigned char[3*viewport.width*viewport.height]);
  for (int i=0; i<viewport.width*viewport.height; i++) {
    depth8[3*i] = depth8[3*i+1] = depth8[3*i+2] = depth[i] * 255;
  }

  mjr_drawPixels(depth8.get(), nullptr, viewport, &sensor_context);

    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));


  }

  mjv_freeScene(&sensor_scene);
  mjr_freeContext(&sensor_context);

}
void second_view (mjModel* model, mjData* data, int argc, const char** argv)
{
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(1200, 800, "Camera", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);


  mjvOption sensor_option;
  mjvPerturb sensor_perturb;
  mjvScene sensor_scene;
  mjrContext sensor_context;

  mjv_defaultOption(&sensor_option);
  mjv_defaultScene(&sensor_scene);
  mjr_defaultContext(&sensor_context);

  // create scene and context
  mjv_makeScene(model, &sensor_scene, 1000);
  mjr_makeContext(model, &sensor_context, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

//   Glfw rendering
  while (!glfwWindowShouldClose(window))
  {
    // get framebuffer viewport
    mjrRect viewport = {0,0,0,0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    

    mjv_updateScene(model, data, &sensor_option, NULL, &cam, mjCAT_ALL, &sensor_scene);
    mjr_render(viewport, &sensor_scene, &sensor_context);


    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));


  }

  mjv_freeScene(&sensor_scene);
  mjr_freeContext(&sensor_context);

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
 // calling thread form simulaiton and depth camera       
  std::thread simulation_thread(simulation, m, d,argc, argv);
    // std::thread view_thread(second_view, m, d,argc, argv);
    // std::thread depth_thread(depth_show, m, d,argc, argv);

  simulation_thread.join();
//   view_thread.join();
    // depth_thread.join();


    
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