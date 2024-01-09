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
#include "data_handler.h"
#include "array_safety.h"
#include "ControlSystem.h"

#define PI 3.14159265358979323846

// using namespace std;
using Eigen::MatrixXd;

// MuJoCo data structures
mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvCamera cam2;
mjvOption opt;  // visualization options
mjvScene scn;   // abstract scene
mjrContext con; // custom GPU context
mjrContext con_hint;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

char filename[] = "../franka_panda_RH8D_R.xml";
int stage = 0; // flag to for control stage. Gloabl
// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
        stage = 0;
    }
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
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
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void simulation(mjModel *model, mjData *data, int argc, const char **argv)
{
    ControlSystem controlSystem(model);
    // create window, make OpenGL context current, request v-sync
    GLFWwindow *window = glfwCreateWindow(1400, 900, "Robo arm hand simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    opt.flags[mjVIS_CONTACTFORCE] = 1;
    // opt.flags[mjVIS_CONTACTSPLIT] = true;
    // std::cout<<opt.flags[mjVIS_CONTACTFORCE]<<"true"<<std::endl;
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    // position of default free camera.
    double arr_view[] = {86.68, -9.65863, 2.54165, 0.171193, 0.0378619, 0.61732};
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

    // cout << "About to plot your results" << endl;
    Log("Loading worked correctly, plotting simulation now");

    int n_joints = 26;
    for (int i = 0; i < n_joints; i++)
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

    O_T_EE_array[0] = 0;
    O_T_EE_array[4] = -0.5;
    O_T_EE_array[8] = 0.867;
    O_T_EE_array[12] = 0.75;
    O_T_EE_array[1] = 0;
    O_T_EE_array[5] = 0.867;
    O_T_EE_array[9] = 0.5;
    O_T_EE_array[13] = -0.1;
    O_T_EE_array[2] = 1;
    O_T_EE_array[6] = 0;
    O_T_EE_array[10] = 0;
    O_T_EE_array[14] = 0.65;
    O_T_EE_array[3] = 0;
    O_T_EE_array[7] = 0;
    O_T_EE_array[11] = 0;
    O_T_EE_array[15] = 1;

    Ik_solution ik_sol;
    ik_sol.define_sol_par(O_T_EE_array, q_actual_array, q7);
    ik_sol.get_Solution();
    ik_sol.print_sol();
    /////////////////////////////////////////////////////////////////////////////////////////////

    int ncon = data->ncon; // Number of contacts

    double ref[n_joints] = {-1.42172, -1.18835, 0.977646, -2.34855, 0.884779, 1.46295, 0, // Panda joints
                            0,                                                            // Forearm --> Should be kept in 0 as it is not actuated
                            0,                                                            // Wrist adduction/abduction
                            0.50,                                                         // Wrist Flexion/Extension
                            1.5,                                                          // Thumb MCP Adduction
                            0.5,                                                          // Thumb MCP Flexion
                            0.70,                                                         // Thumb PIP Flexion
                            0.70,                                                         // Thumn DIP Flexion
                            1.5,                                                          // Index MCP Flexion
                            0.70,                                                         // Index PIP Flexion
                            0.70,                                                         // Index DIP Flexion
                            1.5,                                                          // Middle MCP Flexion
                            0.70,                                                         // MIddle PIP Flexion
                            0.70,                                                         // MIddle DIP Flexion
                            1.50,                                                         // Ring MCP flexion
                            0.70,                                                         // Ring PIP Flexion
                            0.70,                                                         // Ring DIP Flexion
                            1.50,                                                         // Little MCP Flexion
                            0.70,                                                         // Little PIP Flexion
                            0.70};                                                        // Little DIP Flexion

    for (int i = 0; i < 7; i++)
        ref[i] = ik_sol.q_sol[i];

    // run main loop, target real-time simulation and 60 fps rendering
    double fps = 60.0;
    std::sscanf(argv[3], "%lf", &fps);
    // std::cout << fps <<"\n" ;

    // get framebuffer viewport
    mjrRect viewport = mjr_maxViewport(&con);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    // mjcb_control = controlLaw;              // initialize a control law

    double desired_real_time_factor = 1.0;            // Adjust this factor as needed
    double normal_dt = 1.0 / fps;                     //
    double dt = normal_dt / desired_real_time_factor; // 60 Hz for control system and rendering
    // m->opt.timestep = dt;  // the integral time in each step. should align with the step size defined earlier.
    std::cout << "the intergal time is " << model->opt.timestep << std::endl;

    // for P controller the calling frequency influence the controled behavior
    double controlSystem_dt = 1.0 / 100.0;
    double lastControlUpdateTime = 0.0;
    // mjcb_control = pcontroller;

    DataHandler datahandler;
    datahandler.openData();

    const char *objectName = "Multi_shaped_object";
    int objectID = mj_name2id(m, mjOBJ_BODY, objectName);
    mjtNum tip_index_force[6];
    int tip_index_ncon{0};
    int object_ncon{0};
    int bodyID{-1};
    int fingerID{-1};

    const char *tableName = "Table";
    const int tableID = mj_name2id(m, mjOBJ_BODY, tableName);
    int table_ncon{0};

    mjtNum start_time = data->time;
    while (!glfwWindowShouldClose(window))
    {

        mjtNum simstart = data->time;

        // while last step is finished and a frame is rendered,, keep do the simulation without re-rendering.
        while (data->time - simstart < controlSystem_dt)
        {
            mj_step(model, data);
        }

        // get contact force
        for (int i = 0; i < d->ncon; i++) // loop over all contacts.
        {
            int body1 = m->geom_bodyid[d->contact[i].geom1];
            int body2 = m->geom_bodyid[d->contact[i].geom2];

            if (body1 == objectID || body2 == objectID)
            {
                body1 == objectID ? fingerID = body2 : fingerID = body1; // other objcet that have contact with object.

                if (fingerID == tableID)
                    break; // not recording the contact force with the table.
                else
                { // recording other contact, perhaps with hand
                    mj_contactForce(m, d, i, tip_index_force);
                    // Extract 6D force:torque given contact id, in the contact frame.
                    tip_index_ncon++;
                    // Write contents to the file, the contact force.
                    datahandler.record_contact(m, d, tip_index_force, fingerID);
                }
            }

        } // end of for loop, checking every contact force.

        // mjtNum elapsed_time = data->time - start_time;//set after the while loop to decrease the influence from.
        // steady the calling frequency of controllSystem.
        // if (elapsed_time - lastControlUpdateTime >= controlSystem_dt)
        // {
        stage = controlSystem.update(data, ref, stage);
        // }

        if (data->time - lastControlUpdateTime >= dt ||lastControlUpdateTime ==0 )

        {
            std::cout << data->time << "\n";
            // update scene and render
            mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
            lastControlUpdateTime = data->time;
        }

    } // end simulation

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
}

void depth_show(mjModel *model, mjData *data, int argc, const char **argv, bool depth_window)
{
    int width = 300;  // Default width
    int height = 300; // Default height

    std::FILE *fp = nullptr; // Declare and initialize fp_depth
    // std::cout << argc << std::endl;
    if (argc > 5)
    {
        std::cout << "new out \n";
        width = std::atoi(argv[6]);  // Convert argv[6] to integer and assign to width
        height = std::atoi(argv[7]); // Convert argv[7] to integer and assign to height
    }
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(width, height, "Camera", NULL, NULL);
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

    //   RGBD_mujoco mj_RGBD;
    // get framebuffer viewport
    // mjrRect viewport = {0,0,0,0};
    mjrRect viewport = mjr_maxViewport(&sensor_context);
    int W = viewport.width;
    int H = viewport.height;

    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    int total = viewport.width * viewport.height;
    std::unique_ptr<float[]> depth(new float[total]);
    std::unique_ptr<unsigned char[]> depth8(new unsigned char[3 * viewport.width * viewport.height]);

    double fps = 30;
    std::sscanf(argv[3], "%lf", &fps);
    // create output rgb file

    fp = std::fopen(argv[5], "wb");
    if (!fp)
    {
        mju_error("Could not open rgbfile for writing");
    }

    double frametime = 0;
    int framecount = 0;
    namespace mju = ::mujoco::sample_util;
    while (!glfwWindowShouldClose(window))
    {
        if ((d->time - frametime) > 1 / fps || frametime == 0)
        {
            // if ((d->time-frametime)>1/fps || frametime==0) {
            mjv_updateScene(model, data, &sensor_option, NULL, &rgbd_camera, mjCAT_ALL, &sensor_scene);
            mjr_render(viewport, &sensor_scene, &sensor_context);

            // mjr_readPixels(nullptr, depth.get(), viewport, &sensor_context);

            // convert to meters
            float extent = model->stat.extent;
            float near = model->vis.map.znear * extent;
            float far = model->vis.map.zfar * extent;
            for (int i = 0; i < total; ++i)
            {
                depth[i] = near / (1.0f - depth[i] * (1.0f - near / far));
            }

            // convert to a 3-channel 8-bit image
            mjr_readPixels(nullptr, depth.get(), viewport, &sensor_context);
            for (int i = 0; i < viewport.width * viewport.height; i++)
            {
                depth8[3 * i] = depth8[3 * i + 1] = depth8[3 * i + 2] = depth[i] * 255;
            }

            mjr_drawPixels(depth8.get(), nullptr, viewport, &sensor_context);
            // add time stamp in upper-left corner
            char stamp[50];
            mju::sprintf_arr(stamp, "Time = %.3f", d->time);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &sensor_context);
            if (depth_window)
            {
                std::fwrite(depth8.get(), 3, W * H, fp);
            }
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
            frametime = d->time;

            // } end of if, frequenze of rendering. comment because, the render frequeze
            // is defined inthe simulation loop(doubt), through controw the glbw updating.
        }
    }

    mjv_freeScene(&sensor_scene);
    mjr_freeContext(&sensor_context);
}

void second_view(mjModel *model, mjData *data, int argc, const char **argv, bool depth_window)
{
    int width = 300;               // Default width
    int height = 300;              // Default height
    std::FILE *fp_depth = nullptr; // Declare and initialize fp_depth
    if (argc > 5)                  // input window size
    {
        width = std::atoi(argv[6]);  // Convert argv[6] to integer and assign to width
        height = std::atoi(argv[7]); // Convert argv[7] to integer and assign to height
    }
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(width, height, "Camera_rgb", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // mjv_defaultCamera(&cam2);
    mjvOption sensor_option;
    mjvPerturb sensor_perturb;
    mjvScene sensor_scene;
    mjrContext sensor_context;

    mjvCamera rgbd_camera; // link to the camera setted in xml file.
    rgbd_camera.type = mjCAMERA_FIXED;
    rgbd_camera.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "camera");

    mjv_defaultOption(&sensor_option);
    mjv_defaultScene(&sensor_scene);
    mjr_defaultContext(&sensor_context);

    // create scene and context
    mjv_makeScene(model, &sensor_scene, 1000);
    mjr_makeContext(model, &sensor_context, mjFONTSCALE_150);

    // get framebuffer viewport
    mjrRect viewport = mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    int total = viewport.width * viewport.height;
    std::unique_ptr<float[]> depth(new float[total]);
    std::unique_ptr<unsigned char[]> rgbBuffer(new unsigned char[total * 3]);
    std::unique_ptr<unsigned char[]> depth8(new unsigned char[3 * viewport.width * viewport.height]);

    double duration = 10, fps = 30;
    std::sscanf(argv[3], "%lf", &fps);
    // create output rgb file
    std::FILE *fp_rgb = std::fopen(argv[4], "wb"); // file for rgb image
    if (!fp_rgb)
    {
        mju_error("Could not open rgb file for writing");
    }

    if (!depth_window) // depth window closed, gernate depth image here.
    {
        fp_depth = std::fopen(argv[5], "wb"); // file for depth image
        if (!fp_depth)
        {
            mju_error("Could not open deth file for writing");
        }
    }

    double frametime = 0; // time of last rendering
    namespace mju = ::mujoco::sample_util;
    while (!glfwWindowShouldClose(window))
    {
        // render new frame if it is time (or first frame)
        if ((d->time - frametime) > 1 / fps || frametime == 0)
        {
            mjv_updateScene(model, data, &sensor_option, NULL, &rgbd_camera, mjCAT_ALL, &sensor_scene);
            mjr_render(viewport, &sensor_scene, &sensor_context);

            if (!depth_window) // depth window closed, gernate depth image here.
            {
                // calculat and wirte depth image to file
                std::cout << "depth image from second view\n";
                // convert to meters
                float extent = model->stat.extent;
                float near = model->vis.map.znear * extent;
                float far = model->vis.map.zfar * extent;
                for (int i = 0; i < total; ++i)
                {
                    depth[i] = near / (1.0f - depth[i] * (1.0f - near / far));
                }
                // convert to a 3-channel 8-bit image
                mjr_readPixels(nullptr, depth.get(), viewport, &sensor_context);
                for (int i = 0; i < viewport.width * viewport.height; i++)
                {
                    depth8[3 * i] = depth8[3 * i + 1] = depth8[3 * i + 2] = depth[i] * 255;
                }
                std::fwrite(depth8.get(), 3, W * H, fp_depth);
            }

            // add time stamp in upper-left corner
            char stamp[50];
            mju::sprintf_arr(stamp, "Time = %.3f", d->time);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &sensor_context);

            mjr_readPixels(rgbBuffer.get(), nullptr, viewport, &sensor_context);

            // write rgb image to file
            std::fwrite(rgbBuffer.get(), 3, W * H, fp_rgb);

            mjr_drawPixels(rgbBuffer.get(), nullptr, viewport, &sensor_context);

            glfwSwapBuffers(window);
            frametime = d->time;
        }
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    mjv_freeScene(&sensor_scene);
    mjr_freeContext(&sensor_context);
}
// main function
int main(int argc, const char **argv)
{
    // std::cout <<argv[8] << "  " << argv[9] <<"\n" ;
    std::cout << argc << "\n";
    bool depth_window = false;
    // check command-line arguments
    if (argc > 8)
    {
        std::string arg(argv[8]);
        if (arg == "true" || arg == "1")
        {
            depth_window = true;
        }
    }

    if (argc == 0)
    {
        printf(" No arguments passed. Loading model...\n");
    }

    // load and compile model
    char error[1000] = "Could not load binary model";

    if (argc < 2)
    {
        m = mj_loadXML(filename, 0, error, 1000);
    }
    else
    {
        if (strlen(argv[1]) > 4 && !strcmp(argv[1] + strlen(argv[1]) - 4, ".mjb"))
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    }

    if (!m)
        mju_error_s("Load model error: %s", error);
    // depth_window = (argc > 8) ? std::atoi(argv[8]) : depth_window;

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    // calling thread form simulaiton and depth camera
    std::thread simulation_thread(simulation, m, d, argc, argv);
    std::thread view_thread(second_view, m, d, argc, argv, depth_window);
    std::thread depth_thread;
    // std::thread offscreen_thread(offscreen_rgb, m, d,argc, argv);
    if (depth_window)
    {
        std::cout << "main depth";
        depth_thread = std::thread(depth_show, m, d, argc, argv, depth_window);
    }
    simulation_thread.join();
    view_thread.join();
    if (depth_thread.joinable())
    {
        depth_thread.join();
    }

    // offscreen_thread.join();

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);

// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
    // delete[] corr_sys;
    return 1;
}