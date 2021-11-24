//
// Created by Mel on 10/28/2021.
//

#include <iostream>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// #include "example.hpp"          // Include short list of convenience functions for rendering
#include "../includes/RealSense.h"
#include "GLFW/glfw3.h"

#include <algorithm>            // std::min, std::max

#define WIDTH_COLOR_FRAME   1280
#define HEIGTH_COLOR_FRAME  720

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char * argv[]) try
{

    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    // cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH_COLOR_FRAME, HEIGTH_COLOR_FRAME, RS2_FORMAT_RGB8, 15);
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    while (app) // Application still alive?
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();


        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Draw the pointcloud
        // draw_pointcloud(app.width(), app.height(), app_state, points);
        // if (!points)
        // return;

        // OpenGL commands that prep screen for the pointcloud
        glLoadIdentity();
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
        glClear(GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        gluPerspective(60, app.width() / app.height(), 0.01f, 10.0f);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

        glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
        glRotated(app_state.pitch, 1, 0, 0);
        glRotated(app_state.yaw, 0, 1, 0);
        glTranslatef(0, 0, -0.5f);

        glPointSize(app.width() / 640);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, app_state.tex.get_gl_handle());
        float tex_border_color[] = { 0.8f, 0.8f, 0.8f, 0.8f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, tex_border_color);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, 0x812F); // GL_CLAMP_TO_EDGE
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, 0x812F); // GL_CLAMP_TO_EDGE
        glBegin(GL_POINTS);


        /* this segment actually prints the pointcloud */
        auto vertices = points.get_vertices();              // get vertices
        auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
        auto ptr = (uint8_t*)color.get_data();
        auto stride = color.as<rs2::video_frame>().get_stride_in_bytes();
        // std::cout << stride;
        int j = 640; int k = 360;
        /*std::cout << "  R= " << int(ptr[k * stride + (3 * j)]);
        std::cout << ", G= " << int(ptr[k * stride + (3 * j) + 1]);
        std::cout << ", B= " << int(ptr[k * stride + (3 * j) + 2]) << std::endl;*/
        float maxu = 0;
        float maxv = 0;
        float minu = 0;
        float minv = 0;
        for (int i = 0; i < points.size(); i++)
        {
            int u = fmod(tex_coords[i].u, 1) * app.width();
            int v = fmod(tex_coords[i].v, 1) * app.height();
            if (v >= 0) {
                int R = int(ptr[v * stride + (3 * u)]);
                int G = int(ptr[v * stride + (3 * u) + 1]);
                int B = int(ptr[v * stride + (3 * u) + 2]);
                if (vertices[i].z && R > 120 && G < 50 && B < 50) {
                    glVertex3fv(vertices[i]);
                    glTexCoord2fv(tex_coords[i]);
                }
            }
            if (u > maxu) {
                maxu = u;
            }
            if (v > maxv) {
                maxv = v;
            }
            if (u < minu) {
                minu = u;
            }
            if (v < minv) {
                minv = v;
            }
            //glVertex3fv(vertices[i]);
            //glTexCoord2fv(tex_coords[i]);
        }
        std::cout << app.width() << std::endl;
        std::cout << "maxu=" << maxu << ", maxv=" << maxv << std::endl;
        std::cout << "minu=" << minu << ", minv=" << minv << std::endl;

        // OpenGL cleanup
        glEnd();
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glPopAttrib();
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
