/**LICENSE LOCATED AT BOTTOM; DO NOT MODIFY THIS**/
/** leonfrickensmith@gmail.com **/
/** IRIS at UIUC 2015 **/

#include <iostream> //cout
#include <pthread.h>//pthreads
#include <vector> //for std::vector

#include "CoordSystemKinect.hpp"//Kinect Input
#include "libfreenect.hpp"//Kinect Input
#include "Linear.hpp"//Mat3
#include "Map.hpp"//Map<T>

using namespace std;
//csk namespace represents CoordinateSystemKinect

static volatile bool main_done = false;
static volatile bool depthCB_done = false;
static volatile bool videoCB_done = false;

static freenect_context* f_ctx;
freenect_device* f_dev;
pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
uint16_t t_gamma[2048];




static Vec3f downDirection(0,0,0);//static to prevent other files from seeing this
static char userChoice = 't';//

/**================================================================================**/
/**DEPTH SENSOR CALLBACK**/
/**================================================================================**/
void depth_cb(freenect_device* pDevice, void* v_depth, uint32_t timestamp)
{
    if(downDirection.z != 0)//make sure we don't take an image with bad accelerometer data
    {
        uint16_t* pDepth = static_cast<uint16_t*>(v_depth);
        const int pointCount = csk::dimX*csk::dimY;
        Map<float> gradient(Vec2i(80,80));
        Map<float> height(Vec2i(80,80));

        height.getPoint(Vec2i(0,0)).value = 9;
        vector<Vec3f> pointCloud;
        pointCloud.resize(csk::dimX*csk::dimY);//make our pointcloud large enough

        /**REMOVE INVALID POINTS FROM DEPTH DATA**/
        for(int i = 0; i<pointCount; ++i)
            if(csk::RawDepthToMilli(pDepth[i]) < 450)
                pDepth[i] = 0;


        /**CREATE CARTESIAN POINT CLOUD**/
        for(int y = 0; y<csk::dimY; ++y)
            for(int x = 0; x<csk::dimX; ++x)
                pointCloud[csk::GetCoord(x,y)] = csk::GetCartCoord(x, y, pDepth);


        /**POINT CLOUD ADJUSTED FOR PITCH AND ROLL**/
        /**NOTE THAT WE CANNOT KNOW YAW (How the kinect is turned in relation to another object)**/
        Mat3f pitchRoll = csk::FindDownMatrix(downDirection);//find the rotation matrix
        for(int i = 0; i<pointCount; ++i)//rotate the point cloud data appropriatly
        {
            pointCloud[i] = pitchRoll*pointCloud[i];
        }


        /**POINT CLOUD UNITS ADJUSTED FOR HUMAN VIEWING**/
        const float unitConvert = 1.0f/50.0f;//half decimeters (50 times larger than a millimeter is half a decimeter)
        for(int i = 0; i<pointCount; ++i)
        {
            pointCloud[i].z *= unitConvert;
            pointCloud[i].y *= unitConvert;
            pointCloud[i].x *= unitConvert;
        }


        /**CONVERT POINT CLOUD INTO HEIGHT MAP**/
        for(int i = 0; i<pointCount; ++i)
        {
            if(height.getPoint(Vec2i(pointCloud[i].x-40, pointCloud[i].y+40)).value < pointCloud[i].z)
                height.getPoint(Vec2i(pointCloud[i].x-40, pointCloud[i].y+40)).value = pointCloud[i].z;
        }

        /**REMOVE STRANGE VALUES FROM MAP, PRINT TO A FILE, PAUSE PROGRAM**/
        height.makeGradient(gradient);
        gradient.minValue = -1;
        gradient.maxValue = 9;
        gradient.nullRep = '-';
        gradient.toFile("gradientMap.txt");
        //height.normalizeMap();
        //height.toFile("heightMap.txt");

        //cin >> userChoice;
    }

    pthread_cond_signal(&gl_frame_cond);
}
/**================================================================================**/
/**VIDEO CALLBACK**/
/**================================================================================**/
void video_cb(freenect_device* pDevice, void* v_video, uint32_t timestamp)
{
    //cast v_video to something better
}


/**================================================================================**/
/**THIS IS THE SECOND THREAD, effectivly a second main()**/
/**================================================================================**/
void* freenect_threadfunc(void* arg)
{
    freenect_video_format requested_format = FREENECT_VIDEO_RGB;
    freenect_video_format current_format = FREENECT_VIDEO_RGB;
    bool enableAccel = true;

    //freenect_set_tilt_degs(f_dev, -22);//set kinect angle
    freenect_set_led(f_dev, static_cast<LED_COLOR>(3));//set kinect LED color, LED_RED, libfreenect.h

    //freenect_set_video_callback(f_dev, video_cb);
    //freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
    //freenect_start_video(f_dev);//tell it to start reading rgb

    freenect_set_depth_callback(f_dev, depth_cb);//set the function that will be called for each depth call
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_start_depth(f_dev);//tell it to start reading depth


    while(!depthCB_done && freenect_process_events(f_ctx) >= 0 && enableAccel)/**this is primary loop for kinect stuff**/
    {
        double dx,dy,dz;
        freenect_raw_tilt_state* pState;
        freenect_update_tilt_state(f_dev);
        pState = freenect_get_tilt_state(f_dev);
        freenect_get_mks_accel(pState, &dx, &dy, &dz);
        downDirection = csk::FindDown(pState->accelerometer_x, pState->accelerometer_y, pState->accelerometer_z);


        //cout << "\nDown:\t" << downDirection.x << "\t" << downDirection.y << "\t" << downDirection.z;

        if(requested_format != current_format)
        {
            freenect_stop_video(f_dev);
            //freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
            freenect_start_video(f_dev);
            current_format = requested_format;
        }
    }

    /**SHUT DOWN STREAMS**/
    freenect_stop_depth(f_dev);
    freenect_stop_video(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    return NULL;
}


/**================================================================================**/
/**=================================  MAIN  =======================================**/
/**================================================================================**/
int main(int argc, char **argv)
{
    pthread_t freenect_thread;

    /**===================================================**/
    /**ALL ABOUT INITIALIZING THE CONNECTION WITH KINECT!!**/
    /**===================================================**/


    if(freenect_init(&f_ctx, NULL) < 0)
    {
        cout << "\nFreenect_init() failed.(1)";
        return 1;
    }
    freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    int nr_devices = freenect_num_devices(f_ctx);
    cout << "\nNumber of devices found: " << nr_devices;
    int user_device_number = 0;
    if(argc > 1)
        user_device_number = atoi(argv[1]);
    if(nr_devices < 1)
    {
        cout << "\nNo devices found.(2)";
        return 2;
    }
    if(freenect_open_device(f_ctx, &f_dev, user_device_number) < 0)
    {
        cout << "\nCould not open device.(3)";
        return 3;
    }
    else
        cout << "\nOpened a device.";



    /**WE MUST HAVE OPENED A DEVICE, SO CREATE A NEW THREAD TO DEAL WITH IT**/
    int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    if(res)
    {
        cout << "\nPThread_create failed.(5)";
        return 4;
    }


    while(not main_done)//this loops while the other threads do things like depth callback
        sleep(10);

    cout << "\nExit Success.(0)";
    return 0;
}
/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */
