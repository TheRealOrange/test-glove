//
// Created by orange on 2/9/21.
//

#ifndef TEST_GLOVE_DATACAPTUREAPP_H
#define TEST_GLOVE_DATACAPTUREAPP_H


#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Surface.h"
#include "cinder/Capture.h"
#include "cinder/Camera.h"
#include "cinder/Text.h"
#include "cinder/Log.h"

#include "cinder/audio/Utilities.h"

#include "GloveData.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class DataCaptureApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

    glm::vec3 handPos = glm::vec3();
    double jointAngles[5][5] = {{0.0, 0.0, 0.0, 0.0, 0.0},
                             {0.0, 0.0, 0.0, 90.0, 90.0},
                             {0.0, 0.0, 0.0, 90.0, 90.0},
                             {0.0, 0.0, 0.0, 90.0, 90.0},
                             {0.0, 0.0, 0.0, 90.0, 90.0}};
    // thumb    cmc mcp < > dip
    // index    cmc mcp pip dip
    // middle   cmc mcp pip dip
    // ring     cmc mcp pip dip
    // pinky    cmc mcp pip dip


private:
    void setupGraphics();
    void drawBackground();
    void drawDebug();
    void drawInfo();

    bool					mDrawDebug, mDrawInfo;
    float					mFps;
    float					mMasterGain;
    CameraPersp		mCam;
    mat4			mRotation;

    gl::BatchRef            mPalm;
    gl::BatchRef            mThumb1;
    gl::BatchRef            mThumb2;
    gl::BatchRef            mIndex1;
    gl::BatchRef            mIndex2;
    gl::BatchRef            mIndex3;
    gl::BatchRef            mMiddle1;
    gl::BatchRef            mMiddle2;
    gl::BatchRef            mMiddle3;
    gl::BatchRef            mRing1;
    gl::BatchRef            mRing2;
    gl::BatchRef            mRing3;
    gl::BatchRef            mPinky1;
    gl::BatchRef            mPinky2;
    gl::BatchRef            mPinky3;

    GloveData glove;

    float THUMB1 = 4.0;
    float THUMB2 = 3.0;
    float LENGTH1 = 4.0;
    float LENGTH2 = 4.0;
    float LENGTH3 = 3.0;
    float FINGERW = 2.5;
    float FINGERH = 2.0;

    vec3 camPos = vec3(0.0f, 25.0f, 40.0f);
    vec3 camAngle = vec3(0);
};


#endif //TEST_GLOVE_DATACAPTUREAPP_H
