//
// Created by orange on 2/9/21.
//

#ifndef TEST_GLOVE_DATACAPTUREAPP_H
#define TEST_GLOVE_DATACAPTUREAPP_H


#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/Timeline.h"
#include "cinder/Rand.h"
#include "cinder/Text.h"
#include "cinder/CinderImGui.h"

#include "cinder/audio/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class DataCaptureApp : public App {
public:
    void setup() override;
    void update() override;
    void draw() override;

private:
    void setupGraphics();
    void drawBackground();
    void drawDebug();
    void drawInfo();

    bool					mDrawDebug, mDrawInfo;
    float					mFps;
    float					mMasterGain;
};


#endif //TEST_GLOVE_DATACAPTUREAPP_H
