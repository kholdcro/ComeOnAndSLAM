/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    int mImageHeight_bar = int(mImageHeight);
    int pangoWidth = int(mImageWidth);
    bool useBlack = false;
    cv::Mat bottomCorn;

    if (mImageHeight*mImageWidth > 1000*1000)
    {
        mImageHeight_bar = mImageHeight/2;
        pangoWidth = mImageWidth/2;
        useBlack = true;
        
        bottomCorn = cv::imread("basketball.jpg", cv::IMREAD_COLOR); // Read the file
        if( bottomCorn.empty() )                      // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
            cv::Mat black(mImageHeight_bar, pangoWidth, CV_8UC3, cv::Scalar(0,0,0));
            bottomCorn = black;
        }
        else
        {
            cv:: resize(bottomCorn, bottomCorn, cv::Size(pangoWidth, mImageHeight_bar));
            cout << "Image Found!" << endl;
        }
    }
    else
    {
        cv::Mat black;
    }


    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",pangoWidth,mImageHeight_bar+20);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(pangoWidth,mImageHeight_bar,mViewpointF,mViewpointF,pangoWidth/2,mImageHeight_bar/2,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -float(pangoWidth)/float(mImageHeight_bar+20))
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    // cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

    // Save Video ---------------------------------------------------------------------------------
    cv::VideoWriter video("sampleViewer.avi",CV_FOURCC('M','J','P','G'),1e3/mT, cv::Size(mImageWidth+pangoWidth,mImageHeight+20),true);
    // Save Video ---------------------------------------------------------------------------------
    unsigned long int f = 0;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        // Get CV matrix from image
        pangolin::Image<unsigned char> buffer;
        pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("RGBA32");
        buffer.Alloc(d_cam.v.w, d_cam.v.h, d_cam.v.w * fmt.bpp/8 );
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(d_cam.v.l, d_cam.v.b, d_cam.v.w, d_cam.v.h, GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr );
        cv::Mat pointImg, imgBuffer = cv::Mat(d_cam.v.h, d_cam.v.w, CV_8UC4, buffer.ptr);
        cv::cvtColor(imgBuffer, pointImg,  cv::COLOR_RGBA2BGR);

        //Clean Things
        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::Mat combi;
        cv:: Mat flipPoint;
        cv::flip(pointImg, flipPoint, 0);

        if(f != mpTracker->mCurrentFrame.mnId)
        {
            f = mpTracker->mCurrentFrame.mnId;

            if(im.rows != mImageHeight+20)
            {
                 cv::Mat temp(mImageHeight+20, mImageWidth, CV_8UC3, cv::Scalar(0,0,0));
                 im = temp;
                 cout << "Image not rendered" << endl;
            }

            if(useBlack)
            {
                cv::Mat tmp;
                cv::vconcat(flipPoint,bottomCorn,tmp);
                hconcat(tmp,im,combi);
            }
            else
            {
                hconcat(flipPoint,im,combi);
            }

            // cv::imshow("ORB-SLAM2: Current Frame",im);

            video.write(combi);
        }  

        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
