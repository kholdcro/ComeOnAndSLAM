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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps, int subinterval);
void saveTrackTimes(const string &filename);

int main(int argc, char **argv)
{
    if(argc <= 5)
    {
        cerr << endl << "Usage: ./widefisheye path_to_vocabulary path_to_settings path_to_sequence useFisheye(bool) subframes" << endl;
        return 1;
    }

    int subinterval;
    if(argc <= 6)
        subinterval = atoi(argv[5]);// - char()'0';
    else
        subinterval = 1;

    cout << "Interval" << subinterval << endl;

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    cout << "derp1" << endl;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps, subinterval);
    cout << "derp2" << endl;
    int nImages = vstrImageFilenames.size();

    ORB_SLAM2::System::eSensor sensor;
    if(string(argv[4]) == "true")
        sensor = ORB_SLAM2::System::FISHEYE;
    else
        sensor = ORB_SLAM2::System::MONOCULAR;
    cout << "derp3" << endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],sensor,true);
    cout << "derp4" << endl;
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    vector<int> trackState;
    trackState.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // sleep(2000); 

    cout << "Starting!" << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        if(ni % subinterval != 0)
            continue;
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        if(string(argv[4]) == "true")
            SLAM.TrackFisheye(im,tframe);
        else
            SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        trackState[ni] = SLAM.GetTrackingState();

        // cout << "Thing: " << SLAM.GetTotalMapPoints() << endl;

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
        cout << "Frame: " << ni << "\tState: " << trackState[ni] << endl;
    }

    cout << "Exiting SLAM..." << endl;
    // Stop all threads
    SLAM.ShutdownWindows();
    cout << "Success!" << endl << endl;

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << nImages << endl;
    cout << vTimesTrack[97] << endl;
    cout << vTimesTrack[98] << endl;
    cout << vTimesTrack[99] << endl;
    cout << vTimesTrack[100] << endl;
    cout << "Thinger: " << SLAM.GetTotalMapPoints() << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(string(argv[3])+"/KeyFrameTrajectory.txt");

    ofstream g;
    g.open((string(argv[3])+"/stateFrames.txt").c_str());
    for(int ni=0; ni<nImages; ni++)
    {
        g << trackState[ni] << endl;
    }
    g.close();
    
    ofstream f;
    f.open((string(argv[3])+"/trackPercentage.txt").c_str());
    f << fixed;
    f << "median tracking time: " << vTimesTrack[nImages/2] << "\n" << endl;
    f << "mean tracking time: " << totaltime/nImages << "\n" << endl;
    f << "Average tracking time: " << totaltime/vTimestamps[nImages-1] << "\n" << endl;
    f << "Total tracking time: " << totaltime << "\n" << endl;
    f << "Total time: " << vTimestamps[nImages-1] << "\n" << endl;
    f << "Number of images: " << nImages << "\n" << endl;
    f << "Total Map Points: " << SLAM.GetTotalMapPoints() << "\n" << endl; 
    f << "-------";
    f.close();


    cv::Mat orbMatches = SLAM.GetMatches();

    double min, max;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(orbMatches, &min, &max, &minLoc, &maxLoc);
    cout << min << "\t\t\t" << max << endl;
    cout << minLoc.x << " " << minLoc.y << "\t" << maxLoc.x << " " << maxLoc.y << endl;
    // orbMatches = orbMatches/max;    
    cv::imwrite(string(argv[3])+"orbMatches.png", orbMatches);

    // cv::FileStorage fs; 
    // fs.open(string(argv[3])+"orbMatches.xml", cv::FileStorage::WRITE); 
    // cv::Mat orbMatches = SLAM.GetMatches();
    // fs << "orbMatches" << orbMatches;
    // fs.release();


    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps, int subinterval)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "images/image";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        if(i % subinterval != 0)
        {
            continue;
        }
        // stringstream ss;
        // ss << setfill('0') << setw(6) << i;
        // cout << strPrefixLeft + ss.str() + ".jpg" << endl;
        // cout << strPrefixLeft + to_string(i+230) + ".jpg" << endl;
        vstrImageFilenames[i] = strPrefixLeft +  to_string(i) + ".jpg";
        // vstrImageFilenames[i] = strPrefixLeft +  to_string(i) + ".jpg";
    }
}