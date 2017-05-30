//----------------------------------------------------------------------------------------------------
// Implementation of TCM Cumulative Fatigue Model and Biomechanical Arm Analysis
// Author: Sujin Jang, Purdue University, Oct. 2016.
// "cdUserData" class is designed to load and svae user interaction data including body tracking
// and interaction event.
// Please cite the following publication for public use:
// DOI: https://doi.org/10.1145/3025453.3025523
// Project Webpage: https://engineering.purdue.edu/cdesign/wp/modeling-cumulative-arm-fatigue/
//----------------------------------------------------------------------------------------------------

#ifndef CDUSERDATA_H
#define CDUSERDATA_H

#include <iostream>
#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

typedef struct dataInstance // interaction data structure for an individual user
{
    int id;  // user ID
    int nPoints; // number of points forming this motion data
    vector <int> time; // array of marked time
    vector <float*> pointList; // array of tracking points
    vector <int> event; // array of event log
}userData;

class cdUserData
{
protected:
    bool m_isOpened;
    char m_taskName[100]; // task_name (description)
    int m_taskID; // optional
    int m_nUsers;  // number of users (data instance)
    int m_nTrackedPoints; // number of tracked points (pointer=1, upper-body=11, hand skeleton=5)
    int m_nEventSequence; // number of occurred event sequences (N/A)
    int m_nDimensions;  // dimension of motion inputs 3:[X, Y, Z] or 2:[X, Y]
    map <int, userData> m_InteractDataList; // list of user interaction data (input)
    
public:
    cdUserData();
    ~cdUserData();
    bool OpenDataFile(const char *file_path);
    int GetNpoints(int userId);
    float* GetCoordinate(int userId, int pointIdx = 0); // return a set of coordinates clustered by event sequence

    int GetNUsers(){return m_nUsers;}
    int GetNTrackedPoints(){return m_nTrackedPoints;}
    int GetNDimensions(){return m_nDimensions;}
    int GetUserID(int idx);
    int GetTimeStamp(int userId, int pointIdx);
    int GetEvent(int userId, int pointIdx);
    int GetTaskID(){return m_taskID;}
};

#endif // CDUSERDATA_H

