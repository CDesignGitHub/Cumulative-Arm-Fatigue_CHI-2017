#include "cdUserData.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <algorithm>

#define PI 3.141592

cdUserData::cdUserData()
{
	m_nUsers=0;  // number of users
	m_nTrackedPoints=0; // number of tracked points (hand=1, upper-body=11)
	m_nEventSequence=0; // number of occurred event sequences
    m_nDimensions=0;  // dimension of input data
}

cdUserData::~cdUserData()
{
    map<int, userData>::iterator iter;
    for(iter=m_InteractDataList.begin(); iter!=m_InteractDataList.end(); iter++)
    {
        userData data = iter->second;
        data.time.clear();
        data.event.clear();
        int pointListSize = data.pointList.size();
        for(int i=0; i<pointListSize; i++)
        {
            delete data.pointList[i];
        }
    }
}

bool cdUserData::OpenDataFile(const char *file_path)
{
    FILE *file;
    file = fopen(file_path, "rb");

    if(!file)
    {
        cout<<"Cannot Open the File"<<endl;
        return false;
    }

    int nUsers,nTrackedPoints,nDim,nPoints,t,eve,taskId;

    fscanf(file, "%s %d", m_taskName, &taskId); // read task name (description)
    fread((void*)(&nTrackedPoints),sizeof(int),1,file); // read number of tracked points (hand=1,u-body=11)
    fread((void*)(&nUsers),sizeof(int),1,file); // read number of users in the data set
    fread((void*)(&nDim),sizeof(int),1,file); // read dimension in the data set

    this->m_nUsers = nUsers;
    this->m_nTrackedPoints = nTrackedPoints;
    this->m_nDimensions = nDim;
    this->m_taskID = taskId;

    for (int i=0; i<this->m_nUsers; i++) // iteration by the number of users
    {
        userData pInteractData; // create buffer for individual interaction data
        pInteractData.id = i; // m_Id (user ID)
        fread((void*)(&nPoints),sizeof(int),1,file);    // read number of points forming a user motion sequence
        pInteractData.nPoints = nPoints; // m_nPoints

        for (int k=0; k<pInteractData.nPoints; k++)
        {
            float *point = new float[m_nTrackedPoints*m_nDimensions];
            fread(point,sizeof(float),3*(m_nTrackedPoints),file); // read tracked points,(3*nTrackedPoints)
            fread((void*)(&t),sizeof(int),1,file); // read marking time
            fread((void*)(&eve),sizeof(int),1,file); // read event status

            pInteractData.pointList.push_back(point); // m_point
            pInteractData.event.push_back(eve);   // m_event
            pInteractData.time.push_back(t);      // m_time
        }
        m_InteractDataList.insert(pair<int, userData>(i, pInteractData));
    }
    fclose(file);

    return true;
}

float* cdUserData::GetCoordinate(int userId, int pointIdx)
{
    float *pt = (m_InteractDataList[userId].pointList[pointIdx]);
    return pt;
}

int cdUserData::GetUserID(int idx)
{
    userData data = m_InteractDataList[idx];
    return data.id;
}

int cdUserData::GetTimeStamp(int userId, int pointIdx)
{
//    int time = this->m_InteractDataList[userId]->m_time[pointIdx];
    int time = this->m_InteractDataList[userId].time[pointIdx];
    return time;
}

int cdUserData::GetEvent(int userId, int pointIdx)
{
//    int event = this->m_InteractDataList[userId]->m_event[pointIdx];
    int event = this->m_InteractDataList[userId].event[pointIdx];
    return event;
}

int cdUserData::GetNpoints(int userId)
{
//    int nPoints = m_InteractDataList[userId]->GetNpoints();
    int nPoints = m_InteractDataList[userId].nPoints;
    return nPoints;
}