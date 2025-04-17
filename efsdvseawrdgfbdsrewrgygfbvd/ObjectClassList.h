#ifndef __OBJECTCLASSLIST_H__
#define __OBJECTCLASSLIST_H__

struct ObjectDetectionItem {
    uint8_t index;
    const char* objectName;
    uint8_t filter;
};

// List of objects the pre-trained model is capable of recognizing
// Index number is fixed and hard-coded from training
// Set the filter value to 0 to ignore any recognized objects
ObjectDetectionItem itemList[80] = {
    {0,  "Psilopogon nuchalis",1},
    {1,  "Passer montanus",1},
    {2,  "Gorsachius melanolophus",1},
    {3,  "Cheirotonus formosanus",1},
    {4,  "Trypoxylus dichotomus",1}, 
};

#endif