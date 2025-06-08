#ifndef __OBJECTCLASSLIST_H__
#define __OBJECTCLASSLIST_H__

struct ObjectDetectionItem {
    uint8_t index;
    const char* objectName;
    uint8_t filter;
};

ObjectDetectionItem itemList[80] = {
{0,  "Psilopogon nuchalis",         1}, 
    {1,  "Passer montanus",        1},
    {2,  "Gorsachius melanolophus",            1},
    {3,  "Cheirotonus formosanus",      1},
    {4,  "Trypoxylus dichotomus",      1},
};

#endif
