#ifndef PATHS
#define PATHS

typedef struct pathStruct{
    struct nodeGate* A;
    struct nodeGate* B;
    double totTime;
    int traverseCount;
} pathStruct;

typedef struct nodeGate{
    struct nodeGate* ccGate;
    struct nodeGate* cwGate;
    struct pathStruct* path;
    struct nodeStruct* node;
    int conNum;
} nodeGate;

typedef struct nodeStruct{
    unsigned char broadcast;
    int n_gates;
    struct nodeGate gateArr[8];
    struct nodeGate* gateCon[8];
    int u_gates;
    int complete;
} nodeStruct;

nodeStruct* newNode(char broadcast, int n_gates);
pathStruct* newPath(nodeGate* gate);
nodeGate* addCCGate(nodeGate* gate, nodeStruct* node);
nodeGate* finishPath(pathStruct** path, nodeStruct* node);
int checkComplete(nodeStruct* node);
int autoComplete(nodeStruct* node);

#endif