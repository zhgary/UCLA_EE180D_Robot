#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "paths.h"

nodeStruct* newNode(char broadcast, int n_gates)
{
    nodeStruct* node = calloc(1,sizeof(nodeStruct));
    node->broadcast = broadcast;
    node->n_gates = n_gates;
    node->u_gates = 0;
    node->complete = 0;
    int i;
    for (i=0;i<8;i++)
    {
        node->gateArr[i].ccGate = NULL;
        node->gateArr[i].cwGate = NULL;
        node->gateArr[i].path = NULL;
        node->gateArr[i].node = node;
        node->gateArr[i].conNum = -1;
        node->gateCon[i] = NULL;
    }
    return node;
}

pathStruct* newPath(nodeGate* gate)
{
    pathStruct* path = calloc(1,sizeof(pathStruct));
    path->A = gate;
    path->traverseCount = 0;
    path->totTime = 0;
    gate->path = path;
    return path;
}

nodeGate* addCCGate(nodeGate* gate, nodeStruct* node)
{
    int i;
    nodeGate* thisGate;
    
    if (gate->node != node)
    {
        return NULL;
    }
    for (i=0;i < node->n_gates;i++)
    {
        thisGate = node->gateCon[i];
        if (thisGate == gate)
        {
            thisGate = node->gateCon[i] = &(node->gateArr[node->u_gates]);
            thisGate->conNum = i;
            thisGate->cwGate = gate;
            node->u_gates++;
            
            if (node->u_gates > node->n_gates)
            {
                printf("this should never happen/n");
            }
            
            gate->conNum = -1;
            gate->ccGate = thisGate;
            return thisGate;
            break;
        }
    }
    return NULL;
}

nodeGate* finishPath(pathStruct** path, nodeStruct* node)
{
    int i;
    int j;
    nodeGate* thisGate;
    for (i=0;i < node->n_gates;i++)
    {
        thisGate = node->gateCon[i];
        if (thisGate != NULL)
        {
            while(thisGate != NULL)
            {
                if (thisGate->path->A->node == (*path)->A->node || thisGate->path->B->node == (*path)->A->node)
                {
                    if (thisGate->path != (*path))
                    {
                        thisGate->path->totTime += (*path)->totTime;
                        thisGate->path->traverseCount += (*path)->traverseCount;
                        free(*path);
                        *path = NULL;
                    }
                    return thisGate;
                }
                thisGate = thisGate->ccGate;
            }
        }
        else
        {
            thisGate = node->gateCon[i] = &(node->gateArr[node->u_gates]);
            thisGate->conNum = i;
            node->u_gates++;
            thisGate->path = *path;
            (*path)->B = thisGate;
            return thisGate;
            break;
        }
    }
    return NULL;
}

int autoComplete(nodeStruct* node)
{
    int i;
    int gateCount = 0;
    nodeGate* prevGate1;
    nodeGate* prevGate2;
    nodeGate* thisGate1;
    nodeGate* thisGate2;
    nodeGate* startGate1;
    nodeGate* startGate2;
    
    if (node == NULL) return 0;
    
    if (checkComplete(node)) return 1;
    
    if (node->u_gates != node->n_gates) return 0;
    for (i=7;i>1;i--)
    {
        if (node->gateCon[i] != NULL) return 0;
    }
    
    thisGate1 = startGate1 = node->gateCon[0];
    while(thisGate1 != NULL)
    {
        if (thisGate1->path != NULL) gateCount++;
        prevGate1 = thisGate1;
        thisGate1 = thisGate1->ccGate;
    }
    
    thisGate2 = startGate2 = node->gateCon[1];
    while(thisGate2 != NULL)
    {
        if (thisGate2->path != NULL) gateCount++;
        prevGate2 = thisGate2;
        thisGate2 = thisGate2->ccGate;
    }
    
    if (gateCount != node->n_gates) return 0;
    
    startGate1->cwGate = prevGate2;
    startGate2->cwGate = prevGate1;
    prevGate1->ccGate = startGate2;
    prevGate2->ccGate = startGate1;
    startGate2->conNum = -1;
    node->gateCon[1] = NULL;
    
    if (!checkComplete(node))
    {
        printf("this should never happen/n");
    }
    return 1;
}

int checkComplete(nodeStruct* node)
{
    int i;
    int gateCount = 0;
    nodeGate* thisGate;
    nodeGate* startGate;
    
    if (node == NULL) return 0;
    
    if (node->u_gates != node->n_gates) return 0;
    for (i=7;i>0;i--)
    {
        if (node->gateCon[i] != NULL) return 0;
    }
    
    startGate = node->gateCon[0];
    if (startGate != NULL && startGate->path != NULL)
    {
        gateCount = 1;
        thisGate = startGate->ccGate;
        while(thisGate != startGate && thisGate != NULL)
        {
            if (thisGate->path != NULL) gateCount++;
            thisGate = thisGate->ccGate;
        }
    }
    if (gateCount == node->n_gates)
    {
        node->complete = 1;
        return 1;
    }
}
