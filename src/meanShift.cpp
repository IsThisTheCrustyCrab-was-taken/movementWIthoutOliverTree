//
// Created by benni on 25.01.2022.
//

#include "meanShift.h"
#include <Arduino.h>
int value[3];
int confidence[3];
int penis[1024];
int windowSize = 16; //should be 2^n
/*for(int i=0; i<1024; i++){
    penis[i] = random(0, 1024);
    if(randomm(0,3)==0){
        penis[i]=random(0,3);
    }
}*/
int histogram[1024];
void learn(int data[]){
    int windows[(int)round(sizeof(histogram)/windowSize)]; //mayb floor??
    for(int i = 0; i < sizeof(windows); i++){
        windows[i] *= windowSize;
    }
    for(int i = 0; i < sizeof(data); i++){
        histogram[data[i]]++;
    }
    bool solved = false;
    while(!solved){
        solved = true;
        for(int i = 0; i < sizeof(windows); i++){
            int windowI = windows[i];
            int sum = 0;
            int amm = 0;
            for (int j = windowI; i<windowI +16; i++){
                sum += j*histogram[j];
                amm += histogram[j];
            }
            int newWindowI = floor((sum/amm)-(windowSize/2));
            if (newWindowI != windowI){
                solved = false;
                windows[i] = newWindowI;
            }
        }
    }
    for(int i = 0; i < sizeof (windows); i++){

    }
}