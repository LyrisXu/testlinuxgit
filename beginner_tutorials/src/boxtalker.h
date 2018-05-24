#ifndef boxtalker
#define boxtalker
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <string>
#include <stdio.h>      /* printf */
#include <math.h>       /* ceil */

//right arm
ros::Publisher body_to_head;
ros::Publisher head_to_arm;


std::vector<std::vector<std::vector<float> > > population;

int special_indicator=0;//0 means start, 1 means left, -1 means right

float randomRange=0.8f;

int EXPTIME=40;
int TARGET_EXP_TIME=500;
int ONE_CHUNK_SIZE=5;
int BONUS_FOR_DISTANCE=3;
int ORGIN_POPULATION_SIZE=50;
int DECREASE_RATE_POP=0;
int START_GENERATION=0;

int POPULATION_SIZE=ORGIN_POPULATION_SIZE;
int count=0;
int pop_index=0;
int bonus_mark=0;
int generation_index=0;
float mutation_rate=0.1f;

double distance_largest=0;
double distance_sum=0;
double distance_avg=0;
double distance_standardDeviation=0;

double time_sum=0;
double time_avg=0;
double time_standardDeviation=0;

double point_sum=0;
double point_avg=0;
double point_standardDeviation=0;

double bonus_sum=0;
double bonus_avg=0;
double bonus_standardDeviation=0;

void mutation();
void saveFile(int generation);
void saveAvg(int generation);
void readFile();
void crossOver();
void selection();
void fitCal();
void resetExp();

#endif