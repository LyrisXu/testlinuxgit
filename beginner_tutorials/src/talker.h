#ifndef TALKER
#define TALKER
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/LinkStates.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <string>
#include <stdio.h>      /* printf */
#include <math.h>       /* ceil */

//right arm
ros::Publisher body_to_right_shoulder;
ros::Publisher right_shoulder_to_right_up_arm;
ros::Publisher right_up_arm_to_right_down_arm;
//left arm
ros::Publisher body_to_left_shoulder;
ros::Publisher left_shoulder_to_left_up_arm;
ros::Publisher left_up_arm_to_left_down_arm;
//right leg
ros::Publisher body_to_right_leg_joint;
ros::Publisher right_leg_joint_to_right_up_leg;
ros::Publisher right_up_leg_to_right_down_leg;
ros::Publisher right_down_leg_to_right_ankle;
ros::Publisher right_ankle_to_right_foot;
//left leg
ros::Publisher body_to_left_leg_joint;
ros::Publisher left_leg_joint_to_left_up_leg;
ros::Publisher left_up_leg_to_left_down_leg;
ros::Publisher left_down_leg_to_left_ankle;
ros::Publisher left_ankle_to_left_foot;

std::vector<std::vector<std::vector<float> > > population;

int special_indicator=0;//0 means start, 1 means left, -1 means right

int randomRange=1.8;

int EXPTIME=40;
int TARGET_EXP_TIME=500;
int ONE_CHUNK_SIZE=19;
int BONUS_FOR_DISTANCE=3;
int ORGIN_POPULATION_SIZE=1600;
int DECREASE_RATE_POP=1;
int START_GENERATION=0;

int POPULATION_SIZE=ORGIN_POPULATION_SIZE;
int count=0;
int pop_index=0;
int bonus_mark=0;
int generation_index=0;
float mutation_rate=0.1f;

std::vector<std::vector<float> > joint_limitions;

double distance_best=0;

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