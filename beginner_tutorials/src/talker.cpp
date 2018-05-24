#include "talker.h"

void saveAvg(int generation){
  std::ofstream out;
  std::stringstream ss;
  ss<<generation;
  ss<<".txt";
  std::string fileName = "/home/zqshi/ExperimentData/Statistics";
  fileName+=ss.str();
  out.open (fileName.c_str());

  if (out.is_open()) {
        out <<distance_avg;
        out <<",";
        out <<distance_standardDeviation;
        out <<",";
        out <<time_avg;
        out <<",";
        out <<time_standardDeviation;
        out <<",";
        out <<point_avg;
        out <<",";
        out <<point_standardDeviation;
        out <<",";
        out <<bonus_avg;
        out <<",";
        out <<bonus_standardDeviation;
        out <<",";
        out <<distance_best;
        out<<"\n";
  }
  distance_best=0;
  out.close();
}

void saveFile(int generation){
  std::ofstream out;
  std::stringstream ss;
  ss<<generation;
  ss<<".txt";
  std::string fileName = "/home/zqshi/ExperimentData/Generation";
  fileName+=ss.str();
  out.open (fileName.c_str());

  if (out.is_open()) {
      for(int n=0;n<POPULATION_SIZE;n++){
        for(int m=0;m<EXPTIME;m++){
          for(int i=0;i<ONE_CHUNK_SIZE;i++){
              out << population[n][m][i];
              out <<" ";
          }
          out<<"\n";
         }
       }
      out<<"-999";
      out.close();  
     }  

  out.close();

}

void readFile(){
  ROS_INFO("Read File Start");
  std::stringstream ss;
  std::ifstream fin2("/home/zqshi/ExperimentData/config.txt");
  int index=0,tmp_index=0;
  while (!fin2.eof()){
    int idata;
    fin2 >> idata;
    switch(index){
      case 0:EXPTIME=idata;break;
      case 1:TARGET_EXP_TIME=idata;break;
      case 2:ONE_CHUNK_SIZE=idata;break;
      case 3:BONUS_FOR_DISTANCE=idata;break;
      case 4:ORGIN_POPULATION_SIZE=idata;POPULATION_SIZE=ORGIN_POPULATION_SIZE;break;
      case 5:DECREASE_RATE_POP=idata;break;
      case 6:START_GENERATION=idata;break;
    }
    index++;
  }
  ROS_INFO("Read Limitation");
  std::ifstream fin3("/home/zqshi/ExperimentData/config2.txt");
  index=0;
  joint_limitions.resize(ONE_CHUNK_SIZE-3);
  for (int i = 0; i < ONE_CHUNK_SIZE-3; ++i)
    joint_limitions[i].resize(2);
  while (!fin3.eof()){

    float idata;
    fin3 >> idata;
    joint_limitions[index][tmp_index]=idata;
    ROS_INFO("Config %d, %d, %f", index,tmp_index,idata);
    tmp_index++;
    if(tmp_index==2){
      index++;
      tmp_index=0;
    }
    if(index==ONE_CHUNK_SIZE-3)
      break;
  }

  ROS_INFO("Config File Done");
  ss<<"/home/zqshi/ExperimentData/Generation";
  ss<<START_GENERATION;
  ss<<".txt";
  generation_index=START_GENERATION;
  POPULATION_SIZE=ORGIN_POPULATION_SIZE*ORGIN_POPULATION_SIZE/(ORGIN_POPULATION_SIZE+DECREASE_RATE_POP*generation_index);

  std::ifstream fin(ss.str().c_str());
  population.resize(POPULATION_SIZE);
  for (int i = 0; i < POPULATION_SIZE; ++i){
    population[i].resize(EXPTIME);
    for(int n=0;n<EXPTIME;n++)
      population[i][n].resize(ONE_CHUNK_SIZE);
  }
  /*
  Data Structure:
  0 ~ ONE_CHUNK_SIZE-4: Joint Position Data
  ONE_CHUNK_SIZE-3:     Robot Stand Time
  ONE_CHUNK_SIZE-2:     Robot Move Distance Used for Calculation Point not the real distance
  ONE_CHUNK_SIZE-1:     Robot Point
  */
  srand (static_cast <unsigned> (time(0)));

  int i=0,n=0,m=0;

  if(generation_index==0){
    while (true){
      //std::cout<<idata<<" ";
      if(n<ONE_CHUNK_SIZE-3){

         population[i][m][n] = (joint_limitions[n][0]+joint_limitions[n][1])/2 -(joint_limitions[n][1]-joint_limitions[n][0])*randomRange/2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(joint_limitions[n][1]*randomRange-randomRange*joint_limitions[n][0])));
        
        //ROS_INFO("Start is %f", population[i][m][n]);
       
      }
      else{
        population[i][m][n]=0;
      }


      n++;
      if(n==ONE_CHUNK_SIZE){
        m++;
        n=0;
      }
      if(m==EXPTIME){
        i++;
        m=0;
      }
      if(i==POPULATION_SIZE){
        break;
      }
    }
  }
  else{
    while (!fin.eof()){
      float idata;
      fin >> idata;
      population[i][m][n]=idata;
      //std::cout<<idata<<" ";
      n++;
      if(n==ONE_CHUNK_SIZE){
        m++;
        n=0;
      }
      if(m==EXPTIME){
        i++;
        m=0;
      }
      if(i==POPULATION_SIZE){
        break;
      }
    }
  }
  ROS_INFO("Generation File Done");
  pop_index=0;
}


void mutation(){
  //When start Mutation, it is down with the crossover, still old generation number

  ROS_INFO("Start Mutation For Generation %d", generation_index);
  for(int i=POPULATION_SIZE/10+1;i<POPULATION_SIZE;i++){
    for(int m=0;m<EXPTIME;m++){
      for(int n=0;n<ONE_CHUNK_SIZE-3;n++){
          if(rand()%1024/1024<mutation_rate){

             population[i][m][n] = (joint_limitions[n][0]+joint_limitions[n][1])/2 -(joint_limitions[n][1]-joint_limitions[n][0])*randomRange/2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(joint_limitions[n][1]*randomRange-randomRange*joint_limitions[n][0])));
        
          }
          // if(population[i][m][n]<joint_limitions[n][0])
          //   population[i][m][n]=joint_limitions[n][0];
          // if(population[i][m][n]>joint_limitions[n][1])
          //   population[i][m][n]=joint_limitions[n][1];
      }
    }
  }
  float temp = generation_index+TARGET_EXP_TIME;
  mutation_rate=0.1f*TARGET_EXP_TIME/temp;

  if(generation_index%50==0)
    mutation_rate=0.1f;
  ROS_INFO("END Mutation For Generation %d, next time Mutation Rate is %f", generation_index, mutation_rate);
}

void crossOver(){
  //When start Cross Over, it is according to new generation popluation, not older one
  ROS_INFO("Start Crossover For Generation %d", generation_index);
  int tmp=0,tmp1=POPULATION_SIZE/10;;
  for(int i=POPULATION_SIZE/10+1;i<POPULATION_SIZE-1;i++){
    int parent_index1=tmp;
    int parent_index2=parent_index1+tmp1;
    if(parent_index2>=POPULATION_SIZE)
      parent_index2=POPULATION_SIZE-1;

    for(int n=0;n<EXPTIME;n++){
      for(int m=0;m<ONE_CHUNK_SIZE-3;m++){
          if(rand()%8>=4){
            population[i][n][m]=population[parent_index1][n][m];
            population[i+1][n][m]=population[parent_index2][n][m];
          }
          else{
            population[i][n][m]=population[parent_index2][n][m];
            population[i+1][n][m]=population[parent_index1][n][m];
          }
      }
    }

    tmp1--;
    if(tmp1==0){
      tmp++;
      tmp1=POPULATION_SIZE/10;
      if(tmp>=POPULATION_SIZE/10)
        tmp=0;
    }

  }

  ROS_INFO("End Crossover For Generation %d", generation_index);
}

void selection(){
  ROS_INFO("Start Selection For Generation %d", generation_index);
  for (int i = 0; i < POPULATION_SIZE; ++i){
    for (int j = 0; j < POPULATION_SIZE-i-1; ++j){
      // Comparing consecutive data and switching values if value at j > j+1.
      if (population[j][0][ONE_CHUNK_SIZE-1] < population[j+1][0][ONE_CHUNK_SIZE-1] ){
        std::vector<std::vector<float> > temp = population[j];
        population[j] = population[j+1];
        population[j+1] = temp;
      }
    }
    // Value at n-i-1 will be maximum of all the values below this index.
  } 
  ROS_INFO("END Selection For Generation %d", generation_index);
}

void fitCal(){
    ROS_INFO("Generation Statistics Calculation Start");
    time_sum=0;
    point_sum=0;
    distance_sum=0;
    bonus_sum=0;

    distance_standardDeviation=0;
    point_standardDeviation=0;
    time_standardDeviation=0;
    bonus_standardDeviation=0;

    for(int n=0;n<POPULATION_SIZE;n++){
      population[n][0][ONE_CHUNK_SIZE-1]=population[n][0][ONE_CHUNK_SIZE-3]+BONUS_FOR_DISTANCE*population[n][0][ONE_CHUNK_SIZE-2]+0*bonus_mark;
      //population[n][0][ONE_CHUNK_SIZE-1]=BONUS_FOR_DISTANCE*population[n][0][ONE_CHUNK_SIZE-2];
      time_sum+=population[n][0][ONE_CHUNK_SIZE-3];
      point_sum+=population[n][0][ONE_CHUNK_SIZE-1];
      distance_sum+=population[n][1][ONE_CHUNK_SIZE-2];
      bonus_sum+=population[n][2][ONE_CHUNK_SIZE-3];
      //Time passed before falling down + distance grade
    }
  
    time_avg=time_sum/POPULATION_SIZE;
    point_avg=point_sum/POPULATION_SIZE;
    distance_avg=distance_sum/POPULATION_SIZE;
    bonus_avg=bonus_sum/POPULATION_SIZE;

    for(int n=0;n<POPULATION_SIZE;n++){
      distance_standardDeviation+=(population[n][1][ONE_CHUNK_SIZE-2]-distance_avg)*(population[n][1][ONE_CHUNK_SIZE-2]-distance_avg);
      point_standardDeviation+=(population[n][0][ONE_CHUNK_SIZE-1]-point_avg)*(population[n][0][ONE_CHUNK_SIZE-1]-point_avg);
      time_standardDeviation+=(population[n][0][ONE_CHUNK_SIZE-3]-time_avg)*(population[n][0][ONE_CHUNK_SIZE-3]-time_avg);
      bonus_standardDeviation+=(population[n][2][ONE_CHUNK_SIZE-3]-bonus_avg)*(population[n][2][ONE_CHUNK_SIZE-3]-bonus_avg);
    }
    //Variance
    distance_standardDeviation=distance_standardDeviation/POPULATION_SIZE;
    point_standardDeviation=point_standardDeviation/POPULATION_SIZE;
    time_standardDeviation=time_standardDeviation/POPULATION_SIZE;
    bonus_standardDeviation=bonus_standardDeviation/POPULATION_SIZE;
    //Standard Deviation
    distance_standardDeviation=sqrt(distance_standardDeviation);
    point_standardDeviation=sqrt(point_standardDeviation);
    time_standardDeviation=sqrt(time_standardDeviation);
    bonus_standardDeviation=sqrt(bonus_standardDeviation);

    ROS_INFO("TIME AVG: %f",time_avg);
    ROS_INFO("DISTANCE AVG: %f",distance_avg);
    ROS_INFO("POINT AVG: %f",point_avg);
    ROS_INFO("Bonus AVG: %f",bonus_standardDeviation);

    ROS_INFO("Generation Statistics Calculation END");
}

void resetExp(){
    std_srvs::Empty resetWorldSrv;
   
    if(ros::service::call("/gazebo/reset_simulation", resetWorldSrv)){
      ROS_INFO("Individual:%d finished,Reset World",pop_index); 
    }
    else { 
      ROS_INFO("Failed topic reset the world"); 
    }
    population[pop_index][0][ONE_CHUNK_SIZE-3]=count/8;
    population[pop_index][2][ONE_CHUNK_SIZE-3]=bonus_mark;
    count=0;
    bonus_mark=0;
    pop_index++;
}



void linkstatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){

  //ROS_INFO("%f",msg->pose[2].position.z);

  //population[pop_index][0][15]-=msg->pose[16].position.y;
  //population[pop_index][0][15]-=msg->pose[17].position.y;
  //population[pop_index][0][15]=msg->pose[17].position.x;
  if(msg->pose[1].position.z>0.7){ //1.1){
    double x = 0;

    //0 for start, 1 for left, 2 for right
    switch(special_indicator){
      case 0:
        if(msg->pose[7].position.x>msg->pose[1].position.x&&msg->pose[19].position.x<msg->pose[1].position.x)
          if(fabs(msg->pose[7].position.x-msg->pose[19].position.x)>0.3)
            if(msg->pose[2].position.x>msg->pose[1].position.x)
              if(msg->pose[5].position.z-msg->pose[7].position.z>0.05)
                if(msg->pose[17].position.z-msg->pose[19].position.z<0.05){
                  bonus_mark+=0.3;
                  special_indicator=2;//next should be right
                  }
        if(msg->pose[19].position.x>msg->pose[1].position.x&&msg->pose[7].position.x<msg->pose[1].position.x)
          if(fabs(msg->pose[7].position.x-msg->pose[19].position.x)>0.3)
            if(msg->pose[2].position.x>msg->pose[1].position.x)
              if(msg->pose[5].position.z-msg->pose[7].position.z>0.05)
                if(msg->pose[17].position.z-msg->pose[19].position.z<0.05){
                  bonus_mark+=0.3;
                  special_indicator=1;//next should be left
                }
        break;

      case 1:
        if(msg->pose[7].position.x>msg->pose[1].position.x&&msg->pose[19].position.x<msg->pose[1].position.x)
          if(fabs(msg->pose[7].position.x-msg->pose[19].position.x)>0.3)
            if(msg->pose[2].position.x>msg->pose[1].position.x)
              if(msg->pose[5].position.z-msg->pose[7].position.z>0.05)
                if(msg->pose[17].position.z-msg->pose[19].position.z<0.05){
                  bonus_mark+=0.3;
                  special_indicator=2;//next should be right
                  }
        break;
      case 2:
        if(msg->pose[19].position.x>msg->pose[1].position.x&&msg->pose[7].position.x<msg->pose[1].position.x)
          if(fabs(msg->pose[7].position.x-msg->pose[19].position.x)>0.3)
            if(msg->pose[2].position.x>msg->pose[1].position.x)
              if(msg->pose[5].position.z-msg->pose[7].position.z>0.05)
                if(msg->pose[17].position.z-msg->pose[19].position.z<0.05){
                  bonus_mark+=0.3;
                  special_indicator=1;//next should be left
                }
        break;

    }
   
      if(msg->pose[1].position.x>population[pop_index][0][ONE_CHUNK_SIZE-2])
        population[pop_index][0][ONE_CHUNK_SIZE-2]=msg->pose[1].position.x;
   
  }

  if(msg->pose[1].position.z<0.6){//1.0){
    if(msg->pose[2].position.z-msg->pose[1].position.z>0.2)
       bonus_mark+=0.05;

    //population[pop_index][0][ONE_CHUNK_SIZE-2]=population[pop_index][0][ONE_CHUNK_SIZE-2];
    //population[pop_index][1][ONE_CHUNK_SIZE-2]=msg->pose[1].position.x;
    if(distance_best<population[pop_index][1][ONE_CHUNK_SIZE-2])
      distance_best=population[pop_index][1][ONE_CHUNK_SIZE-2];
    resetExp();
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ROS_INFO("Start Simulation, God Bless Me!");
 
  ros::NodeHandle n;
  //right arm
  body_to_right_shoulder = n.advertise<std_msgs::Float64>("my_robot_body_to_right_shoulder_controller/command", 1);
  right_shoulder_to_right_up_arm = n.advertise<std_msgs::Float64>("my_robot_right_shoulder_to_right_up_arm_controller/command", 1);
  right_up_arm_to_right_down_arm = n.advertise<std_msgs::Float64>("my_robot_right_up_arm_to_right_down_arm_controller/command", 1);
  //left arm
  body_to_left_shoulder = n.advertise<std_msgs::Float64>("my_robot_body_to_left_shoulder_controller/command", 1);
  left_shoulder_to_left_up_arm = n.advertise<std_msgs::Float64>("my_robot_left_shoulder_to_left_up_arm_controller/command", 1);
  left_up_arm_to_left_down_arm = n.advertise<std_msgs::Float64>("my_robot_left_up_arm_to_left_down_arm_controller/command", 1);
  //right leg
  body_to_right_leg_joint = n.advertise<std_msgs::Float64>("my_robot_body_to_right_leg_joint_controller/command", 1);
  right_leg_joint_to_right_up_leg = n.advertise<std_msgs::Float64>("my_robot_right_leg_joint_to_right_up_leg_controller/command", 1);
  right_up_leg_to_right_down_leg = n.advertise<std_msgs::Float64>("my_robot_right_up_leg_to_right_down_leg_controller/command", 1);
  right_down_leg_to_right_ankle = n.advertise<std_msgs::Float64>("my_robot_right_down_leg_to_right_ankle_controller/command", 1);
  right_ankle_to_right_foot = n.advertise<std_msgs::Float64>("my_robot_right_ankle_to_right_root_controller/command", 1);
  //left leg
  body_to_left_leg_joint = n.advertise<std_msgs::Float64>("my_robot_body_to_left_leg_joint_controller/command", 1);
  left_leg_joint_to_left_up_leg = n.advertise<std_msgs::Float64>("my_robot_left_leg_joint_to_left_up_leg_controller/command", 1);
  left_up_leg_to_left_down_leg = n.advertise<std_msgs::Float64>("my_robot_left_up_leg_to_left_down_leg_controller/command", 1);
  left_down_leg_to_left_ankle = n.advertise<std_msgs::Float64>("my_robot_left_down_leg_to_left_ankle_controller/command", 1);
  left_ankle_to_left_foot = n.advertise<std_msgs::Float64>("my_robot_left_ankle_to_left_foot_controller/command", 1);

  //get the status
  ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>("gazebo/link_states", 1, linkstatesCallback); //ROS gazebo publishes all joint states, you can use these.

  ros::Rate loop_rate(32);

  readFile();

  /**
   * A count of how many messages we have sen This is used to create
   * a unique string for each message.
   */
  int expTime=count/8;
  for(;generation_index<TARGET_EXP_TIME;generation_index++){
    special_indicator=0;
    ROS_INFO("New Generation %d Started!\n POPULATION SIZE:%d",generation_index,POPULATION_SIZE);
    while (ros::ok()&&pop_index<POPULATION_SIZE){

      /**
       * This is a message object. You stuff it with data, and then publish it.
       */
      std_msgs::Float64 msg;

      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      expTime=count/8;
      if(count==0){
        ROS_INFO("Individual %d Started!",pop_index );  
        msg.data=0.0f;
        
        body_to_right_shoulder.publish(msg);
        body_to_right_leg_joint.publish(msg);
        //right arm
        body_to_right_shoulder.publish(msg);
        right_shoulder_to_right_up_arm.publish(msg);
        right_up_arm_to_right_down_arm.publish(msg);
        //left arm
        body_to_left_shoulder.publish(msg);
        left_shoulder_to_left_up_arm.publish(msg);
        left_up_arm_to_left_down_arm.publish(msg);
        //right leg
        body_to_right_leg_joint.publish(msg);
        right_leg_joint_to_right_up_leg.publish(msg);
        right_up_leg_to_right_down_leg.publish(msg);
        right_down_leg_to_right_ankle.publish(msg);
        right_ankle_to_right_foot.publish(msg);
        //left leg
        body_to_left_leg_joint.publish(msg);
        left_leg_joint_to_left_up_leg.publish(msg);
        left_up_leg_to_left_down_leg.publish(msg);
        left_down_leg_to_left_ankle.publish(msg);
        left_ankle_to_left_foot.publish(msg);
        count++;
      }else{  
        //ROS_INFO("Individual %d Testing, with %d",pop_index, expTime); 
        /**
        Joint Data Index
        Right Shoulder: 0 1
        Right Elbow: 2
        Left Shoulder: 3 4
        Left Elbow: 5
        Right Hip: 6 7 
        Right Knee: 8
        Right Ankle: 9 10
        Left Hip: 11 12
        Left Knee: 13
        Left Ankle: 14
        */ 
        //right arm
        msg.data= population[pop_index][expTime][0];
        body_to_right_shoulder.publish(msg);
        msg.data= population[pop_index][expTime][1];
        right_shoulder_to_right_up_arm.publish(msg);
        msg.data= population[pop_index][expTime][2];
        right_up_arm_to_right_down_arm.publish(msg);

        //left arm
        msg.data= population[pop_index][expTime][3];
        body_to_left_shoulder.publish(msg);
        msg.data= population[pop_index][expTime][4];
        left_shoulder_to_left_up_arm.publish(msg);
        msg.data= population[pop_index][expTime][5];
        left_up_arm_to_left_down_arm.publish(msg);

        //right leg
        msg.data= population[pop_index][expTime][6];
        body_to_right_leg_joint.publish(msg);
        msg.data= population[pop_index][expTime][7];
        right_leg_joint_to_right_up_leg.publish(msg);
        msg.data= population[pop_index][expTime][8];
        right_up_leg_to_right_down_leg.publish(msg);
        msg.data= population[pop_index][expTime][9];
        right_down_leg_to_right_ankle.publish(msg);
        msg.data= population[pop_index][expTime][10];
        right_ankle_to_right_foot.publish(msg);
        //left leg

        msg.data= population[pop_index][expTime][11];
        body_to_left_leg_joint.publish(msg);
        msg.data= population[pop_index][expTime][12];
        left_leg_joint_to_left_up_leg.publish(msg);
        msg.data= population[pop_index][expTime][13];
        left_up_leg_to_left_down_leg.publish(msg);
        msg.data= population[pop_index][expTime][14];
        left_down_leg_to_left_ankle.publish(msg);
        msg.data= population[pop_index][expTime][15];
        left_ankle_to_left_foot.publish(msg);
        //ROS_INFO("count:%d, data:%f",count, msg.data);

        ros::spinOnce();

        loop_rate.sleep();
        count++;

        if(count>=EXPTIME*8){
          resetExp();
        }
      }
    }
    ROS_INFO("END OF Simulation, Start Generation");
    fitCal();

    selection();
    saveFile(generation_index);
    saveAvg(generation_index);
    //get new Generation Population Size
    POPULATION_SIZE=ORGIN_POPULATION_SIZE*ORGIN_POPULATION_SIZE/(ORGIN_POPULATION_SIZE+DECREASE_RATE_POP*generation_index);
    crossOver();
    mutation();
    pop_index=0;

  }
  return 0;
}
