#include "boxtalker.h"

void saveAvg(int generation){
  std::ofstream out;
  std::stringstream ss;
  ss<<generation;
  ss<<".txt";
  std::string fileName = "/home/zqshi/BoxExperimentData/Statistics";
  fileName+=ss.str();
  out.open (fileName.c_str());

  if (out.is_open()) {
        out <<distance_avg;
        out <<",";
        out <<distance_standardDeviation;
        out <<",";
        out <<distance_largest;
        out<<"\n";
  }
  distance_largest=0;

  out.close();
}

void saveFile(int generation){
  std::ofstream out;
  std::stringstream ss;
  ss<<generation;
  ss<<".txt";
  std::string fileName = "/home/zqshi/BoxExperimentData/Generation";
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
  std::ifstream fin2("/home/zqshi/BoxExperimentData/config.txt");
  int index=0;
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
  ss<<"/home/zqshi/BoxExperimentData/Generation";
  ss<<START_GENERATION;
  ss<<".txt";
  generation_index=START_GENERATION;
  POPULATION_SIZE=ORGIN_POPULATION_SIZE*ORGIN_POPULATION_SIZE/(ORGIN_POPULATION_SIZE+DECREASE_RATE_POP*generation_index);
  ROS_INFO("Config File Done");
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
  int i=0,n=0,m=0;
 if(generation_index==0){
    while (true){
      //std::cout<<idata<<" ";
      if(n<ONE_CHUNK_SIZE-3){
        if(n<5){
          population[i][m][n] = 
          -randomRange + 
          static_cast <float> (rand()) /( 
            static_cast <float> (RAND_MAX/(randomRange+randomRange))
            );
        }
        else{
           population[i][m][n]= population[i][m][n-5];
        }
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
    ROS_INFO("Generation File Done");
    pop_index=0;
  }
}


void mutation(){
  //When start Mutation, it is down with the crossover, still old generation number
  ROS_INFO("Start Mutation For Generation %d", generation_index);
  for(int i=POPULATION_SIZE/10+1;i<POPULATION_SIZE;i++){
    for(int n=0;n<5;n++){
      for(int m=0;m<ONE_CHUNK_SIZE-3;m++){
          if(rand()%1024/1024<mutation_rate){
             population[i][n][m] = 
              -randomRange + 
              static_cast <float> (rand()) /( 
                static_cast <float> (RAND_MAX/(randomRange+randomRange)
                ));
          }
      }
    }
  }

 for (int i = POPULATION_SIZE/10+1; i < POPULATION_SIZE; ++i){

    for(int j=5;j<EXPTIME;j++){
      //Exchange left arm and right arm info
      if(j>=EXPTIME)
        break;

      for(int q=0;q<ONE_CHUNK_SIZE;q++){
         population[i][j][q]=population[i][j-5][q];
      }

    }
  }

  mutation_rate=0.05f*TARGET_EXP_TIME/(generation_index+TARGET_EXP_TIME);

  ROS_INFO("END Mutation For Generation %d, next time Mutation Rate is %f", generation_index, mutation_rate);
}

void crossOver(){
  //When start Cross Over, it is according to new generation popluation, not older one
  ROS_INFO("Start Crossover For Generation %d", generation_index);
  int tmp =0,tmp1=POPULATION_SIZE/2;
  for(int i=POPULATION_SIZE/10+1;i<POPULATION_SIZE-1;i+=2){
    int parent_index1=tmp;
    int parent_index2=parent_index1+tmp1;
    if(parent_index2>=POPULATION_SIZE)
      parent_index2=POPULATION_SIZE-1;
    for(int n=0;n<5;n++){
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
      if(tmp>=POPULATION_SIZE/tmp)
        tmp=0;
    }
  }

  
    // Value at n-i-1 will be maximum of all the values below this index.
   

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

  for (int i = 0; i < POPULATION_SIZE; ++i){

      for(int j=5;j<EXPTIME;j++){
        //Exchange left arm and right arm info
        if(j>=EXPTIME)
          break;

        for(int q=0;q<ONE_CHUNK_SIZE;q++){
           population[i][j][q]=population[i][j-5][q];
        }

      }
    // Value at n-i-1 will be maximum of all the values below this index.
  } 
  ROS_INFO("END Selection For Generation %d", generation_index);
}

void fitCal(){
    ROS_INFO("Generation Statistics Calculation Start");
    distance_sum=0;
    distance_standardDeviation=0;
    for(int n=0;n<POPULATION_SIZE;n++){
      population[n][0][ONE_CHUNK_SIZE-1]=population[n][0][ONE_CHUNK_SIZE-2];
      distance_sum+=population[n][0][ONE_CHUNK_SIZE-2];
      //Time passed before falling down + distance grade
    }
    distance_avg=distance_sum/POPULATION_SIZE;
    for(int n=0;n<POPULATION_SIZE;n++){
      distance_standardDeviation+=(population[n][0][ONE_CHUNK_SIZE-2]-distance_avg)*(population[n][0][ONE_CHUNK_SIZE-2]-distance_avg);
    }
    //Variance
    distance_standardDeviation=distance_standardDeviation/POPULATION_SIZE;
    distance_standardDeviation=sqrt(distance_standardDeviation);
    ROS_INFO("DISTANCE AVG: %f",distance_avg);
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
    count=0;
    pop_index++;
}



void linkstatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){

    population[pop_index][0][ONE_CHUNK_SIZE-2]=msg->pose[1].position.x;//*msg->pose[1].position.x+msg->pose[1].position.y*msg->pose[1].position.y;
    if(distance_largest<population[pop_index][0][ONE_CHUNK_SIZE-2])
      distance_largest=population[pop_index][0][ONE_CHUNK_SIZE-2];
  // population[pop_index][0][ONE_CHUNK_SIZE-2]=msg->pose[1].position.x;
  //   if(distance_largest<population[pop_index][0][ONE_CHUNK_SIZE-2])
  //     distance_largest=population[pop_index][0][ONE_CHUNK_SIZE-2];
   
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");
  ROS_INFO("Start Simulation, God Bless Me!");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  //right arm
  body_to_head = n.advertise<std_msgs::Float64>("my_box_head_controller/command", 1);
  head_to_arm = n.advertise<std_msgs::Float64>("my_box_arm_controller/command", 1);


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
        msg.data=0.5f;
        
        body_to_head.publish(msg);
        head_to_arm.publish(msg);
 
        count++;
      }else{  
        //ROS_INFO("Individual %d Testing, with %d",pop_index, expTime);  
        //right arm
        msg.data= population[pop_index][expTime][0];
        body_to_head.publish(msg);
        msg.data= population[pop_index][expTime][1];
        head_to_arm.publish(msg);
  
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
    POPULATION_SIZE=ORGIN_POPULATION_SIZE;//*ORGIN_POPULATION_SIZE/(ORGIN_POPULATION_SIZE+DECREASE_RATE_POP*generation_index);
    crossOver();
    mutation();
    pop_index=0;

  }
  return 0;
}
