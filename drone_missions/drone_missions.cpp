#include<stdio.h>


class Drone
{
private:
    /* data */
public:

    float postion[2] ;
    float capacity;
    float total_wait_time;
    float total_distance;
    float avg_vel;
    float initial_postion[2];
    float total_time;

    void getpoistion(float pos[2]);

    Drone(float cap,float in_pos[2],float speed){
        initial_postion[0] = in_pos[0];
        initial_postion[1] = in_pos[1];
        avg_vel = speed;
        capacity = cap;
    }

};

void Drone::getpoistion(float pos[2]){
    // data from pybind 11

}

class Mission{
    private:

    public:
    int payload_len = 5;
    int payload_idx[5];
    int drones_len = 4;
    int drones_goals[5][4];
    int status[5];

    // status = 0 not started
    // status = 1 started
    // status = 2 queued
    // status = 3 completed
    void initialize_status(void);
    void missions_check(void);
    void mission_start(void);
};

void Mission::initialize_status(void){
    int len_ = Mission::payload_len;
    for(int i=0; i<=len_; i++){
        Mission::status [i] = 0;
    }
}

void Mission::missions_check(void){
    int len_ = Mission::payload_len;
    int drones_len = Mission::drones_len;
    for(int i=0 ; i<len_; i++){
        if  (Mission::status[i]==0){
            int check = 1;
            for(int j=0; j<drones_len; j++){
                if (Mission::drones_goals[i][j]==1){

                    // check if position is reached
                        check=check*1;
                    
                    if(check==1){
                        this->mission_start();
                        Mission::status[i]=1;
                    }

                }
            }
        }

        else if (Mission::status[i]==1){
            for(int j=0; j<drones_len; j++){
                if(Mission::drones_goals[i][j]==1){
                    // drone j reached position
                        Mission::status[i]=3;
                }
            }
            
        }
    }
}

void Mission::mission_start(){
    // add start 
}

class Drones{
    private:
    public:
    int drone_lenght;
    void drones_creater(void);
    void drone_status(void);
};

void Drones::drones_creater(void){
    Mission mission;
    Drones::drone_lenght = mission.drones_len;
    int drones_list[Drones::drone_lenght]; 
    for(int i=0;i<Drones::drone_lenght;i++){
        

    }
    
}
void Drones::drone_status(void){

}
int main(){

    

}

