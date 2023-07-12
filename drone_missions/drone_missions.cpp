#include<iostream>
#include<vector>

class Drone {
private:
    std::vector<float> position;  // Position of the drone [x, y]
    float payloadCapacity;        // Maximum payload capacity of the drone
    float velocity;               // Velocity of the drone

public:
    std::vector<int> current_postion;
    float total_wait_time;
    float toatl_distance;
    float total_time;

    // Constructor
    Drone(float initialX, float initialY, float capacity, float speed)
        : position({ initialX, initialY }), payloadCapacity(capacity), velocity(speed) {}

    // Getter methods
    float getX() const { return position[0]; }
    float getY() const { return position[1]; }
    float getPayloadCapacity() const { return payloadCapacity; }
    float getVelocity() const { return velocity; }

    // Setter methods
    void setX(float newX) { position[0] = newX; }
    void setY(float newY) { position[1] = newY; }
    void setPayloadCapacity(float newCapacity) { payloadCapacity = newCapacity; }
    void setVelocity(float newSpeed) { velocity = newSpeed; }

    // Other methods
    void printPosition() const {
        std::cout << "Current position: (" << position[0] << ", " << position[1] << ")" << std::endl;
    }

    void set_current_postion(std::vector<int> vec){
        current_postion[0]=vec[0];
        current_postion[1]=vec[1];
    }
    float get_current_x() const{return current_postion[0];}
    float get_current_y() const {return current_postion[1];}


};



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
    Drone *drones_list[Drones::drone_lenght];
     
    for(int i=0;i<Drones::drone_lenght;i++){
        drones_list[i]= new Drone(0,0,30,0.1);
    }

    
}
void Drones::drone_status(void){

}
int main(){
    std::vector<int> drones_list
    

    

}

