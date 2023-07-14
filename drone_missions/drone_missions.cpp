#include<iostream>
#include<vector>
#include<cmath>

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


class Payload {
private:
    std::vector<std::vector<std::pair<float, float>>> locations;
    int length;  // Length of the payload

public:
    std::vector<int> payload_idx;
    std::vector<int> payload_weight;
    // Constructor
    Payload() {}

    Payload(int payloadLength)
        : length(payloadLength) {}

    // Getter method
    int getLength() const { return length; }
    std::vector<int> getpayload_idx() const { return payload_idx;}
    std::vector<int> getpayload_weight() const { return payload_weight;}
    std::vector<std::vector<std::pair<float, float>>> getLocations() const { return locations; }

    // Setter method
    void setLength(const int newLength) { length = newLength; }
    void setpayload_idx(const std::vector<int>& payload_idx_) {
        if(payload_idx_.size()==Payload::length){
            payload_idx = payload_idx_;
        }
        else{
            std::cout<<"the lenght of the payload is not same as lenght of payload_idx "<<std::endl;
        }

    }

    void editLocation(size_t payloadIndex, const std::vector<std::pair<float, float>>& newLocation) {
        if (payloadIndex < locations.size()) {
            locations[payloadIndex] = newLocation;
        } else {
            std::cout << "Invalid payload index or location index." << std::endl;
        }
    }

    void modellocations(){
        std::vector<std::vector<std::pair<float,float>>> location;
        std::vector <int> payload_list;
        location = getLocations();
        payload_list = getpayload_weight();
        std::vector<std::pair<float, float>> newlocation;
        for(int i=0; i<location.size();i++){
            if(location[i].size() == payload_list[i]){
                continue;
            }
            else if (payload_list[i] == 2){
                newlocation = split_2(location[i]);
                editLocation(i,newlocation);
            }
            else if(payload_list[i] == 3){
                newlocation=split_3(location[i]);
                editLocation(i,newlocation);
            }
        }
        
    } 

    std::vector<std::pair<float, float>>split_2(std::vector<std::pair<float,float>> location){
        float x = location[0].first;
        float y = location[0].second;
        float d =0.5;
        std::vector<std::pair<float,float>> location_;
        location_= {{x,y-d},{x,y+d}};

        return location_;
        
    }
    
    std::vector<std::pair<float,float>> split_3(std::vector<std::pair<float,float>> location){
        float cx = location[0].first;
        float cy = location[0].second;
        float d = 0.3;
        //yet to do;
        float y1 = cy - (d)*cos(M_PI/6);
        float x1 = cx - d/2;
        float y2 = cy + (d)*cos(M_PI/6);
        float x2 = cx - d/2;

        std::vector<std::pair<float,float>> location_;
        location_ = {{x1,y1},{x2,y2},{cx+d,cy}};
        return location_;
    }


    void setpayload_weight(const std::vector<int>& payload_weight_){
        if(payload_weight_.size()==Payload::length){
            payload_weight=payload_weight_;
        }
        else{
            std::cout<<"lenght of the payload is not same as lenght of payload_weight"<<std::endl;
        }
        
    }

    void setLocations(const std::vector<std::vector<std::pair<float, float>>>& newLocations) {
        if(newLocations.size()==length){ 
            locations = newLocations; 
        }
        else{
            std::cout<<"the location lenght is not equal to payload lenght"<<std::endl;
        }
    }


    // Other methods
    void print_length(){
        int x = Payload::getLength();
        std::cout<<x<<std::endl;
    }

    void printpayloads_idx() const {
        std::cout << "Payload idx: ";
        for (const auto& l : payload_idx) {
            std::cout << l << " ";
        }
        std::cout <<std::endl;
    }

    void printpayload_weight() const{
        std::cout<<"Payload weight: ";
        for(const auto&l : payload_weight){
            std::cout<<l<<" ";
        }
        std::cout<<std::endl;
    }

    void printlocations() const {
        std::cout << "Payload Details:" << std::endl;
        for (size_t i = 0; i < locations.size(); ++i) {
            std::cout << "Locations: " << std::endl;
            for (const auto& location : locations[i]) {
                std::cout << "(" << location.first << ", " << location.second << ")" << std::endl;
            }
            std::cout << std::endl;
        }
    }

};

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

int main(){

    int payload_len = 5;
    std::vector<int> payload_idx_= {0,5,1,2,3};
    std::vector<int> payload_weight_ ={1,2,3,2,1};
    std::vector<std::vector<std::pair<float, float>>> Locations_ = {
        { {1.1, 0.1}},       
        { {0.5, 0.6}},
        { {0,0}},
        { {-1,-1}},  
        { {-0.6, 0.8}} 
    };

    std::vector<std::vector<std::pair<float,float>>> final_locations;


    Payload payloads;
    std::vector<int> drones_list;
    payloads.setLength(payload_len);
    payloads.print_length();
    payloads.setpayload_idx(payload_idx_);
    payloads.printpayloads_idx();
    payloads.setpayload_weight(payload_weight_);
    payloads.printpayload_weight();
    payloads.setLocations(Locations_);
    payloads.modellocations();
    payloads.printlocations();


    Drone drone();



    

}

