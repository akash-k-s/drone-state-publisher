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
    int mission_len;
    std::vector<int> mission_idx;
    int drones_len;
    std::vector<std::vector<int>> drones_mission;
    std::vector <int> status;
    public:
    

    // get fuctions
    int getmission_len(){
        return mission_len;
    }

    std::vector<int> getmission_idx(void){
        return mission_idx;
    }

    int getdrones_len(void){
        return drones_len;
    }

    std::vector<std::vector<int>> getdrones_missons( void ){
        return drones_mission;
    }

    std::vector<int> getstatus (void){
        return status;

    }
    //set functions
     
    void setmissions_len(int x){ mission_len = x;}

    void setmission_idx(const std::vector<int>& payload_idx_) {
        if(payload_idx_.size()==mission_len){
            mission_idx = payload_idx_;
        }
        else{
            std::cout<<"the lenght of the mision is not same as lenght of mission_idx "<<std::endl;
        }

    }

    void setdrones_len(int x){
        drones_len = x;
    }

    void setdrones_mission(const std::vector<std::vector<int>> x){
        if(x.size() == mission_len && x[0].size() == drones_len){
            drones_mission = x;
        }
        else{
            std::cout<<" the matrix size is not same as drone lenght and payload lenght "<<std::endl;
        }
    }

    void setstatus_list(){
        status.clear();
        for(int i=0 ;i<mission_len;i++){
            status.push_back(0);
            // status = 0 not started
            // status = 1 started
            // status = 2 queued
            // status = 3 completed
        }
    }

    //other methods

    void print_mission_len(){
        int x = this->getmission_len();
        std::cout<<"mission lenght : " << mission_len << std::endl;
    }

    void print_mission_idx(){
        std::cout<< "mission idx : ";
        std::vector<int> x = this-> getmission_idx();
        for (const auto& l : x) {
            std::cout << l << " ";
        }
        std::cout <<std::endl;
    }

    void print_dronelen(){
        int x = this->getdrones_len();
        std::cout<<"drone len: "<< x<< std::endl;
    }

    void print_drones_mission(){
        std::cout<<"mission_drones : "<<std::endl;
        for(int i=0; i<mission_len;i++){
            for(int j=0; j<drones_len;j++){
                std::cout<<drones_mission[i][j]<<" "; 
            }
            std::cout<<std::endl;
        }
    }
    void print_status(){
        std::cout<<"status : ";
        for(int i =0;i<mission_len;i++){
            std::cout<<status[i]<<" ";
        }
        std::cout<<std::endl;
    }

    void misions_check(void){
        int len_ = mission_len;
        int drones_len_ = drones_len;
        for (int i=0; i<len_;i++){
            if(status[i]==0){
                int check = 1;
                for(int j=0; j<drones_len_; j++){
                    if (drones_mission[i][j] == 1){
                       //check if the drones have recahed the postion
                        check = check*1;
                        if (check ==1){
                            this->mission_start();
                            status[i]=1;
                        }
                    }

                }
            }
            else if(status[i] == 1){
                for(int j =0; j<drones_len_;j++){
                    if(drones_mission[i][j]==1){
                        status[i]=3;
                    } 
                }
            }
        } 
    }

    void mission_start(){
        //add start
    }

};



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


int main(){

    // payload initilization

    int payload_len = 5;
    std::vector<int> payload_idx_= {0,4,1,2,3};
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
    //payloads.print_length();
    payloads.setpayload_idx(payload_idx_);
    //payloads.printpayloads_idx();
    payloads.setpayload_weight(payload_weight_);
    //payloads.printpayload_weight();
    payloads.setLocations(Locations_);
    payloads.modellocations();
    //payloads.printlocations();

    //mission initilization

    int drone_len = 4 ;
    std::vector<std::vector<int>> mission_drones_list = {
        {0,0,0,1},
        {0,1,1,0},
        {1,1,0,1},
        {1,0,1,0},
        {0,1,0,0}
    } ;


    Mission mission;
    mission.setmissions_len(payload_len);
    //mission.print_mission_len();
    mission.setmission_idx(payload_idx_);
    //mission.print_mission_idx();
    mission.setdrones_len(drone_len);
    //mission.print_dronelen();
    mission.setdrones_mission(mission_drones_list);
    //mission.print_drones_mission();
    mission.setstatus_list();
    mission.print_status();


    //creation of drones
    Drone D1(0,0,30,0.1);
    Drone *drone_list[drone_len];
    for (int i=0; i<drone_len;i++){
        drone_list[i]= new Drone(0,0,30,0.1);
    }

    //Drone *drones_list[drone_len];
    //for (int i = 0; i<drone_len; i++){
    //    drones_list[i] = new Drone(0,0,30,0.1);
    //}


    

}

