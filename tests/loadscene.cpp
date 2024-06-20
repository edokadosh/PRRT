#include <json/json.h>
#include <fstream>
#include <iostream>

int main(){
    std::ifstream scene_file("scenes/tunnels_disc.json", std::ifstream::binary);
    Json::Value scene;
    scene_file >> scene;

    // std::cout<<scene;


    // std::cout<<scene["obstacles"];
    // std::cout<<scene["obstacles"][0]; 
    std::cout<<scene["obstacles"][0]["poly"][0][0] << std::endl;

    return 0;

}
