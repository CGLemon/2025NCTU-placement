#include <iostream>
#include <string>

#include "placer.hpp"

int main(int argc, const char ** argv){
    if (argc != 3) {
        std::cout<<"usage: ./hw4 in.txt out.out\n"; return -1;
    }
    Placer p;
    p.ReadFile(std::string(argv[1]));
    p.RunSimulatedAnnealing();
    p.WriteFile(std::string(argv[2]));

    return 0;
}
