#include "Placer.hpp"
#include <bits/stdc++.h>

using namespace std;

int main(int argc,char**argv){
    if(argc!=3){
        cout<<"usage: ./hw4 in.txt out.out\n"; return 0;
    }
    Placer pl;
    pl.readInput(argv[1]);
    pl.build();
    pl.runSA();
    pl.writeOutput(argv[2]);
    return 0;
}