/************************  Placer.hpp  ************************/
#include "ASFIsland.hpp"
#include "HBTree.hpp"
#include "Utils.hpp"
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <iomanip>
#include <iostream>
#include <algorithm>

class Placer {
    /* ------------------ data ------------------ */
    std::vector<Block>     blocks;      // 所有 HardBlock
    std::vector<SymGroup>  groups;      // 對稱群
    std::unordered_map<std::string,int> idmap;   // block name → idx

    std::vector<ASFIsland*> islands;    // 每群一顆島
    std::vector<int>        soloIds;    // 非對稱單塊
    HBTree                  hb;         // 外層打包器

    /* Best solution cache */
    int64_t   bestArea = INT64_MAX;
    std::vector<Block> bestBlocks;

    /* -------------- helpers ------------------- */
    int64_t packAll() {
        int64_t area = hb.pack(islands, blocks, idmap);
        return area;
    }

public:
    /* -------------- API ----------------------- */
    void readInput(const std::string& path);
    void build();
    void runSA();
    void writeOutput(const std::string& path);
};