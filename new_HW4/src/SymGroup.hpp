#pragma once
#include <vector>
#include <string>

enum AxisDir { VERTICAL, HORIZONTAL };

struct SymPair { std::string a,b; };
struct SymSelf { std::string a; };

struct SymGroup {
    std::string   name;
    AxisDir       axis;                    // 垂直/水平
    std::vector<SymPair> pairs;
    std::vector<SymSelf> selfs;
};