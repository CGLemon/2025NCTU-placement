#pragma once
#include <string>

struct Block {
    std::string name;
    int w, h;
    int x = 0, y = 0;   // lower–left
    int gid; // group id => 判斷某顆 Block 是不是屬於某個 symmetry-island，以及屬於哪一座 island。
    bool rot = false;   // rotated 90°
    Block() = default;
    Block(std::string n,int W,int H):name(n),w(W),h(H){}
    inline int W() const { return rot ? h : w; }
    inline int H() const { return rot ? w : h; }
};