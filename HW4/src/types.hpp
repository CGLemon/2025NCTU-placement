#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

#include "BStarTree.hpp"

struct Block {
    Block() = default;
    Block(std::string n, int width, int height) : name(n), w(width), h(height) {}

    std::string name;
    int w, h;
    int x{0}, y{0};
    int gid{-1};
    bool rotated{false};
    bool pre_rotated{false};

    inline int GetRotatedWidth() const { return rotated ? h : w; }
    inline int GetRotatedHeight() const { return rotated ? w : h; }
    inline void PreRotate() { std::swap(h, w); pre_rotated = !pre_rotated; }
    inline void Rotate() { rotated = !rotated; }
    inline bool IsSolo() const { return gid == -1; }
};


enum class Axis {
    kVertical,
    kHorizontal
};

struct SymmPair {
    std::string a;
    int aid;
    std::string b;
    int bid;
};
struct SymmSelf {
    std::string a;
    int id;
};

struct SymmGroup {
    std::string name;
    int gid{-1};
    Axis axis;
    std::vector<SymmPair> pairs;
    std::vector<SymmSelf> selfs;
};

using NameToIdMap = std::unordered_map<std::string, size_t>;
using IdType = std::int64_t;
using NodeType = Node<IdType>;
using NodePointer = Node<IdType>*;
using NodePointerList = std::vector<NodePointer>;
