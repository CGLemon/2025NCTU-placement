#pragma once
#include "BStarTree.hpp"
#include "Block.hpp"
#include "SymGroup.hpp"
#include <unordered_map>
#include <vector>

using namespace std;

/* 代表一個 symmetry-island：用 BStarTree 打包「代表半邊」，再鏡射 */
class ASFIsland {
public:
    SymGroup *sg;                         // 指回原對稱群
    BStarTree<int64_t> bst;
    vector<int> blockIds;           // 代表半邊的 block id
    vector<pair<int,int>> topSegs; // 代表半邊的 contour segments
    int  bboxW = 0, bboxH = 0;            // 半邊外框
    int  axisPos = 0;                     // 垂直：x；水平：y

    ASFIsland(SymGroup* g): sg(g) {}

    /* 以 blocks[] 為資料來源建立代表節點並 buildTree */
    void build(const std::unordered_map<std::string,int>& id,
               const std::vector<Block>& blocks);

    /* 打包代表半邊 → 鏡射並寫回 blocks 的 (x,y) */
    void pack(std::vector<Block>& blocks, const std::unordered_map<std::string,int>& idx);

    // void buildInitialSolution(
    //     const std::vector<Node<int64_t>*>& modules,
    //     std::vector<Node<int64_t>*>& preorder,
    //     std::vector<Node<int64_t>*>& inorder);


    void buildInitialSolution(
        const std::vector<Node<int64_t>*>& pairReps,
        const std::vector<Node<int64_t>*>& selfReps,
        std::vector<Node<int64_t>*>& preorder,
        std::vector<Node<int64_t>*>& inorder);
};
