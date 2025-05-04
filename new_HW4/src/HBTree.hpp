#pragma once
#include "BStarTree.hpp"
#include "ASFIsland.hpp"
#include "Block.hpp"
#include <bits/stdc++.h>

/* 只處理「島視為矩形 + 其餘模組矩形」的簡化 HB-tree */
class HBTree {
    BStarTree<int64_t> bst;
    std::vector<Node<int64_t>*> hierNodes;
    std::vector<Node<int64_t>*> soloNodes;
    vector<int> soloNumber;
public:
    /* 建立 init node vector（左傾）並 buildTree */
    void build(const std::vector<ASFIsland*>& islands,
               const std::vector<int>&   soloIds,
               std::vector<Block>&       blocks,
               unordered_map<std::string,int>& idx);

    /* 打包 → 更新所有 block 座標 (調用 island.pack) → 回傳面積 */
    int64_t pack(std::vector<ASFIsland*>& islands,
                 std::vector<Block>& blocks,
                 const std::unordered_map<std::string,int>& idx);

    void buildInitialSolution(
        const std::vector<Node<int64_t>*>& modules,
        std::vector<Node<int64_t>*>& preorder,
        std::vector<Node<int64_t>*>& inorder);

};