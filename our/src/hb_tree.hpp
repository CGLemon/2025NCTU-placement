#pragma once
#include <cstdint>
#include <vector>
#include <memory>

#include "BStarTree.hpp"
#include "asf_island.hpp"
#include "utils.hpp"
#include "types.hpp"

/* 只處理「島視為矩形 + 其餘模組矩形」的簡化 HB-tree */
class HbTree {
public:
    void Initialize(std::vector<Block> &blocks,
                    std::vector<SymmGroup> &groups);

    void UpdateNodes(const std::vector<Block> &blocks);

    void BuildInitialSolution();

    std::int64_t PackAndGetArea(std::vector<Block> &blocks);

    int GetNumberNodes() const;
    AsfIsland * GetIsland(int idx);

    void RotateNode(std::vector<Block> &blocks, const int idx);
    SwapNodeOp SwapNodeRandomize();
    LeafMoveOp MoveLeafNodeRandomize();

private:
    NodePointer GetNode(int idx);
    bool IsSoloNode(const int idx) const;

    NodePointerList solo_nodes_;                      // 單個 block 代表的節點
    NodePointerList hier_nodes_;                      // 對稱群代表的節點
    NodePointerList all_nodes_;
    std::vector<std::unique_ptr<AsfIsland>> islands_; // 所有對稱群
    BStarTree<IdType> bs_tree_;
};
