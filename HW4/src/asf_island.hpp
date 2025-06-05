#pragma once
#include <unordered_map>
#include <vector>
#include <cstdint>

#include "BStarTree.hpp"
#include "types.hpp"
#include "utils.hpp"

/* 代表一個 symmetry-island：用 BStarTree 打包「代表半邊」，再鏡射 */
class AsfIsland {
public:
    AsfIsland(SymmGroup * g): group_(g) {}

    void Initialize(std::vector<Block> &blocks);
    std::int64_t PackAndGetPenaltyArea(std::vector<Block>& blocks);
    void GetPenalty(std::vector<Block>& blocks);
    void BuildInitialSolution();
    void UpdateNodes(const std::vector<Block>& blocks);

    void Mirror(std::vector<Block>& blocks);
    int GetNumberNodes() const;

    RotateNodeOp RotateNodeRandomize(std::vector<Block>& blocks);
    SwapNodeOp SwapNodeRandomize();
    LeafMoveOp MoveLeafNodeRandomize();

    inline int GetWidth() const { return bbox_w_; }
    inline int GetHeight() const { return bbox_h_; }
    const std::vector<int>& GetBlockIds() const { return block_ids_; }

private:
    NodePointer GetTreesRoot();
    NodePointer TryConnectTrees();

    SymmGroup * group_;                       // 指回原對稱群
    BStarTree<IdType> bs_tree_;               // 代表半邊的 BStarTree
    
    std::vector<int> block_ids_;              // 全部的 block id  
    std::vector<std::pair<int,int>> contour_; // 代表半邊的 contour segments

    NodePointerList pair_represent_nodes_;    // 代表半邊的對稱對點
    NodePointerList self_represent_nodes_;    // 代表半邊的字對稱點
    NodePointerList all_represent_nodes_;

    int bbox_w_{0}, bbox_h_{0};               // 半邊外框
    int axis_pos_{0};                         // 垂直：x；水平：y

    NodePointer pair_root_;
    NodePointer self_root_;
};
