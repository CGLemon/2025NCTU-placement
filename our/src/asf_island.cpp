#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdint>
#include <stack>

#include "asf_island.hpp"
#include "utils.hpp"

/// BuildInitialSolution 把  pair_represent_nodes 构成一棵平衡树，
/// 再把 self_represent_nodes 串到最 “极端” 的那条分支上
void AsfIsland::BuildInitialSolution() {
    // 1. 只用 pair_represent_nodes 先做平衡树
    auto sorted = pair_represent_nodes_;
    std::sort(sorted.begin(), sorted.end(),
              [](auto a, auto b){
                  return a->width * a->height > b->width * b->height;
              });
    if (sorted.empty()) {
        pair_root_ = nullptr;
    } else {
        pair_root_ = BuildBalancedTree(sorted);
    }
    // 2. 把所有 self_represent_nodes 串成一条链
    sorted = self_represent_nodes_;

    if (sorted.empty()) {
        self_root_ = nullptr;
    } else {
        std::sort(sorted.begin(), sorted.end(),
                  [](auto a, auto b){
                      return a->width * a->height > b->width * b->height;
                  });
        if (group_->axis == Axis::kVertical) {
            self_root_ = BuildRightSkewedTree(sorted);
        } else {
            self_root_ = BuildLeftSkewedTree(sorted);
        }
    }
}

void AsfIsland::Initialize(std::vector<Block> &blocks) {
    if (!pair_represent_nodes_.empty() ||
            !self_represent_nodes_.empty()) {
        // 已經初始化過了
        return;
    }

    // (a) symmetry‑pair：固定使用右側模組 b' 當代表
    for (const auto& symm_pair: group_->pairs) {
        pair_represent_nodes_.emplace_back(new NodeType());
        pair_represent_nodes_.back()->blockId = symm_pair.bid;
        block_ids_.emplace_back(symm_pair.aid);
        block_ids_.emplace_back(symm_pair.bid);
    }
    // (b) self‑symmetric：取右(上)半；width/height 擇一對半
    for (const auto& symm_self: group_->selfs) {
        self_represent_nodes_.emplace_back(new NodeType());
        self_represent_nodes_.back()->blockId = symm_self.id;
        block_ids_.emplace_back(symm_self.id);
    }

    all_represent_nodes_.reserve(pair_represent_nodes_.size() + self_represent_nodes_.size());
    all_represent_nodes_.insert(std::end(all_represent_nodes_),
        std::begin(pair_represent_nodes_), std::end(pair_represent_nodes_));
    all_represent_nodes_.insert(std::end(all_represent_nodes_),
        std::begin(self_represent_nodes_), std::end(self_represent_nodes_));

    UpdateNodes(blocks);
    BuildInitialSolution();
}

void AsfIsland::UpdateNodes(const std::vector<Block>& blocks) {
    for (NodePointer n: pair_represent_nodes_) {
        n->setShape(
            blocks[n->blockId].GetRotatedWidth(),
            blocks[n->blockId].GetRotatedHeight()
        );
    }
    for (NodePointer n: self_represent_nodes_) {
        int half_w = (group_->axis == Axis::kVertical) ?
            blocks[n->blockId].GetRotatedWidth() / 2 :
                blocks[n->blockId].GetRotatedWidth();
        int half_h = (group_->axis == Axis::kHorizontal) ?
            blocks[n->blockId].GetRotatedHeight() / 2 :
                blocks[n->blockId].GetRotatedHeight();
        n->setShape(half_w, half_h);
    }
}

NodePointer AsfIsland::GetTreesRoot() {
    if (pair_root_) {
        return pair_root_;
    }
    return self_root_;
}

NodePointer AsfIsland::TryConnectTrees() {
    if (!pair_root_) {
        return nullptr;
    }
    NodePointer connect_node = pair_root_;
   
    if (group_->axis == Axis::kVertical) {
       while (connect_node->rchild) {
           connect_node = connect_node->rchild;
       }
       connect_node->rchild = self_root_;
    } else {
       while (connect_node->lchild) {
           connect_node = connect_node->lchild;
       }
       connect_node->lchild = self_root_;
    }
    return connect_node;
}

/********************************************************************
 *  pack()  —  1) 代表半平面打包          (bst.setPosition)
 *             2) 鏡射 mate / self 模組  (式 (1)(2))
 *             3) 校正 bbox 置於 (0,0)
 ********************************************************************/
void AsfIsland::Pack(std::vector<Block>& blocks) {   
    /* ---------- 0) 打包代表半平面 ---------- */
    UpdateNodes(blocks);

    NodePointer connect_node = TryConnectTrees();
    bs_tree_.root = GetTreesRoot();
    bs_tree_.setPosition();

    /* ---------- 1) 掃描代表並鏡射 ---------- */
    std::int64_t min_x = LLONG_MAX, min_y = LLONG_MAX;
    std::int64_t max_x = LLONG_MIN, max_y = LLONG_MIN;
    axis_pos_ = 0;

    std::stack<NodePointer> stk;
    stk.emplace(bs_tree_.root);

    while (!stk.empty()) {
        NodePointer n = stk.top();
        stk.pop();
        Block& rep = blocks[n->blockId];

        /* 1-a  設定代表座標 */
        rep.x = n->x;
        rep.y = n->y;

        /* 1-b  處理 symmetry-pair 的另一半 */
        auto pair_it = std::find_if(std::begin(group_->pairs), std::end(group_->pairs),
                           [&](const SymmPair& p){ return p.bid == n->blockId; });
        if (pair_it != std::end(group_->pairs)) {
            int mate_id = pair_it->aid;
            Block& mate = blocks[mate_id];
            mate.rotated = rep.rotated;

            if (group_->axis == Axis::kVertical) {
                mate.x = 2 * axis_pos_ - rep.x - rep.GetRotatedWidth(); // 式 (1)
                mate.y = rep.y;
            } else {
                mate.x = rep.x;
                mate.y = 2 * axis_pos_ - rep.y - rep.GetRotatedHeight(); // 式 (2)
            }
            
            // 也要把 mate 也納入 bounding‐box 更新
            min_x = std::min<std::int64_t>(min_x, mate.x);
            min_y = std::min<std::int64_t>(min_y, mate.y);
            max_x = std::max<std::int64_t>(max_x, mate.x + mate.GetRotatedWidth());
            max_y = std::max<std::int64_t>(max_y, mate.y + mate.GetRotatedHeight());
        }

        /* 1-c  self-symmetric：置中於軸 */
        auto self_it = std::find_if(std::begin(group_->selfs), std::end(group_->selfs),
                           [&](const SymmSelf& s){ return s.id == n->blockId; });
        if (self_it != std::end(group_->selfs)){
            if( group_->axis == Axis::kVertical) {
                rep.x = axis_pos_ - rep.GetRotatedWidth()/2; // 中心落在 x
            } else {
                rep.y = axis_pos_ - rep.GetRotatedHeight()/2; // 中心落在 y
            }
        }

        /* 1-d  更新 bounding box */
        min_x = std::min<std::int64_t>(min_x, rep.x);
        min_y = std::min<std::int64_t>(min_y, rep.y);
        max_x = std::max<std::int64_t>(max_x, rep.x + rep.GetRotatedWidth());
        max_y = std::max<std::int64_t>(max_y, rep.y + rep.GetRotatedHeight());

        if (n->lchild) {
            stk.emplace(n->lchild);
        }
        if (n->rchild) {
            stk.emplace(n->rchild);
        }
    }

    /* ---------- 2) 平移全島到 (0,0) ---------- */
    const std::int64_t dx = -min_x;
    const std::int64_t dy = -min_y;

    for (int id: block_ids_) {
        blocks[id].x += dx;
        blocks[id].y += dy;
    }

    bbox_w_ = max_x - min_x;
    bbox_h_ = max_y - min_y;

    // 根據對稱軸方向正確更新軸位置
    if (group_->axis == Axis::kVertical) {
        axis_pos_ += dx;  // 垂直對稱軸，x軸平移
    } else {
        axis_pos_ += dy;  // 水平對稱軸，y軸平移
    }

    if (connect_node) {
        if (group_->axis == Axis::kVertical) {
           connect_node->rchild = nullptr;
        } else {
           connect_node->lchild = nullptr;
        }
    }
}

void AsfIsland::Mirror(std::vector<Block>& blocks) {
    if (group_->axis == Axis::kVertical) {
        group_->axis = Axis::kHorizontal;
    } else {
        group_->axis = Axis::kVertical;
    }
    for (auto id: block_ids_) {
        blocks[id].Rotate();
    }
    MirrorTree(bs_tree_.root);
}

int AsfIsland::GetNumberNodes() const {
    return all_represent_nodes_.size();
}

RotateNodeOp AsfIsland::RotateNodeRandomize(std::vector<Block>& blocks) {
    RotateNodeOp op;
    op.Apply(blocks, all_represent_nodes_);
    return op;
}

SwapNodeOp AsfIsland::SwapNodeRandomize() {
    SwapNodeOp op;
    op.Apply(&pair_root_, pair_represent_nodes_);
    return op;
}

LeafMoveOp AsfIsland::MoveLeafNodeRandomize() {
    LeafMoveOp op;
    op.Apply(pair_root_);
    return op;
}
