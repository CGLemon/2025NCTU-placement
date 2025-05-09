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
    std::function<NodePointer(NodePointer, int, int)> BuildBalanced = 
        [&](NodePointer parent, int l, int r) -> NodePointer {
            if (l > r) return nullptr;
            int m = (l + r) / 2;
            NodePointer node = sorted[m];
            node->parent = parent;
            node->lchild = BuildBalanced(node, l, m - 1);
            node->rchild = BuildBalanced(node, m + 1, r);
            return node;
        };

    NodePointer root = BuildBalanced(nullptr, 0, (int)sorted.size() - 1);

    // 2. 把所有 self_represent_nodes 串成一条链
    for (NodePointer s: self_represent_nodes_) {
        s->lchild = s->rchild = nullptr;
        if (!root) { // root 還不存在：第一顆 self represent 當根
            root = s;
            continue;
        }

        if (group_->axis == Axis::kVertical) {
            // vertical branch -> right‐child
            NodePointer curr = root;
            while (curr->rchild) {
                curr = curr->rchild;
            }
            curr->rchild = s;
        } else {
            // horizontal branch -> left‐child
            NodePointer curr = root;
            while (curr->lchild) {
                curr = curr->lchild;
            }
            curr->lchild = s;
        }
    }
    bs_tree_.root = root;
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

/********************************************************************
 *  pack()  —  1) 代表半平面打包          (bst.setPosition)
 *             2) 鏡射 mate / self 模組  (式 (1)(2))
 *             3) 校正 bbox 置於 (0,0)
 ********************************************************************/
void AsfIsland::Pack(std::vector<Block>& blocks) {   
    /* ---------- 0) 打包代表半平面 ---------- */
    UpdateNodes(blocks);
    bs_tree_.setPosition();

    /* ---------- 1) 掃描代表並鏡射 ---------- */
    std::int64_t min_x = LLONG_MAX, min_y = LLONG_MAX;
    std::int64_t max_x = LLONG_MIN, max_y = LLONG_MIN;

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
}

int AsfIsland::GetNumberNodes() const {
    return pair_represent_nodes_.size() + self_represent_nodes_.size();
}

int AsfIsland::GetNumberPairRepresentNodes() const {
    return pair_represent_nodes_.size();
}

NodePointer AsfIsland::GetNode(int idx) {
    if (idx < (int)pair_represent_nodes_.size()) {
        return pair_represent_nodes_[idx];
    }
    idx -= (int)pair_represent_nodes_.size();
    if (idx < (int)self_represent_nodes_.size()) {
        return self_represent_nodes_[idx];
    }
    return nullptr;
}

void AsfIsland::SwapNode(const int src_idx, const int dst_idx) {
    NodePointer src = GetNode(src_idx);
    NodePointer dst = GetNode(dst_idx);

    // 更新  root
    if (bs_tree_.root == src) {
        bs_tree_.root = dst;
    } else if (bs_tree_.root == dst) {
        bs_tree_.root = src;
    }
    SwapNodeDirection(src, dst);
}
