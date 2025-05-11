#include <algorithm> 
#include <functional>
#include "hb_tree.hpp"
#include "utils.hpp"

void HbTree::Initialize(std::vector<Block> &blocks,
                        std::vector<SymmGroup> &groups) {

    const int bsize = blocks.size();
    for (int i = 0; i < bsize; ++i) {
        auto &block = blocks[i];
        if (block.IsSolo()) {
            solo_nodes_.emplace_back(new NodeType());
            solo_nodes_.back()->blockId = i;
        }
    }
    const int gsize = groups.size();
    for (int i = 0; i < gsize; ++i) {
        auto &group = groups[i];
        hier_nodes_.emplace_back(new NodeType());
        hier_nodes_.back()->blockId = i;

        islands_.emplace_back(std::make_unique<AsfIsland>(&group));
        islands_.back()->Initialize(blocks);
    }
    UpdateNodes(blocks);
    BuildInitialSolution();
}

void HbTree::UpdateNodes(const std::vector<Block> &blocks) {
    for (NodePointer n: solo_nodes_) {
        auto &block = blocks[n->blockId];
        n->setShape(
            block.GetRotatedWidth(),
            block.GetRotatedHeight()
        );
    }
    for (NodePointer n: hier_nodes_) {
        auto &island = islands_[n->blockId];
        n->setShape(
            island->GetWidth(),
            island->GetHeight()
        );
    }
}

void HbTree::BuildInitialSolution() {
    NodePointerList sorted;
    sorted.reserve(solo_nodes_.size() + hier_nodes_.size());
    sorted.insert(std::end(sorted), std::begin(solo_nodes_), std::end(solo_nodes_));
    sorted.insert(std::end(sorted), std::begin(hier_nodes_), std::end(hier_nodes_));

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
    bs_tree_.root = BuildBalanced(nullptr, 0, (int)sorted.size() - 1);
}

std::int64_t HbTree::PackAndGetArea(std::vector<Block> &blocks) {
    // 重新 pack 每個 island 內部 （如果 island 內部也要重新 SA 擾動）
    for (auto &isl: islands_) {
        isl->Pack(blocks);
    }

    // 1. 用 B*-Tree 計算全局 (x,y)
    UpdateNodes(blocks);
    bs_tree_.setPosition();

    // 2. 把每個 symmetry island 的 local pack 結果平移到全局座標
    //    hier_nodes_[i] 對應 islands_[i]
    for (size_t i = 0; i < hier_nodes_.size(); ++i) {
        NodePointer n = hier_nodes_[i];
        std::int64_t dx = n->x;
        std::int64_t dy = n->y;
        // 每個 ASFIsland 內部都已經在 pack() 時設定好 local (0,0) 開始的
        // blocks 座標，我們只要把它們往 (dx,dy) 平移就能到全局位置
        for (const auto& id: islands_[i]->GetBlockIds()) {
            // 假設 ASFIsland 暴露了一個 map<string,int> 叫 localIdxMap
            // 其 value 就是對應到全域 blocks 的索引
            blocks[id].x += dx;
            blocks[id].y += dy;
        }
    }

    // 3. 放 solo blocks
    //    solo_nodes_[i] 對應 solo_ids[i]
    for (size_t i = 0; i < solo_nodes_.size(); ++i) {
        NodePointer n = solo_nodes_[i];
        blocks[n->blockId].x = n->x;
        blocks[n->blockId].y = n->y;
    }

    // 4. 回傳整個排版面積
    return bs_tree_.getArea();
}

int HbTree::GetNumberNodes() const {
    return solo_nodes_.size() + hier_nodes_.size();
}

bool HbTree::IsSoloNode(const int idx) const {
    return idx < (int)solo_nodes_.size();
}

NodePointer HbTree::GetNode(int idx) {
    if (idx < (int)solo_nodes_.size()) {
        return solo_nodes_[idx];
    }
    idx -= (int)solo_nodes_.size();
    if (idx < (int)hier_nodes_.size()) {
        return hier_nodes_[idx];
    }
    return nullptr;
}

AsfIsland * HbTree::GetIsland(int idx) {
    if (idx < (int)islands_.size()) {
        return islands_[idx].get();
    }
    return nullptr;
}

void HbTree::RotateNode(std::vector<Block> &blocks, const int idx) {
    NodePointer n = GetNode(idx);

    if (IsSoloNode(idx)) {
        blocks[n->blockId].Rotate();
    } else {
        islands_[n->blockId]->Mirror(blocks);
    }
}

void HbTree::SwapNode(const int src_idx, const int dst_idx) {
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
