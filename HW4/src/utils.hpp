#pragma once

#include <cassert>
#include <cstdint>
#include <chrono>
#include <limits>
#include <random>
#include <thread>
#include <vector>
#include <functional>
#include <unordered_set>

#include "types.hpp"

class RandomBase {
public:
    // The interface for STL.
    using result_type = std::uint64_t;

    constexpr static result_type min() {
        return std::numeric_limits<result_type>::min();
    }

    constexpr static result_type max() {
        return std::numeric_limits<result_type>::max();
    }

    template<int Range>
    int RandomFix() {
        return (int)(*this)() / Range;
    }

    virtual result_type operator()() = 0;
};

/*
 * xorshift64star Pseudo-Random Number Generator
 * This class is based on original code written and dedicated
 * to the public domain by Sebastiano Vigna (2014).
 * It has the following characteristics:
 *
 *  -  Outputs 64-bit numbers
 *  -  Passes Dieharder and SmallCrush test batteries
 *  -  Does not require warm-up, no zeroland to escape
 *  -  Internal state is a single 64-bit integer
 *  -  Period is 2^64 - 1
 *  -  Speed: 1.60 ns/call (Core i7 @3.40GHz)
 *
 * For further analysis see
 *   <http://vigna.di.unimi.it/ftp/papers/xorshift.pdf>
 */
class PRNG : public RandomBase {
public:
    PRNG(std::uint64_t seed) : s_(seed) { assert(seed); }

    static PRNG &Get() {
        std::random_device rd_;
        static thread_local PRNG rng(rd_());
        return rng;
    }

    constexpr std::uint64_t Rand64() {
        s_ ^= s_ >> 12;
        s_ ^= s_ << 25;
        s_ ^= s_ >> 27;
        return s_ * 2685821657736338717ULL;
    }

    constexpr std::uint64_t GetSeed() const {
        return s_;
    }
    constexpr void SetSeed(std::uint64_t seed) {
        s_ = seed;
    }
    

    // Special generator used to fast init magic numbers.
    // Output values only have 1/8th of their bits set on average.
    constexpr std::uint64_t SparseRand() { return Rand64() & Rand64() & Rand64(); }

    virtual std::uint64_t operator()() { return Rand64(); }

private:
    std::uint64_t s_;
};

class Timer {
public:
    Timer() {
        Clock();
    }
    void Clock() {
        clock_time_ = std::chrono::steady_clock::now();
    }

    int GetDurationSeconds() const {
        const auto end_time =
            std::chrono::steady_clock::now();
        const auto seconds =
            std::chrono::duration_cast<std::chrono::seconds>(end_time - clock_time_).count();
        return seconds;
    }
    int GetDurationMilliseconds() const {
        const auto end_time =
            std::chrono::steady_clock::now();
        const auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - clock_time_).count();
        return milliseconds;
    }

private:
    std::chrono::steady_clock::time_point clock_time_;
};


inline int RandInt(int l, int r) {
    std::uniform_int_distribution<int> dist(l,r);
    return dist(PRNG::Get());
}
inline double Rand01() {
    std::uniform_real_distribution<double> dist(0,1);
    return dist(PRNG::Get());
}
inline std::vector<int> RandSample(int l, int r, int size) {
    std::vector<int> result;
    std::unordered_set<int> seen;
    do {
        result.clear();
        seen.clear();

        for (int i = 0; i < size; ++i) {
            int val = RandInt(l, r);
            result.push_back(val);
            seen.insert(val);
        }
    } while ((int)seen.size() < size);  // 如果有重複，就重試
    return result;
}
inline std::uint64_t GetCurrentSeed() {
    return PRNG::Get().GetSeed();
}
inline void SetCurrentSeed(std::uint64_t seed) {
    PRNG::Get().SetSeed(seed);
}

inline NodePointer BuildBalancedTree(NodePointerList& nodes) {
    std::function<NodePointer(NodePointer, int, int)> BuildBalanced = 
        [&](NodePointer parent, int l, int r) -> NodePointer {
            if (l > r) return nullptr;
            int m = (l + r) / 2;
            NodePointer node = nodes[m];
            node->parent = parent;
            node->lchild = BuildBalanced(node, l, m - 1);
            node->rchild = BuildBalanced(node, m + 1, r);
            return node;
        };
    NodePointer root = BuildBalanced(nullptr, 0, (int)nodes.size() - 1);
    return root;
}
inline NodePointer BuildLeftSkewedTree(NodePointerList& nodes) {
    std::function<NodePointer(NodePointer, int)> BuildLeftSkewed =
        [&](NodePointer parent, int idx) -> NodePointer {
            if (idx >= (int)nodes.size()) return nullptr;
            NodePointer node = nodes[idx];
            node->parent = parent;
            node->lchild = BuildLeftSkewed(node, idx + 1);
            node->rchild = nullptr;
            return node;
        };
    NodePointer root = BuildLeftSkewed(nullptr, 0);
    return root;
}
inline NodePointer BuildRightSkewedTree(NodePointerList& nodes) {
    std::function<NodePointer(NodePointer, int)> BuildRightSkewed =
        [&](NodePointer parent, int idx) -> NodePointer {
            if (idx >= (int)nodes.size()) return nullptr;
            NodePointer node = nodes[idx];
            node->parent = parent;
            node->lchild = nullptr;
            node->rchild = BuildRightSkewed(node, idx + 1);
            return node;
        };
    NodePointer root = BuildRightSkewed(nullptr, 0);
    return root;
}
inline void ReplaceParentChild(NodePointer parent,
                               NodePointer old_child,
                               NodePointer new_child) {
    if (!parent) {
        return;
    }
    if (parent->lchild == old_child) {
        parent->lchild = new_child;
    }
    if (parent->rchild == old_child) {
        parent->rchild = new_child;
    }
}
inline void SwapNodeDirection(NodePointer src, NodePointer dst) {
    // 更新 parent 指向
    if (src->parent != dst->parent) {
        ReplaceParentChild(src->parent, src, dst);
        ReplaceParentChild(dst->parent, dst, src);
    } else {
        std::swap(src->parent->lchild, dst->parent->rchild);
    }

    // 交換  parent lchild 與 rchild
    std::swap(src->parent, dst->parent);
    std::swap(src->lchild, dst->lchild);
    std::swap(src->rchild, dst->rchild);

    // 更新孩子們的 parent 指向
    if (src->lchild) src->lchild->parent = src;
    if (src->rchild) src->rchild->parent = src;
    if (dst->lchild) dst->lchild->parent = dst;
    if (dst->rchild) dst->rchild->parent = dst;
}
inline void MirrorTree(NodePointer n) {
    if (n) {
        std::swap(n->lchild, n->rchild);
        MirrorTree(n->lchild);
        MirrorTree(n->rchild);
    }
}

class RotateNodeOp {
public:
    void Apply(std::vector<Block>& blocks, NodePointerList& nodes) {
        num_nodes_ = nodes.size();
        if (!Valid()) {
            return;
        }
        NodePointer n = nodes[RandInt(0, num_nodes_ - 1)];
        block_ = &blocks[n->blockId];
        block_->Rotate();
    }
    void Undo() {
        if (!Valid()) {
            return;
        }
        block_->Rotate();
    }
    bool Valid() const {
        return num_nodes_ >= 1;
    }
private:
    int num_nodes_{0};
    Block * block_{nullptr};
};

class SwapNodeOp {
public:
    void Apply(NodePointer *root, NodePointerList& nodes) {
        num_nodes_ = nodes.size();
        if (!Valid()) {
            return;
        }

        auto buf = RandSample(0, num_nodes_-1, 2);
        src_ = nodes[buf[0]];
        dst_ = nodes[buf[1]];
        root_ = root;

        if (*root_ == src_) {
            *root_ = dst_;
        } else if (*root_ == dst_) {
            *root_ = src_;
        }
        SwapNodeDirection(src_, dst_);
    }
    void Undo() {
        if (!Valid()) {
            return;
        }
        if (*root_ == src_) {
            *root_ = dst_;
        } else if (*root_ == dst_) {
            *root_ = src_;
        }
        SwapNodeDirection(src_, dst_);
    }
    bool Valid() const {
        return num_nodes_ >= 2;
    }

private:
    int num_nodes_{0};
    NodePointer *root_{nullptr};
    NodePointer src_{nullptr}, dst_{nullptr};
};

class LeafMoveOp {
public:
   LeafMoveOp() = default;

    void Apply(NodePointer root) {
        if (!root) {
            return;
        }
        std::function<void(NodePointer, NodePointerList&)> GatherAllLeafNodes =
            [&] (NodePointer node, NodePointerList &buf) {
            if (node) {
                if (!node->lchild && !node->rchild) {
                    buf.emplace_back(node);
                } else {
                    GatherAllLeafNodes(node->lchild, buf);
                    GatherAllLeafNodes(node->rchild, buf);
                }
            }
        };
        NodePointerList leaves;
        GatherAllLeafNodes(root, leaves);

        // 隨機選擇葉節點
        leaf_ = leaves[RandInt(0, (int)leaves.size() - 1)];
        old_parent_ = leaf_->parent;
        was_left_child_ =
            (old_parent_ && old_parent_->lchild == leaf_);

        // 將葉節點從舊位置移除
        if (was_left_child_) {
            old_parent_->lchild = nullptr;
        } else {
            old_parent_->rchild = nullptr;
        }
        leaf_->parent = nullptr;

        // 找可插入的新位置
        NodePointerList candidates;
        std::function<void(NodePointer, NodePointerList&)> Collect =
            [&] (NodePointer node, NodePointerList &buf) {
            if (!node) return;
            if (!node->lchild || !node->rchild) {
                if (node != leaf_) { candidates.push_back(node); } // 避免插入自己
            }
            Collect(node->lchild, buf);
            Collect(node->rchild, buf);
        };
        Collect(root, candidates);

        if (candidates.empty()) {
            return;
        }
        new_parent_ = candidates[RandInt(0, (int)candidates.size() - 1)];
        inserted_as_left_ = !new_parent_->lchild;

        // 插入
        if (inserted_as_left_) {
            new_parent_->lchild = leaf_;
        } else {
            new_parent_->rchild = leaf_;
        }
        leaf_->parent = new_parent_;
    }
    void Undo() const {
        // 從新位置移除
        if (inserted_as_left_) {
            new_parent_->lchild = nullptr;
        } else {
            new_parent_->rchild = nullptr;
        }

        // 還原到舊位置
        if (was_left_child_) {
            old_parent_->lchild = leaf_;
        } else {
            old_parent_->rchild = leaf_;
        }
        leaf_->parent = old_parent_;
    }
    bool Valid() const {
        return new_parent_ != nullptr;
    }

private:
    NodePointer leaf_{nullptr};
    NodePointer old_parent_{nullptr};
    bool was_left_child_{false};

    NodePointer new_parent_{nullptr};
    bool inserted_as_left_{false};
};
