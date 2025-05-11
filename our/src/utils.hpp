#pragma once

#include <cassert>
#include <cstdint>
#include <chrono>
#include <limits>
#include <random>
#include <thread>
#include <vector>
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

    // Special generator used to fast init magic numbers.
    // Output values only have 1/8th of their bits set on average.
    constexpr std::uint64_t SparseRand() { return Rand64() & Rand64() & Rand64(); }

    virtual std::uint64_t operator()() { return Rand64(); }

private:
    std::uint64_t s_;
};

inline uint64_t NowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(
            high_resolution_clock::now().time_since_epoch()).count();
}

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
inline void GatherAllLeafNodesF(NodePointer node, std::vector<NodePointer> &buf) {
    if (node) {
        if (!node->lchild && !node->rchild) {
            buf.emplace_back(node);
        } else {
            GatherAllLeafNodesF(node->lchild, buf);
            GatherAllLeafNodesF(node->rchild, buf);
        }
    }
}


struct LeafMoveOp {
    NodePointer leaf;
    NodePointer old_parent;
    bool was_left_child;

    NodePointer new_parent;
    bool inserted_as_left;

    void MoveLeafNodeOnce(NodePointer root) {
        std::function<void(NodePointer, std::vector<NodePointer>&)> GatherAllLeafNodes =
            [] (NodePointer node, std::vector<NodePointer> &buf) {
            if (node) {
                if (!node->lchild && !node->rchild) {
                    buf.emplace_back(node);
                } else {
                    GatherAllLeafNodes(node->lchild, buf);
                    GatherAllLeafNodes(node->rchild, buf);
                }
            }
        }

        std::vector<NodePointer> leaves;
        GatherAllLeafNodes(root, leaves);

        // 隨機選擇葉節點
        op.leaf = leaves[RandInt(0, (int)leaves.size() - 1)];
        op.old_parent = op.leaf->parent;
        op.was_left_child = (op.old_parent && op.old_parent->lchild == op.leaf);

        // 將葉節點從舊位置移除
        if (op.was_left_child) op.old_parent->lchild = nullptr;
        else                   op.old_parent->rchild = nullptr;
        op.leaf->parent = nullptr;

        // 找可插入的新位置
        std::vector<NodePointer> candidates;
        std::function<void(NodePointer)> Collect = [&](NodePointer node) {
            if (!node) return;
            if (!node->lchild || !node->rchild) {
                if (node != op.leaf) candidates.push_back(node); // 避免插入自己
            }
            Collect(node->lchild);
            Collect(node->rchild);
        };
        Collect(tree.root);

        op.new_parent = candidates[RandInt(0, (int)candidates.size() - 1)];
        op.inserted_as_left = !op.new_parent->lchild;

        // 插入
        if (op.inserted_as_left) op.new_parent->lchild = op.leaf;
        else                     op.new_parent->rchild = op.leaf;
        op.leaf->parent = op.new_parent;

        return op;
    }
};
