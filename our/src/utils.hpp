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
inline void GatherAllLeafNodes(NodePointer node, std::vector<NodePointer> &buf) {
    if (node) {
        if (!node->lchild && !node->rchild) {
            buf.emplace_back(node);
        } else {
            GatherAllLeafNodes(node->lchild, buf);
            GatherAllLeafNodes(node->rchild, buf);
        }
    }
}
inline void GatherAllInsertNodes(NodePointer node, std::vector<NodePointer> &buf) {
    if (node) {
        if (!node->lchild) {
            buf.emplace_back(node);
        }
        if (!node->rchild) {
            buf.emplace_back(node);
        }
        GatherAllInsertNodes(node->lchild, buf);
        GatherAllInsertNodes(node->rchild, buf);
    }
}
