#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <iostream>

#include "placer.hpp"
#include "utils.hpp"

void Placer::ReadFile(const std::string& path) {
    std::ifstream fin(path);
    if (!fin) { 
        throw std::runtime_error("input open failed");
    }
    std::string tok;
    int N;

    /* HardBlock section */
    fin >> tok >> N;
    blocks_.reserve(N);

    for(int i = 0; i < N; ++i) {
        std::string key, name;
        int w, h;

        fin >> key >> name >> w >> h;
        blocks_.emplace_back(name, w, h);
        blocks_.back().gid = -1;
        blockname_to_id_map_[name] = i;
    }

    /* SymGroup section */
    int M;
    fin >> tok >> M;
    groups_.resize(M);

    for (int i = 0 ; i < M; ++i) {
        SymmGroup& group = groups_[i];
        int cnt;
        fin >> tok >> group.name >> cnt;
        // 隨機決定對稱軸
        group.axis = RandInt(0,1) ? Axis::kHorizontal : Axis::kVertical;
        group.gid = i;

        for (int j = 0; j < cnt; ++j) {
            fin >> tok;
            if (tok == "SymPair") {
                SymmPair symm_pair;
                fin >> symm_pair.a >> symm_pair.b;
                symm_pair.aid = blockname_to_id_map_.at(symm_pair.a);
                symm_pair.bid = blockname_to_id_map_.at(symm_pair.b);
                blocks_[symm_pair.aid].gid = i;
                blocks_[symm_pair.bid].gid = i;
                group.pairs.emplace_back(symm_pair);
            } else if (tok == "SymSelf") {
                SymmSelf symm_self;
                fin >> symm_self.a;
                symm_self.id = blockname_to_id_map_.at(symm_self.a);
                blocks_[symm_self.id].gid = i;
                group.selfs.emplace_back(symm_self);
            }
        }
    }

    hb_tree_.Initialize(blocks_, groups_);
    best_area_ = hb_tree_.PackAndGetArea(blocks_);
}

void Placer::WriteFile(const std::string& path) {
    std::ofstream fout(path);
    fout << "Area " << best_area_ << "\n\n";
    fout << "NumHardBlocks " << best_blocks_.size() << "\n";
    for (auto& b: best_blocks_ ){
        fout << b.name << " " << b.x << " " << b.y << " " << (b.rotated ? 1 : 0) << "\n";
    }
    std::cout << "[INFO] final area = " << best_area_ << "\n";
}

bool Placer::TryAcceptWithTemperature(double delta_area) {
    bool accept = false;
    if (delta_area <= 0) {
        accept = true;  // 面積下降，直接接受
    } else if (temperature_ > 0) {
        double prob = std::exp(-1.0 * delta_area / temperature_);
        accept = Rand01() < prob;
    }
    return accept;
}

void Placer::RotateNode() {
    int num_nodes = hb_tree_.GetNumberNodes();
    if (num_nodes < 2) {
        return;
    }

    int rot_id = RandInt(0, num_nodes - 1);
    hb_tree_.RotateNode(blocks_, rot_id);
    std::int64_t new_area = hb_tree_.PackAndGetArea(blocks_);
    std::int64_t delta_area = new_area - curr_area_;

    if (TryAcceptWithTemperature(delta_area)) {
        curr_area_ = new_area;
        if (new_area < best_area_) {
            best_area_ = new_area;
            best_blocks_ = blocks_;
        }
        if (delta_area > 0) {
            uphill_cnt_++;
        }
    } else {
        hb_tree_.RotateNode(blocks_, rot_id);
        hb_tree_.PackAndGetArea(blocks_);
        reject_cnt_++;
    }
    num_simulations_++;
    gen_cnt_++;
}

void Placer::SwapNode() {
    int num_nodes = hb_tree_.GetNumberNodes();
    if (num_nodes < 2) {
        return;
    }

    auto buf = RandSample(0, num_nodes-1, 2);
    int src_idx = buf[0];
    int dst_idx = buf[1];

    hb_tree_.SwapNode(src_idx, dst_idx);
    std::int64_t new_area = hb_tree_.PackAndGetArea(blocks_);
    std::int64_t delta_area = new_area - curr_area_;

    if (TryAcceptWithTemperature(delta_area)) {
        curr_area_ = new_area;
        if (new_area < best_area_) {
            best_area_ = new_area;
            best_blocks_ = blocks_;
        }
        if (delta_area > 0) {
            uphill_cnt_++;
        }
    } else {
        hb_tree_.SwapNode(src_idx, dst_idx);
        hb_tree_.PackAndGetArea(blocks_);
        reject_cnt_++;
    }
    num_simulations_++;
    gen_cnt_++;
}

void Placer::SwapOrRotateGroupNode() {
    if (groups_.empty()) {
        return;
    }

    int idx = RandInt(0, (int)groups_.size()-1);
    bool rot_op = RandInt(0, 1);
    int src_idx = -1, dst_idx = -1;

    if (rot_op) {
        int num_nodes = hb_tree_.GetIsland(idx)->GetNumberNodes();
        src_idx = RandInt(0, num_nodes-1);
        hb_tree_.GetIsland(idx)->RotateNode(blocks_, src_idx);
    } else {
        int num_nodes = hb_tree_.GetIsland(idx)->GetNumberPairRepresentNodes();
        if (num_nodes < 2) {
            return;
        }
        auto buf = RandSample(0, num_nodes-1, 2);
        src_idx = buf[0];
        dst_idx = buf[1];
        hb_tree_.GetIsland(idx)->SwapNode(src_idx, dst_idx);
    }

    std::int64_t new_area = hb_tree_.PackAndGetArea(blocks_);
    std::int64_t delta_area = new_area - curr_area_;

    if (TryAcceptWithTemperature(delta_area)) {
        curr_area_ = new_area;
        if (new_area < best_area_) {
            best_area_ = new_area;
            best_blocks_ = blocks_;
        }
        if (delta_area > 0) {
            uphill_cnt_++;
        }
    } else {
        if (rot_op) {
           hb_tree_.GetIsland(idx)->RotateNode(blocks_, src_idx);
        } else {
           hb_tree_.GetIsland(idx)->SwapNode(src_idx, dst_idx);
        }
        hb_tree_.PackAndGetArea(blocks_);
        reject_cnt_++;
    }
    num_simulations_++;
    gen_cnt_++;
}

void Placer::MoveNode() {

}

void Placer::ResetStats() {
    gen_cnt_ = 0;
    uphill_cnt_ = 0;
    reject_cnt_ = 0;
}

bool Placer::ShouldReduceTemperature() const {
    constexpr int K = 20;
    const int kStopFactor = blocks_.size() * K;
    const int kGenerationMin = kStopFactor * 2;
    return uphill_cnt_ > kStopFactor ||
               gen_cnt_ > kGenerationMin;
}

bool Placer::ShouldStop() const {
    constexpr double kTemperatureMin = 0.1;
    constexpr double kRejectRatio = 1.0;
    return (double)reject_cnt_ / gen_cnt_ > kRejectRatio ||
               temperature_ < kTemperatureMin;
}

void Placer::RunSimulatedAnnealing() {
    temperature_ = best_area_ / 10.0;
    num_simulations_ = 0;

    do {
        ResetStats();
        do {
            curr_area_ = best_area_;
            int move_type = RandInt(0, 2);

            switch (move_type) {
                case 0: RotateNode(); break;
                case 1: SwapNode(); break;
                case 2: SwapOrRotateGroupNode(); break;
                default: ;
            }
            if (num_simulations_ % 1000 == 0) {
                std::cerr << "[Step: " << num_simulations_ << "] Area = " << best_area_ << std::endl;
            }
        } while (!ShouldReduceTemperature());
        temperature_ *= 0.95;
    } while (!ShouldStop());
}

