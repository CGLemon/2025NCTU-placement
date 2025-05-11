#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <limits>
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
        group.axis = Axis::kVertical;
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
    best_blocks_ = blocks_;
    best_cost_ = ComputeCost(best_blocks_);
}

void Placer::WriteFile(const std::string& path) {
    std::ofstream fout(path);
    std::int64_t best_area = ComputeArea(best_blocks_);

    fout << "Area " << best_area << "\n\n";
    fout << "NumHardBlocks " << best_blocks_.size() << "\n";
    for (auto& b: best_blocks_ ){
        fout << b.name << " " << b.x << " " << b.y << " " << (b.rotated ? 1 : 0) << "\n";
    }
    std::cerr << "[INFO] final area = " << best_area << "\n";
}

std::int64_t Placer::ComputeArea(std::vector<Block>& blocks) {
    return hb_tree_.PackAndGetArea(blocks);
}

std::int64_t Placer::ComputeTotalWirelength(const std::vector<Block>& blocks) {
    std::int64_t min_x = std::numeric_limits<std::int64_t>::max();
    std::int64_t min_y = std::numeric_limits<std::int64_t>::max();
    std::int64_t max_x = std::numeric_limits<std::int64_t>::min();
    std::int64_t max_y = std::numeric_limits<std::int64_t>::min();

    for (const auto& block : blocks) {
        std::int64_t center_x = block.x + block.GetRotatedWidth() / 2;
        std::int64_t center_y = block.y + block.GetRotatedHeight() / 2;

        min_x = std::min(min_x, center_x);
        max_x = std::max(max_x, center_x);
        min_y = std::min(min_y, center_y);
        max_y = std::max(max_y, center_y);
    }

    return (max_x - min_x) + (max_y - min_y); // HPWL
}

void Placer::ComputeBaseArea(std::vector<Block>& blocks) {
    base_area_ = ComputeArea(blocks);
    base_hpwl_ = ComputeTotalWirelength(blocks);
}

std::int64_t Placer::ComputeCost(std::vector<Block>& blocks) {
    constexpr double kAlpha = 0.0;
    constexpr double kBeta = 1.0;
    std::int64_t norm_area = ComputeArea(blocks);
    std::int64_t norm_hpwl = ComputeTotalWirelength(blocks);

    if (base_area_ > base_hpwl_) {
        norm_hpwl = std::round(norm_hpwl * ((double)base_area_/base_hpwl_));
    } else {
        norm_area = std::round(norm_area * ((double)base_hpwl_/base_area_));
    }
    return std::round(kAlpha * norm_area + kBeta * norm_hpwl);
}

bool Placer::TryAcceptSimulation(double delta_cost) {
    bool accept = false;
    if (delta_cost <= 0) {
        accept = true;  // 面積下降，直接接受
    } else if (temperature_ > 0) {
        double prob = std::exp(-1.0 * delta_cost / temperature_);
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

    std::int64_t new_cost = ComputeCost(blocks_);
    std::int64_t delta_cost = new_cost - curr_cost_;

    if (TryAcceptSimulation(delta_cost)) {
        curr_cost_ = new_cost;
        if (new_cost < best_cost_) {
            best_cost_ = new_cost;
            best_blocks_ = blocks_;
        }
        if (delta_cost > 0) {
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
    SwapNodeOp op = hb_tree_.SwapNodeRandomize();
    if (!op.Valid()) {
        return;
    }
    std::int64_t new_cost = ComputeCost(blocks_);
    std::int64_t delta_cost = new_cost - curr_cost_;

    if (TryAcceptSimulation(delta_cost)) {
        curr_cost_ = new_cost;
        if (new_cost < best_cost_) {
            best_cost_ = new_cost;
            best_blocks_ = blocks_;
        }
        if (delta_cost > 0) {
            uphill_cnt_++;
        }
    } else {
        op.Undo();
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
    RotateNodeOp rot_op;
    SwapNodeOp swap_op;
    LeafMoveOp move_op;


    int idx = RandInt(0, (int)groups_.size()-1);
    int select_op = RandInt(0, 2);

    if (select_op == 0) {
        rot_op = hb_tree_.GetIsland(idx)->RotateNodeRandomize(blocks_);
        if (!rot_op.Valid()) {
            return;
        }
    } else if (select_op == 1) {
        swap_op = hb_tree_.GetIsland(idx)->SwapNodeRandomize();
        if (!swap_op.Valid()) {
            return;
        }
    } else if (select_op == 2) {
        move_op = hb_tree_.GetIsland(idx)->MoveLeafNodeRandomize();
        if (!move_op.Valid()) {
            return;
        }
    }

    std::int64_t new_cost = ComputeCost(blocks_);
    std::int64_t delta_cost = new_cost - curr_cost_;

    if (TryAcceptSimulation(delta_cost)) {
        curr_cost_ = new_cost;
        if (new_cost < best_cost_) {
            best_cost_ = new_cost;
            best_blocks_ = blocks_;
        }
        if (delta_cost > 0) {
            uphill_cnt_++;
        }
    } else {
        if (select_op == 0) {
            rot_op.Undo();
        } else if (select_op == 1) {
            swap_op.Undo();
        } else if (select_op == 2) {
            move_op.Undo();
        }
        hb_tree_.PackAndGetArea(blocks_);
        reject_cnt_++;
    }
    num_simulations_++;
    gen_cnt_++;
}

void Placer::MoveLeafNode() {
    LeafMoveOp op = hb_tree_.MoveLeafNodeRandomize();
    if (!op.Valid()) {
        return;
    }

    std::int64_t new_cost = ComputeCost(blocks_);
    std::int64_t delta_cost = new_cost - curr_cost_;

    if (TryAcceptSimulation(delta_cost)) {
        curr_cost_ = new_cost;
        if (new_cost < best_cost_) {
            best_cost_ = new_cost;
            best_blocks_ = blocks_;
        }
        if (delta_cost > 0) {
            uphill_cnt_++;
        }
    } else {
        op.Undo();
        hb_tree_.PackAndGetArea(blocks_);
        reject_cnt_++;
    }
    num_simulations_++;
    gen_cnt_++;
}

void Placer::UpdateStats() {
    if (gen_cnt_ == reject_cnt_) {
        continuous_reject_cnt_ += 1;
    } else {
        continuous_reject_cnt_ = 0;
    }
    gen_cnt_ = 0;
    uphill_cnt_ = 0;
    reject_cnt_ = 0;
}

bool Placer::ShouldReduceTemperature() const {
    return true;
}

bool Placer::ShouldStopRound() const {
    constexpr int K = 200;
    const int kStopFactor = blocks_.size() * K;
    const int kGenerationMin = kStopFactor * 2;
    return stop_ ||
               uphill_cnt_ > kStopFactor ||
               gen_cnt_ > kGenerationMin;
}

bool Placer::ShouldStopRunning() const {
    return stop_ ||
               continuous_reject_cnt_ >= 10 ||
               temperature_ < 1.0;
}

void Placer::RunSimulatedAnnealing() {
    temperature_ = best_cost_ / 10.0;
    num_simulations_ = 0;
    continuous_reject_cnt_ = -1;
    gen_cnt_ = 0;
    uphill_cnt_ = 0;
    reject_cnt_ = 0;
    Timer timer;

    stop_ = false;
    int maxtime_sec = (5 * 60) - 10; // 10 秒當緩衝時間

    do {
        UpdateStats();
        ComputeBaseArea(blocks_);
        do {
            curr_cost_ = best_cost_;
            int move_type = RandInt(0, 3);

            switch (move_type) {
                case 0: RotateNode(); break;
                case 1: SwapNode(); break;
                case 2: SwapOrRotateGroupNode(); break;
                case 3: MoveLeafNode(); break;
                default: ;
            }
            if (num_simulations_ % 1000 == 0) {
                std::cerr << "[Step: " << num_simulations_
                              << "] Area = " << ComputeArea(best_blocks_)
                              << " | HPWL = "
                              << ComputeTotalWirelength(best_blocks_)
                              << " | Cost = "
                              << ComputeCost(best_blocks_)
                              << std::endl;
            }
            if (timer.GetDurationSeconds() >= maxtime_sec) {
                std::cerr << "Time out!" << std::endl;
                stop_ = true;
            }
        } while (!ShouldStopRound());
        std::cerr << "gen_cnt = " << gen_cnt_ << std::endl;
        std::cerr << "uphill_cnt = " << uphill_cnt_ << std::endl;
        std::cerr << "reject_cnt = " << reject_cnt_ << std::endl;
        std::cerr << "continuous_reject_cnt = " << continuous_reject_cnt_ << std::endl;
        std::cerr << "temperature = " << temperature_ << std::endl;

        if (ShouldReduceTemperature()) {
            temperature_ *= 0.95;
        }
    } while (!ShouldStopRunning());
}

