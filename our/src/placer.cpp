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
    SwapNodeOp op = hb_tree_.SwapNodeRandomize();
    if (!op.Valid()) {
        return;
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
    temperature_ = best_area_ / 10.0;
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
        do {
            curr_area_ = best_area_;
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
                              << "] Area = " << best_area_ << std::endl;
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

