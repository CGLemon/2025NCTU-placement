#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <cstdint>

#include "types.hpp"
#include "hb_tree.hpp"

class Placer {
public:
    Placer() = default;
    void ReadFile(const std::string& path);
    void RunSimulatedAnnealing();
    void WriteFile(const std::string& path);

private:
    std::int64_t ComputeArea(std::vector<Block>& blocks);
    std::int64_t ComputeTotalWirelength(const std::vector<Block>& blocks);
    void ComputeBaseFactor(std::vector<Block>& blocks);
    std::int64_t ComputeCost(std::vector<Block>& blocks);
    void UpdateCostFactorStage();

    bool TryAcceptSimulation(double delta_area);
    int TryGetSymmMate(int idx) const;
    void RotateNode();
    void SwapNode();
    void SwapOrRotateGroupNode();
    void MoveLeafNode();
    void UpdateStats();

    bool ShouldStopRound() const;
    bool ShouldStopRunning() const;

    std::vector<Block> blocks_;       // 所有 HardBlock
    std::vector<SymmGroup> groups_;   // 對稱群
    NameToIdMap blockname_to_id_map_; // block name -> idx

    std::vector<Block> best_blocks_;  // 最好的 HardBlock
    HbTree hb_tree_;

    double temperature_;
    std::int64_t best_cost_;
    std::int64_t curr_cost_;
    std::int64_t best_area_;

    std::int64_t base_area_;
    std::int64_t base_hpwl_;

    bool found_bestcost_;
    int not_found_bestcost_accum_;
    int beta_reduction_stage_;

    int num_simulations_;
    int num_iterations_;

    int gen_cnt_;
    int reject_cnt_;
    int uphill_cnt_;
    bool stop_;
};

