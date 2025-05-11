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
    void ComputeBaseArea(std::vector<Block>& blocks);
    std::int64_t ComputeCost(std::vector<Block>& blocks);

    bool TryAcceptSimulation(double delta_area);
    int TryGetSymmMate(int idx) const;
    void RotateNode();
    void SwapNode();
    void SwapOrRotateGroupNode();
    void MoveLeafNode();
    void UpdateStats();

    bool ShouldReduceTemperature() const;
    bool ShouldStopRound() const;
    bool ShouldStopRunning() const;

    std::vector<Block> blocks_;       // 所有 HardBlock
    std::vector<SymmGroup> groups_;   // 對稱群
    NameToIdMap blockname_to_id_map_; // block name -> idx

    std::vector<Block> best_blocks_;  // 所有 HardBlock
    HbTree hb_tree_;

    double temperature_;
    std::int64_t best_cost_;
    std::int64_t curr_cost_;

    // std::int64_t best_area_;
    // std::int64_t curr_area_;

    std::int64_t base_area_;
    std::int64_t base_hpwl_;

    int num_simulations_;
    int gen_cnt_;
    int reject_cnt_;
    int uphill_cnt_;
    int continuous_reject_cnt_;
    bool stop_;
};

