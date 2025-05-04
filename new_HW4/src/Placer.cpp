/************************  Placer.cpp  ************************/
#include "Placer.hpp"
/* ---------------------------------------------------------- */
/* ------------------- readInput ---------------------------- */
void Placer::readInput(const std::string& path)
{
    std::ifstream fin(path);
    if(!fin){ throw std::runtime_error("input open failed"); }
    std::string tok; int N;

    /* HardBlock section */
    fin>>tok>>N;                       // NumHardBlocks
    blocks.reserve(N);
    for(int i=0;i<N;++i){
        std::string key, name; int w,h;
        fin>>key>>name>>w>>h;          // HardBlock <name> w h
        blocks.emplace_back(name,w,h);
        blocks.back().gid = -1;
        idmap[name]=i;
    }

    /* SymGroup section */
    int M; fin>>tok>>M; groups.resize(M);
    for(int i=0;i<M;++i){
        SymGroup& sg = groups[i];
        int cnt; fin>>tok>>sg.name>>cnt;   // SymGroup sg0 2
        sg.axis = (i&1)?HORIZONTAL:VERTICAL;  // 若 input 沒指明, 交替
        // sg.axis = VERTICAL;
        for(int j=0;j<cnt;++j){
            std::string kind; fin>>kind;
            if(kind=="SymPair"){
                SymPair sp; fin>>sp.a>>sp.b; sg.pairs.push_back(sp);
            }else if(kind=="SymSelf"){
                SymSelf ss; fin>>ss.a;       sg.selfs.push_back(ss);
            }
        }
    }
}

/* ------------------- build ---------------------------- */
void Placer::build()
{
    /* 1. 建 ASFIsland */
    islands.reserve(groups.size());
    std::unordered_set<int> inGroup;
    for(auto& sg: groups){
        ASFIsland* isl = new ASFIsland(&sg);
        isl->build(idmap, blocks);
        islands.push_back(isl);

        /* 給屬於此 island 的 block 加 gid */
        for(auto& p: sg.pairs){
            blocks[idmap[p.a]].gid = blocks[idmap[p.b]].gid = (int)islands.size()-1;
            inGroup.insert(idmap[p.a]); inGroup.insert(idmap[p.b]);
        }
        for(auto& s: sg.selfs){
            blocks[idmap[s.a]].gid = (int)islands.size()-1;
            inGroup.insert(idmap[s.a]);
        }
    }

    /* 2. soloIds (未在任何 group 的) */
    for(int i=0;i<(int)blocks.size();++i)
        if(!inGroup.count(i)) soloIds.push_back(i);

    /* 3. 建 HBTree */
    hb.build(islands, soloIds, blocks, idmap);

    /* 4. 初始解 pack */
    packAll();
}

/* ------------------- run Simulated Annealing ------------ */
void Placer::runSA() {
    const uint64_t DEADLINE = 290'000'000ULL; // 290 s
    uint64_t tStart = nowUs();

    // Parameters
    double T = 20000, T_MIN = 0.1, T_DECAY = 0.95;
    double REJECT_RATIO = 1;
    // 一般是用來表示在每個溫度下要嘗試多少次鄰域解
    int K = 20;
    int N = blocks.size() * K;
    int DOUBLE_N = N * 2;
    int64_t curCost = bestArea; // 目前成本（例如：面積）
    int64_t min_cost = curCost;
    int gen_cnt = 1, uphill_cnt = 0, reject_cnt = 0;

     // Simulated annealing
    do
    {
        gen_cnt = 0, uphill_cnt = 0, reject_cnt = 0;
        do
        {
            if (nowUs() - tStart > DEADLINE){
                cout << "Timeout!" << endl;
                break;
            }
            int moveType = randint(0, 3);
            cout << "MoveType: " << moveType << endl;
            int idx1 = -1, idx2 = -1;
            int64_t newCost = curCost;
            if (moveType == 0) { // 單個 block 旋轉
                idx1 = randint(0, (int)blocks.size()-1);
                blocks[idx1].rot = !blocks[idx1].rot;
                newCost = packAll();
                int64_t delta_cost = newCost - curCost;
                bool rand_accept = (double)rand() / RAND_MAX < exp(-1 * (delta_cost) / T);
                if(delta_cost <= 0 || rand_accept) {
                    if(delta_cost > 0) {
                        uphill_cnt++;
                    }
                    
                    curCost = newCost;
                    if(curCost < min_cost) {
                        min_cost = curCost;
                        bestBlocks = blocks;
                    }
                } else {
                    reject_cnt++;
                    blocks[idx1].rot = !blocks[idx1].rot; // rollback
                }   
            }
            else if (moveType == 1) { // 交換兩個 solo block 的位置
                if (soloIds.size() < 2) continue;
                int a = randint(0, (int)soloIds.size()-1);
                int b;
                do { b = randint(0, (int)soloIds.size()-1); } while (b == a);
                idx1 = soloIds[a]; idx2 = soloIds[b];
                std::swap(blocks[idx1], blocks[idx2]);
                std::swap(idmap[blocks[idx1].name], idmap[blocks[idx2].name]);
                newCost = packAll();
                int64_t delta_cost = newCost - curCost;
                bool rand_accept = (double)rand() / RAND_MAX < exp(-1 * (delta_cost) / T);
                if(delta_cost <= 0 || rand_accept) {
                    if(delta_cost > 0) {
                        uphill_cnt++;
                    }
                    
                    curCost = newCost;
                    if(curCost < min_cost) {
                        min_cost = curCost;
                        bestBlocks = blocks;
                    }
                } else {
                    reject_cnt++;
                    std::swap(blocks[idx1], blocks[idx2]);
                    std::swap(idmap[blocks[idx1].name], idmap[blocks[idx2].name]);
                    packAll();
                }
            }
            else if (moveType == 2) { // 交換兩個 island 的代表節點
                if (islands.size() < 2) continue;
                int a = randint(0, (int)islands.size()-1);
                int b;
                do { b = randint(0, (int)islands.size()-1); } while (b == a);
                // 交換 island 指標，並更新各 island 內所有 block 的 gid
                std::swap(islands[a], islands[b]);
                for (size_t i = 0; i < islands.size(); ++i) {
                    for (int bid : islands[i]->blockIds)
                        blocks[bid].gid = int(i);
                }
                newCost = packAll();
                int64_t delta_cost = newCost - curCost;
                bool rand_accept = (double)rand() / RAND_MAX < exp(-1 * (delta_cost) / T);
                if(delta_cost <= 0 || rand_accept) {
                    if(delta_cost > 0) {
                        uphill_cnt++;
                    }
                    
                    curCost = newCost;
                    if(curCost < min_cost) {
                        min_cost = curCost;
                        bestBlocks = blocks;
                    }
                } else {
                    reject_cnt++;
                    std::swap(islands[a], islands[b]);
                    for (size_t i = 0; i < islands.size(); ++i) {
                        for (int bid : islands[i]->blockIds)
                            blocks[bid].gid = int(i);
                    }
                    packAll();
                }
            }
            else if (moveType == 3) { // 對某個 island 進行局部擾動
                int idx = randint(0, (int)islands.size()-1);
                if (!islands[idx]->blockIds.empty()) {
                    // 隨機交換 island 內兩個 block 的順序
                    int i1 = randint(0, (int)islands[idx]->blockIds.size()-1);
                    int i2;
                    do { i2 = randint(0, (int)islands[idx]->blockIds.size()-1); } while (i2 == i1);
                    std::swap(islands[idx]->blockIds[i1], islands[idx]->blockIds[i2]);
                    // 重新 pack 觸發內部 bbox 與 topSegs 的更新
                    islands[idx]->pack(blocks, idmap);
                    newCost = packAll();
                    int64_t delta_cost = newCost - curCost;
                    bool rand_accept = (double)rand() / RAND_MAX < exp(-1 * (delta_cost) / T);
                    if(delta_cost <= 0 || rand_accept) {
                        if(delta_cost > 0) {
                            uphill_cnt++;
                        }
                        
                        curCost = newCost;
                        if(curCost < min_cost) {
                            min_cost = curCost;
                            bestBlocks = blocks;
                        }
                    } else {
                        reject_cnt++;
                        std::swap(islands[idx]->blockIds[i1], islands[idx]->blockIds[i2]);
                        islands[idx]->pack(blocks, idmap);
                        packAll();
                    }
                }
            }
            gen_cnt++;
        } while (uphill_cnt <= N && gen_cnt <= DOUBLE_N);
        // Reduce temperature
        T *= T_DECAY;
    } while ((double)reject_cnt / gen_cnt <= REJECT_RATIO && T >= T_MIN );


    blocks.swap(bestBlocks);
    bestArea = min_cost;
}

/* ------------------- writeOutput ------------------------ */
void Placer::writeOutput(const std::string& path)
{
    std::ofstream fout(path);
    fout<<"Area "<<bestArea<<"\n\n";
    fout<<"NumHardBlocks "<<blocks.size()<<"\n";
    for(auto& b: blocks){
        fout<<b.name<<" "<<b.x<<" "<<b.y<<" "<<(b.rot?1:0)<<"\n";
    }
    std::cout<<"[INFO] final area = "<<bestArea<<"\n";
}