#include "ASFIsland.hpp"
#include <cassert>
#include <algorithm>
#include <climits>


using NodeInt = Node<int64_t>;

/// buildInitialSolution 把 pairReps 构成一棵平衡树，
/// 再把 selfReps 串到最 “极端” 的那条分支上
void ASFIsland::buildInitialSolution(
    const std::vector<NodeInt*>& pairReps,
    const std::vector<NodeInt*>& selfReps,
    std::vector<NodeInt*>& preorder,
    std::vector<NodeInt*>& inorder)
{
    // 1. 只用 pairReps 先做平衡树
    auto sorted = pairReps;
    std::sort(sorted.begin(), sorted.end(),
              [](auto a, auto b){
                  return a->width * a->height > b->width * b->height;
              });
    std::function<NodeInt*(int,int)> buildBalanced = [&](int l, int r)->NodeInt* {
        if (l > r) return nullptr;
        int m = (l + r) / 2;
        NodeInt* node = sorted[m];
        node->lchild = buildBalanced(l, m - 1);
        node->rchild = buildBalanced(m + 1, r);
        return node;
    };
    NodeInt* root = buildBalanced(0, (int)sorted.size() - 1);

    // 2. 把所有 selfReps 串成一条链
    for (NodeInt* s : selfReps) {
        s->lchild = s->rchild = nullptr;
        if (sg->axis == VERTICAL) {
            // vertical branch → right‐child 链
            NodeInt* cur = root;
            while (cur->rchild) cur = cur->rchild;
            cur->rchild = s;
        } else {
            // horizontal branch → left‐child 链
            NodeInt* cur = root;
            while (cur->lchild) cur = cur->lchild;
            cur->lchild = s;
        }
    }

    // 3. 从这棵定好的树提 preorder + inorder
    std::function<void(NodeInt*)> dfs_pre = [&](NodeInt* u){
        if (!u) return;
        preorder.push_back(u);
        dfs_pre(u->lchild);
        dfs_pre(u->rchild);
    };
    std::function<void(NodeInt*)> dfs_in = [&](NodeInt* u){
        if (!u) return;
        dfs_in(u->lchild);
        inorder.push_back(u);
        dfs_in(u->rchild);
    };
    dfs_pre(root);
    dfs_in(root);
}

void ASFIsland::build(const std::unordered_map<std::string,int>& idx,
                      const std::vector<Block>& blocks)
{
    // -------- 1. 先準備兩類代表 --------
    std::vector<NodeInt*> pairReps;
    std::vector<NodeInt*> selfReps;

    // (a) symmetry‑pair：固定使用右側模組 b' 當代表
    for (const auto& p : sg->pairs) {
        int id = idx.at(p.b);                          // p.b = right member
        pairReps.push_back(new NodeInt());
        pairReps.back()->setShape(blocks[id].w, blocks[id].h);
        pairReps.back()->blockId = id;  // 記錄代表的模組 id
        blockIds.push_back(id);
        blockIds.push_back(idx.at(p.a));
    }
    // (b) self‑symmetric：取右(上)半；width/height 擇一對半
    for (const auto& s : sg->selfs) {
        int id = idx.at(s.a);
        int halfW = (sg->axis == VERTICAL) ? blocks[id].w / 2 : blocks[id].w;
        int halfH = (sg->axis == HORIZONTAL) ? blocks[id].h / 2 : blocks[id].h;
        selfReps.push_back(new NodeInt());
        selfReps.back()->setShape(halfW, halfH);
        selfReps.back()->blockId = id;  // 記錄代表的模組 id
        blockIds.push_back(id);  // 記錄代表的模組 id
    }
    
    vector<NodeInt*> modules = pairReps;
    modules.insert(modules.end(), selfReps.begin(), selfReps.end());

    vector<NodeInt*> preorder, inorder;
    // buildInitialSolution(modules, preorder, inorder);
    buildInitialSolution(pairReps, selfReps, preorder, inorder);
    bst.buildTree(preorder, inorder);  // 建立 BST

}


/********************************************************************
 *  pack()  —  1) 代表半平面打包          (bst.setPosition)
 *              2) 鏡射 mate / self 模組  (式 (1)(2))
 *              3) 校正 bbox 置於 (0,0)
 ********************************************************************/
void ASFIsland::pack(std::vector<Block>& blocks,
                     const std::unordered_map<std::string,int>& idx)
{   
    
    /* ---------- 0) 打包代表半平面 ---------- */
    bst.setPosition();                // 前序 + contour → 得 (x,y)

    /* ---------- 1) 掃描代表並鏡射 ---------- */
    const int64_t ax = axisPos;           // 垂直→x̂ , 水平→ŷ
    int64_t minX=LLONG_MAX, minY=LLONG_MAX;
    int64_t maxX=LLONG_MIN, maxY=LLONG_MIN;

    std::vector<NodeInt*> stk{bst.root};
    while(!stk.empty()){
        NodeInt* n = stk.back(); stk.pop_back();
        Block& rep = blocks[n->blockId];

        /* 1-a  設定代表座標 */
        rep.x = n->x;
        rep.y = n->y;
        // rep.rot = false;                  // 若有旋轉資訊可在 Node 存

        /* 1-b  處理 symmetry-pair 的另一半 */
        auto itP = std::find_if(sg->pairs.begin(), sg->pairs.end(),
                   [&](const SymPair& p){ return p.b == rep.name; });
        if(itP != sg->pairs.end()){
            int mateId = idx.at(itP->a);
            Block& mate = blocks[mateId];
            // mate.rot = rep.rot;

            if(sg->axis == VERTICAL){
                mate.x = 2 * ax - rep.x - rep.w;   // 式 (1)
                mate.y = rep.y;
            }else{
                mate.x = rep.x;
                mate.y = 2 * ax - rep.y - rep.h;   // 式 (2)
            }

            
            // 也要把 mate 也納入 bounding‐box 更新
            minX = std::min<int64_t>(minX, mate.x);
            minY = std::min<int64_t>(minY, mate.y);
            maxX = std::max<int64_t>(maxX, mate.x + mate.w);
            maxY = std::max<int64_t>(maxY, mate.y + mate.h);
        }

        /* 1-c  self-symmetric：置中於軸 */
        auto itS = std::find_if(sg->selfs.begin(), sg->selfs.end(),
                   [&](const SymSelf& s){ return s.a == rep.name; });
        if(itS != sg->selfs.end()){
            if(sg->axis == VERTICAL){
                rep.x = ax - rep.w/2;            // 中心落在 x̂
            }else{
                rep.y = ax - rep.h/2;            // 中心落在 ŷ
            }
        }

        /* 1-d  更新 bounding box */
        minX = std::min<int64_t>(minX, rep.x);
        minY = std::min<int64_t>(minY, rep.y);
        maxX = std::max<int64_t>(maxX, rep.x + rep.w);
        maxY = std::max<int64_t>(maxY, rep.y + rep.h);

        if(n->lchild) stk.push_back(n->lchild);
        if(n->rchild) stk.push_back(n->rchild);
    }

    /* ---------- 2) 平移全島到 (0,0) ---------- */
    const int64_t dx = -minX;
    const int64_t dy = -minY;

    for (int id : blockIds) {
        Block& b = blocks[id];
        b.x += dx;
        b.y += dy;
    }

    bboxW = maxX - minX;
    bboxH = maxY - minY;

    // 根據對稱軸方向正確更新軸位置
    if(sg->axis == VERTICAL) {
        axisPos += dx;  // 垂直對稱軸，x軸平移
    } else {
        axisPos += dy;  // 水平對稱軸，y軸平移
    }
}


