#include "HBTree.hpp"

using NodeInt = Node<int64_t>;




// 傳入 modules = hierarchy nodes + solo nodes
void HBTree::buildInitialSolution(
    const std::vector<NodeInt*>& modules,
    std::vector<NodeInt*>& preorder,
    std::vector<NodeInt*>& inorder)
{
    // 1. 依照面積（width*height）從大到小排序
    auto sorted = modules;
    std::sort(sorted.begin(), sorted.end(),
              [](auto a, auto b){
                  return (a->width * a->height) > (b->width * b->height);
              });
    // 2. 用中點分割法建一棵平衡樹
    std::function<NodeInt*(int,int)> build = [&](int l, int r)->NodeInt* {
        if (l > r) return nullptr;
        int m = (l + r) / 2;
        NodeInt* node = sorted[m];
        node->lchild = build(l, m - 1);
        node->rchild = build(m + 1, r);
        return node;
    };
    NodeInt* root = build(0, (int)sorted.size() - 1);

    // 3. 由這棵樹分別走一遍 preorder / inorder
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



void HBTree::build(const std::vector<ASFIsland*>& islands,
                   const std::vector<int>& soloIds,
                   std::vector<Block>& blocks, unordered_map<std::string,int>& idx)
{
    
    // 1. hierarchy nodes for symmetry islands
    
    hierNodes.clear();
    hierNodes.reserve(islands.size());
    for (auto* isl : islands) {
        // pack island 以產生內部節點坐標與外接框
        isl->pack(blocks, /* 需要的 idmap，若 HBTree 沒有，可改為傳入或全域存 */ idx);
        // 假設 ASFIsland 提供方法來查外接框大小:
        NodeInt* n = new NodeInt();
        n->setShape(isl->bboxW, isl->bboxH);
        hierNodes.push_back(n);
    }


    // 2. leaf nodes for solo blocks
    soloNodes.clear();
    soloNumber.clear();
    soloNodes.reserve(soloIds.size());
    for (int bid : soloIds) {
        NodeInt* n = new NodeInt();
        n->setShape(blocks[bid].w, blocks[bid].h);
        soloNodes.push_back(n);
        soloNumber.push_back(bid);
    }


    // 3. 合併成 modules 列表，並產生初始 preorder/inorder
    std::vector<NodeInt*> modules;
    modules.reserve(hierNodes.size() + soloNodes.size());
    modules.insert(modules.end(), hierNodes.begin(), hierNodes.end());
    modules.insert(modules.end(), soloNodes.begin(), soloNodes.end());

    std::vector<NodeInt*> preorder, inorder;
    buildInitialSolution(modules, preorder, inorder);

    bst.buildTree(preorder, inorder);
}


/*==============================================================================
 * pack() — 同步打包 HBTree + island + contour node
 *============================================================================*/
int64_t HBTree::pack(std::vector<ASFIsland*>& islands,
                     std::vector<Block>& blocks,
                     const std::unordered_map<std::string,int>& idx)
{
    // 1) 重新 pack 每個 island 內部 （如果 island 內部也要重新 SA 擾動）
    for (auto *isl : islands) {
        isl->pack(blocks, idx);
    }

    // 1. 用 B*-Tree 計算全局 (x,y)
    bst.setPosition();

    // 2. 把每個 symmetry island 的 local pack 結果平移到全局座標
    //    hierNodes[i] 對應 islands[i]
    for (size_t i = 0; i < hierNodes.size(); ++i) {
        NodeInt* n = hierNodes[i];
        int64_t dx = n->x;
        int64_t dy = n->y;
        // 每個 ASFIsland 內部都已經在 pack() 時設定好 local (0,0) 開始的
        // blocks 座標，我們只要把它們往 (dx,dy) 平移就能到全局位置
        for (const auto& id : islands[i]->blockIds) {
            // 假設 ASFIsland 暴露了一個 map<string,int> 叫 localIdxMap
            // 其 value 就是對應到全域 blocks 的索引
            blocks[id].x += dx;
            blocks[id].y += dy;
        }
    }


    // 3. 放 solo blocks
    //    soloNodes[i] 對應 soloIds[i]
    for (size_t i = 0; i < soloNodes.size(); ++i) {
        NodeInt* n = soloNodes[i];
        int bid = soloNumber[i];
        blocks[bid].x = n->x;
        blocks[bid].y = n->y;
    }

    // 4. 回傳整個排版面積
    return bst.getArea();
}

