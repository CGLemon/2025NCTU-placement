# B*-Tree Usage
1. Create two Node vectors as the initial solutions.
2. We will treat the first vector as the preorder traversal of the B*-tree.
3. We will treat the second vector as the inorder traversal of the B*-tree.
3. Pass two Node vectors into the function `buildTree` to build a B*-Tree.
4. Call the function `setPosition` to set the position of all the nodes.
5. After setting the position of all the nodes, call the function `getArea` to get the area of the placement result.

Example:
```cpp
std::vector<Node<int64_t> *> preorder, inorder;
std::tie(preorder, inorder) = yourInitialSolution();
BStarTree<int64_t> bStarTree;
bStarTree.buildTree(preorder, inorder);
bStarTree.setPosition();
int64_t area = bStarTree.getArea();
```
