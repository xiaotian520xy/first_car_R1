#ifndef AUTOPATH_H
#define AUTOPATH_H

#include <vector>
#include <algorithm>

//示意图
/*
 *3      12 11 10
 *2      9  8  7
 *1      6  5  4
 *0      3  2  1
 *
 *       0  1  2    c
 */

enum BlockType {
    EMPTY = 0,
    R1 = 1, // R1块
    R2 = 2, // R2块
    FA = 9    // 假块
};

struct PathResult {
    bool isFeasible;
    int totalTime;
    int colIndex; // 3=左, 2=中, 1=右
    std::vector<int> taskBlocks;  // 存放作为任务取走的ID
    std::vector<int> wasteBlocks; // 存放作为丢弃取走的ID
    
    PathResult() : isFeasible(false), totalTime(0), colIndex(-1) {}
};

class AutoPath
{
public:
    AutoPath();
    PathResult planBestPath(const std::vector<std::vector<BlockType>>& grid);
    std::vector<int> outPutPath(const PathResult& res);
    
private:
    int getBlockID(int r, int c);
    PathResult pathDecision(int col, const std::vector<std::vector<BlockType>>& grid);
};

#endif // AUTOPATH_H