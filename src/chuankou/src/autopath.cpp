#include "chuankou/autopath.h"
#include <cstdint>

AutoPath::AutoPath() {}

// 获取r行c列块id
int AutoPath::getBlockID(int r, int c) {
    if (c == 0) return 3 + r * 3;
    if (c == 1) return 2 + r * 3;
    if (c == 2) return 1 + r * 3;
    return 0;
}

// 单列计算时间
PathResult AutoPath::pathDecision(int col, const std::vector<std::vector<BlockType>>& grid) {
    PathResult res;
    res.colIndex = col;
    res.isFeasible = true;
    res.totalTime = 0;

    uint8_t take_count = 0;
    
    // 检查该列是否有假块
    for (uint8_t i = 0; i < 4; i++) {
        if (grid[i][col] == FA) {
            res.isFeasible = false;
            return res; // 有假则不取
        }
    }
    
    bool startOnPath = (grid[0][col] == R2);
    if (!startOnPath) {
        int bestSideId = -1;
        for (int c = 0; c < 3; c++) {
            if (c == col) continue;
            if (grid[0][c] == R2) {
                bestSideId = getBlockID(0, c);
                res.taskBlocks.push_back(bestSideId);
                break;
            }
        }
        if (bestSideId != -1) {
            take_count = 1;
            res.totalTime += 2;
        }
    }

    // 计算所需时间，即能直走不侧取，能丢块不侧取，少r1则走之
    for (uint8_t i = 0; i < 4; i++) {
        int current_id = getBlockID(i, col);
        BlockType current_block = grid[i][col];
        
        if (current_block == R1) {
            res.totalTime += 5;
        }
        else if (current_block == R2) {
            if (take_count < 2) {
                res.totalTime += 1;
                res.taskBlocks.push_back(current_id);
                take_count++;
            }
            else {
                res.totalTime += 3;
                res.wasteBlocks.push_back(current_id);
            }
        }
        // EMPTY类型不加时间

        // 如果一列r2块少于2，考虑侧取
        uint8_t farRealBlock = 0;
        for (uint8_t j = i + 1; j < 4; j++) {
            if (grid[j][col] == R2) {
                farRealBlock++;
            }
        }
        
        if (take_count + farRealBlock < 2) {
            int sideBlock[] = {col - 1, col + 1};
            for (uint8_t k = 0; k < 2; k++) {
                int sc = sideBlock[k];
                if (sc >= 0 && sc < 3) {
                    if (grid[i][sc] == R2) {
                        int side_id = getBlockID(i, sc);
                        
                        // 检查是否已经在taskBlocks中
                        bool already_exists = false;
                        for (int id : res.taskBlocks) {
                            if (id == side_id) {
                                already_exists = true;
                                break;
                            }
                        }
                        
                        if (!already_exists) {
                            res.totalTime += 2;
                            res.taskBlocks.push_back(side_id);
                            take_count++;
                            if (take_count + farRealBlock >= 2) break;
                        }
                    }
                }
            }
        }
    }

    if (take_count < 2) {
        res.isFeasible = false;
    }
    
    return res;
}

// 比较时间，得出最佳路径
PathResult AutoPath::planBestPath(const std::vector<std::vector<BlockType>>& grid) {
    PathResult bestRes;
    int minTime = 9999;

    for (int i = 0; i < 3; i++) {
        PathResult current = pathDecision(i, grid);
        if (current.isFeasible) {
            if (current.totalTime < minTime) {
                minTime = current.totalTime;
                bestRes = current;
            }
        }
    }
    return bestRes;
}

// 合成数据包，输出路径即取丢块
std::vector<int> AutoPath::outPutPath(const PathResult& res) {
    std::vector<int> take_and_waste;
    
    if (!res.isFeasible) {
        return take_and_waste; // 返回空vector
    }
    
    // 添加基础路径数据
    if (res.colIndex == 0) {
        take_and_waste.push_back(3);
        take_and_waste.push_back(3);
        take_and_waste.push_back(6);
        take_and_waste.push_back(9);
        take_and_waste.push_back(12);
    }
    else if (res.colIndex == 1) {
        take_and_waste.push_back(2);
        take_and_waste.push_back(2);
        take_and_waste.push_back(5);
        take_and_waste.push_back(8);
        take_and_waste.push_back(11);
    }
    else if (res.colIndex == 2) {
        take_and_waste.push_back(1);
        take_and_waste.push_back(1);
        take_and_waste.push_back(4);
        take_and_waste.push_back(7);
        take_and_waste.push_back(10);
    }
    
    // 添加任务块和丢弃块
    take_and_waste.insert(take_and_waste.end(), res.taskBlocks.begin(), res.taskBlocks.end());
    take_and_waste.insert(take_and_waste.end(), res.wasteBlocks.begin(), res.wasteBlocks.end());
    
    // 补齐到9个元素
    while (take_and_waste.size() < 9) {
        take_and_waste.push_back(0);
    }
    
    return take_and_waste;
}