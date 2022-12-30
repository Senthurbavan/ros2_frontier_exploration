#include "explore_frontier/astar.hpp"

namespace astar
{

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
  
AStarPlanner::AStarPlanner()
{
}

AStarPlanner::AStarPlanner(nav2_costmap_2d::Costmap2D* costmap):
  costmap_(costmap)
{
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::isValid(unsigned int row, unsigned int col)
{
  unsigned int col_s = costmap_->getSizeInCellsX();
  unsigned int row_s = costmap_->getSizeInCellsY();

  return /*(row>=0) && (col>=0) &&*/ (row<row_s) && (col<col_s);
}


bool AStarPlanner::isUnBlocked(unsigned int row, unsigned int col)
{
  unsigned char cost = costmap_->getCost(col,row);
  if(cost < INSCRIBED_INFLATED_OBSTACLE)
  {
      return true;
  }
  return false;
}

bool AStarPlanner::isDestination(unsigned int row, unsigned int col,
                                explore_frontier::Frontier& f)
{
  double wx, wy;
  costmap_->mapToWorld(col, row, wx, wy);
  double res = costmap_->getResolution();
  for(geometry_msgs::msg::Point point : f.points)
  {
    if (hypot(point.x - wx, point.y - wy) <= res/2)
    {
      return true;
    }
  }   
  return false;
}

double AStarPlanner::calculateHValue(unsigned int row, unsigned int col, Pair dest)
{
  return hypot((double)(row - dest.first), (double)(col - dest.second));
}

void AStarPlanner::tracePath(std::vector<std::vector<cell>> &cellDetails, Pair dest, 
                    std::vector<geometry_msgs::msg::Point> &outPath)
{
  unsigned int row = dest.first;
  unsigned int col = dest.second;

  std::stack<Pair> PathStack;
  outPath.clear();

  while (!(cellDetails[row][col].parent_i == (int)row && 
          cellDetails[row][col].parent_j == (int)col))
  {
    PathStack.push(std::make_pair(row, col));
    unsigned int temp_row = cellDetails[row][col].parent_i;
    unsigned int temp_col = cellDetails[row][col].parent_j;
    row = temp_row;
    col = temp_col;
  }

  PathStack.push(std::make_pair(row, col));

  while (!PathStack.empty())
  {
    std::pair<unsigned int, unsigned int> p = PathStack.top();
    PathStack.pop();
    geometry_msgs::msg::Point point;
    costmap_->mapToWorld(p.second, p.first, point.x, point.y);
    point.z = 0.0;
    outPath.push_back(point);
  }
}

bool AStarPlanner::searchPath(Pair src, explore_frontier::Frontier& f)
{
  if (!isValid(src.first, src.second))
  {
    return false;
  }

  if (!isUnBlocked(src.first, src.second))
  {
    return false;
  }

  if (isDestination(src.first, src.second, f))
  {
    return false;
  }

  Pair target;
  unsigned int mx, my;
  costmap_->worldToMap(f.centroid.x, f.centroid.y, mx, my);
  target.first = my;
  target.second = mx;

  Pair des;
  
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();

  std::vector<std::vector<bool>> closedList(size_y, std::vector<bool>(size_x, false));
  std::vector<std::vector<cell>> cellDetails(size_y, std::vector<cell>(size_x));
  unsigned int i, j;

  for (i = 0; i < size_y; i++)
  {
    for (j = 0; j < size_x; j++)
    {
      cellDetails[i][j].f = FLT_MAX;
      cellDetails[i][j].g = FLT_MAX;
      cellDetails[i][j].h = FLT_MAX;
      cellDetails[i][j].parent_i = -1;
      cellDetails[i][j].parent_j = -1;
    }
  }

  i = src.first, j = src.second;
  cellDetails[i][j].f = 0.0;
  cellDetails[i][j].g = 0.0;
  cellDetails[i][j].h = 0.0;
  cellDetails[i][j].parent_i = i;
  cellDetails[i][j].parent_j = j;

  std::set<pPair> openList;
  openList.insert(std::make_pair(0.0, std::make_pair(i, j)));

  while (!openList.empty())
  {
    pPair p = *openList.begin();
    openList.erase(openList.begin());
    i = p.second.first;
    j = p.second.second;
    closedList[i][j] = true;

    double gNew, hNew, fNew;

    unsigned int array[8][2] = {{i - 1, j}, {i + 1, j}, {i, j + 1}, 
                                {i, j - 1}, {i - 1, j + 1}, {i - 1, j - 1}, 
                                {i + 1, j + 1}, {i + 1, j - 1}};

    for (unsigned int a = 0; a < 8; a++)
    {
      if (isValid(array[a][0], array[a][1]))
      {
        if (isDestination(array[a][0], array[a][1], f))
        {
          des.first = array[a][0];
          des.second = array[a][1];
          cellDetails[array[a][0]][array[a][1]].parent_i = i;
          cellDetails[array[a][0]][array[a][1]].parent_j = j;
          tracePath(cellDetails, des, f.path);
          costmap_->mapToWorld(des.second, des.first, f.middle.x, f.middle.y);
          f.middle.z = 0.0;
          f.min_distance = f.path.size();
          return true;

        }
        else if (!closedList[array[a][0]][array[a][1]] && 
                isUnBlocked(array[a][0], array[a][1]))
        {
          gNew = cellDetails[i][j].g + 1;
          hNew = calculateHValue(array[a][0], array[a][1], target);
          fNew = gNew + hNew;

          if (cellDetails[array[a][0]][array[a][1]].f == FLT_MAX || 
              cellDetails[array[a][0]][array[a][1]].f > fNew)
          {
            openList.insert(std::make_pair(fNew, 
                            std::make_pair(array[a][0], array[a][1])));
            cellDetails[array[a][0]][array[a][1]].f = fNew;
            cellDetails[array[a][0]][array[a][1]].g = gNew;
            cellDetails[array[a][0]][array[a][1]].h = hNew;
            cellDetails[array[a][0]][array[a][1]].parent_i = i;
            cellDetails[array[a][0]][array[a][1]].parent_j = j;
          }   
        }
      }
    }
  }
  return false;
}


}
