#ifndef ASTAR_H
#define ASTAR_H

#include "Astar_global.h"
#include "iobserver.h"
#include "ipathfinder.h"
#include <iostream>
#include <map>
#include <memory>
#include <algorithm>


using namespace std;

struct AstarPoint{

public:
    AstarPoint(): row(0),col(0),f(0),g(0),h(0),nearestrow(0),nearestcol(0){}
    AstarPoint(int x_,int y_):row(x_),col(y_){}
    AstarPoint(int x_,int y_,double f_,double g_, double h_, int nearestrow_, int nearestcol_)
        :row(x_),col(y_),f(f_),g(g_),h(h_),nearestrow(nearestrow_),nearestcol(nearestcol_){}
    int row;
    int col;
    double f,g,h;
    int nearestrow,nearestcol;
    bool operator==(const AstarPoint& rhs){
        if(this->row == rhs.row && this->col == rhs.col)
            return true;
        return false;
    }

bool operator>(const AstarPoint& p2) const
{

    return this->f > p2.f;
}
};

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;


class Astar : public IPathfinder
{
public:
    virtual ~Astar(){cout<<"Astar destroyed."<<endl;}
    Astar(){}
    int StartSearch(bool *abortFlag = nullptr) override;
    bool Init(const vector<string>& Parameters) override;
    bool isValid(int row, int col);
    bool isUnBlocked(int row, int col);
    bool isDestination(int row, int col, Pair dest);
    double calculateHValue(int row, int col, Pair dest);
    bool aStarSearch(Pair src, Pair dest);
    int ManhattanDistance(const int& row1,const int& row2,const int& col1,const int& col2);
    double EucledianDistance(const int& row1,const int& row2,const int& col1,const int& col2);
    double Hvalue(const int& row1,const int& row2,const int& col1,const int& col2);
    bool isOk(const AstarPoint& p);
    bool isOnClosedList(const AstarPoint& p);
    void RemoveFromOpenList(const AstarPoint& p);
    AstarPoint GetPointFromOpenList(const AstarPoint& p);
   vector<int> getPath() override;

private:
    multimap<int, AstarPoint,std::less<int>> open_list;

    unique_ptr<bool[]> closedList;
    unique_ptr<bool[]> isInTheOpenList;
    unique_ptr<int[]> nearest;

    unique_ptr<int[]> gValues;

    int numberOfColumns;
    int numberOfRows;
    string heuristic;

    AstarPoint src,dest;

};

#endif // ASTAR_H
