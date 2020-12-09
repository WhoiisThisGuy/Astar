#include "astar.h"

int Astar::StartSearch(bool *abortFlag){

    src.row = gridcontroller->src.y;
    src.col = gridcontroller->src.x;

    dest.row = gridcontroller->dst.y;
    dest.col = gridcontroller->dst.x;
    dest.g = 0;
    dest.f = 0;

    //Init f,g,h values

    src.g = 0;
    src.h = Hvalue(src.row,dest.row,src.col,dest.col);
    src.f = src.h;
    nearest[src.row*numberOfColumns+src.col] = -1;

    gValues[src.row*numberOfColumns+src.col] = 0;


    //Init open_list details
    open_list.insert(pair(src.f,src));
    isInTheOpenList[src.row*numberOfColumns+src.col] = true;

    while(!open_list.empty()){

        AstarPoint tempNode;

        tempNode = open_list.begin()->second; //top
        open_list.erase(open_list.begin()); //pop

        closedList[tempNode.row*numberOfColumns+tempNode.col] = true;

        /* Solution Found */
        if(tempNode == dest){

            return 0;
        }

        /* Start Check Neighbours */

        /* TOP NEIGHBOUR BLOCK START */
        {
            AstarPoint topNeighbour(tempNode.row-1,tempNode.col);

            if(isOk(topNeighbour) && !isOnClosedList(topNeighbour)){

                if(!isInTheOpenList[topNeighbour.row*numberOfColumns+topNeighbour.col]){ //if not on the open list then insert it

                    topNeighbour.g = tempNode.g+1.0;
                    topNeighbour.h = Hvalue(topNeighbour.row,dest.row,topNeighbour.col,dest.col);
                    topNeighbour.f = topNeighbour.g+topNeighbour.h;
                    gValues[topNeighbour.row*numberOfColumns+topNeighbour.col] = topNeighbour.g;

                    nearest[topNeighbour.row*numberOfColumns+topNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(topNeighbour.f,topNeighbour));

                    isInTheOpenList[topNeighbour.row*numberOfColumns+topNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(topNeighbour.row,topNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    topNeighbour.g = gValues[topNeighbour.row*numberOfColumns+topNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < topNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(topNeighbour); //remove from open_list
                    //Update new values
                    topNeighbour.g = tempGScore;
                    topNeighbour.h = Hvalue(topNeighbour.row,dest.row,topNeighbour.col,dest.col);
                    topNeighbour.f = topNeighbour.g+topNeighbour.h;

                    nearest[topNeighbour.row*numberOfColumns+topNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(topNeighbour.f,topNeighbour)); //Insert it back
                    gValues[topNeighbour.row*numberOfColumns+topNeighbour.col] = topNeighbour.g; //Update gValue
                }

            }
        }
        /* TOP NEIGHBOUR BLOCK END */


        /* TOP RIGHT NEIGHBOUR BLOCK START */
        {
            AstarPoint topRightNeighbour(tempNode.row-1,tempNode.col+1);

            if(isOk(topRightNeighbour) && !isOnClosedList(topRightNeighbour)){

                if(!isInTheOpenList[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col]){ //if not on the open list then insert it

                    topRightNeighbour.g = tempNode.g+1.0;
                    topRightNeighbour.h = Hvalue(topRightNeighbour.row,dest.row,topRightNeighbour.col,dest.col);
                    topRightNeighbour.f = topRightNeighbour.g+topRightNeighbour.h;
                    gValues[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col] = topRightNeighbour.g;

                    nearest[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(topRightNeighbour.f,topRightNeighbour));

                    isInTheOpenList[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(topRightNeighbour.row,topRightNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    topRightNeighbour.g = gValues[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < topRightNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(topRightNeighbour); //remove from open_list
                    //Update new values
                    topRightNeighbour.g = tempGScore;
                    topRightNeighbour.h = Hvalue(topRightNeighbour.row,dest.row,topRightNeighbour.col,dest.col);
                    topRightNeighbour.f = topRightNeighbour.g+topRightNeighbour.h;

                    nearest[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(topRightNeighbour.f,topRightNeighbour)); //Insert it back
                    gValues[topRightNeighbour.row*numberOfColumns+topRightNeighbour.col] = topRightNeighbour.g; //Update gValue
                }

            }
        }
        /* TOP RIGHT NEIGHBOUR BLOCK END */

        /* RIGHT NEIGHBOUR BLOCK START */
        {
            AstarPoint RightNeighbour(tempNode.row,tempNode.col+1);

            if(isOk(RightNeighbour) && !isOnClosedList(RightNeighbour)){

                if(!isInTheOpenList[RightNeighbour.row*numberOfColumns+RightNeighbour.col]){ //if not on the open list then insert it

                    RightNeighbour.g = tempNode.g+1.0;
                    RightNeighbour.h = Hvalue(RightNeighbour.row,dest.row,RightNeighbour.col,dest.col);
                    RightNeighbour.f = RightNeighbour.g+RightNeighbour.h;
                    gValues[RightNeighbour.row*numberOfColumns+RightNeighbour.col] = RightNeighbour.g;

                    nearest[RightNeighbour.row*numberOfColumns+RightNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(RightNeighbour.f,RightNeighbour));

                    isInTheOpenList[RightNeighbour.row*numberOfColumns+RightNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(RightNeighbour.row,RightNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    RightNeighbour.g = gValues[RightNeighbour.row*numberOfColumns+RightNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < RightNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(RightNeighbour); //remove from open_list
                    //Update new values
                    RightNeighbour.g = tempGScore;
                    RightNeighbour.h = Hvalue(RightNeighbour.row,dest.row,RightNeighbour.col,dest.col);
                    RightNeighbour.f = RightNeighbour.g+RightNeighbour.h;

                    nearest[RightNeighbour.row*numberOfColumns+RightNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(RightNeighbour.f,RightNeighbour)); //Insert it back
                    gValues[RightNeighbour.row*numberOfColumns+RightNeighbour.col] = RightNeighbour.g; //Update gValue
                }

            }
        }
        /* RIGHT NEIGHBOUR BLOCK END */

        /* BOTTOM RIGHT NEIGHBOUR BLOCK START */
        {
            AstarPoint BottomRightNeighbour(tempNode.row+1,tempNode.col+1);

            if(isOk(BottomRightNeighbour) && !isOnClosedList(BottomRightNeighbour)){

                if(!isInTheOpenList[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col]){ //if not on the open list then insert it

                    BottomRightNeighbour.g = tempNode.g+1.0;
                    BottomRightNeighbour.h = Hvalue(BottomRightNeighbour.row,dest.row,BottomRightNeighbour.col,dest.col);
                    BottomRightNeighbour.f = BottomRightNeighbour.g+BottomRightNeighbour.h;
                    gValues[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col] = BottomRightNeighbour.g;

                    nearest[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(BottomRightNeighbour.f,BottomRightNeighbour));

                    isInTheOpenList[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(BottomRightNeighbour.row,BottomRightNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    BottomRightNeighbour.g = gValues[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < BottomRightNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(BottomRightNeighbour); //remove from open_list
                    //Update new values
                    BottomRightNeighbour.g = tempGScore;
                    BottomRightNeighbour.h = Hvalue(BottomRightNeighbour.row,dest.row,BottomRightNeighbour.col,dest.col);
                    BottomRightNeighbour.f = BottomRightNeighbour.g+BottomRightNeighbour.h;

                    nearest[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(BottomRightNeighbour.f,BottomRightNeighbour)); //Insert it back
                    gValues[BottomRightNeighbour.row*numberOfColumns+BottomRightNeighbour.col] = BottomRightNeighbour.g; //Update gValue
                }

            }
        }
        /* BOTTOM RIGHT NEIGHBOUR BLOCK END */

        /* BOTTOM NEIGHBOUR BLOCK START */
        {
            AstarPoint BottomNeighbour(tempNode.row+1,tempNode.col);

            if(isOk(BottomNeighbour) && !isOnClosedList(BottomNeighbour)){

                if(!isInTheOpenList[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col]){ //if not on the open list then insert it

                    BottomNeighbour.g = tempNode.g+1.0;
                    BottomNeighbour.h = Hvalue(BottomNeighbour.row,dest.row,BottomNeighbour.col,dest.col);
                    BottomNeighbour.f = BottomNeighbour.g+BottomNeighbour.h;
                    gValues[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col] = BottomNeighbour.g;

                    nearest[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(BottomNeighbour.f,BottomNeighbour));

                    isInTheOpenList[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(BottomNeighbour.row,BottomNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    BottomNeighbour.g = gValues[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < BottomNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(BottomNeighbour); //remove from open_list
                    //Update new values
                    BottomNeighbour.g = tempGScore;
                    BottomNeighbour.h = Hvalue(BottomNeighbour.row,dest.row,BottomNeighbour.col,dest.col);
                    BottomNeighbour.f = BottomNeighbour.g+BottomNeighbour.h;

                    nearest[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(BottomNeighbour.f,BottomNeighbour)); //Insert it back
                    gValues[BottomNeighbour.row*numberOfColumns+BottomNeighbour.col] = BottomNeighbour.g; //Update gValue
                }

            }
        }
        /* BOTTOM NEIGHBOUR BLOCK END */

        /* BOTTOM LEFT NEIGHBOUR BLOCK START */
        {
            AstarPoint BottomLeftNeighbour(tempNode.row+1,tempNode.col-1);

            if(isOk(BottomLeftNeighbour) && !isOnClosedList(BottomLeftNeighbour)){

                if(!isInTheOpenList[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col]){ //if not on the open list then insert it

                    BottomLeftNeighbour.g = tempNode.g+1.0;
                    BottomLeftNeighbour.h = Hvalue(BottomLeftNeighbour.row,dest.row,BottomLeftNeighbour.col,dest.col);
                    BottomLeftNeighbour.f = BottomLeftNeighbour.g+BottomLeftNeighbour.h;
                    gValues[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col] = BottomLeftNeighbour.g;

                    nearest[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(BottomLeftNeighbour.f,BottomLeftNeighbour));

                    isInTheOpenList[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(BottomLeftNeighbour.row,BottomLeftNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    BottomLeftNeighbour.g = gValues[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < BottomLeftNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(BottomLeftNeighbour); //remove from open_list
                    //Update new values
                    BottomLeftNeighbour.g = tempGScore;
                    BottomLeftNeighbour.h = Hvalue(BottomLeftNeighbour.row,dest.row,BottomLeftNeighbour.col,dest.col);
                    BottomLeftNeighbour.f = BottomLeftNeighbour.g+BottomLeftNeighbour.h;

                   nearest[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(BottomLeftNeighbour.f,BottomLeftNeighbour)); //Insert it back
                    gValues[BottomLeftNeighbour.row*numberOfColumns+BottomLeftNeighbour.col] = BottomLeftNeighbour.g; //Update gValue
                }

            }
        }
        /* BOTTOM LEFT NEIGHBOUR BLOCK END */

        /* LEFT NEIGHBOUR BLOCK START */
        {
            AstarPoint LeftNeighbour(tempNode.row,tempNode.col-1);

            if(isOk(LeftNeighbour) && !isOnClosedList(LeftNeighbour)){

                if(!isInTheOpenList[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col]){ //if not on the open list then insert it

                    LeftNeighbour.g = tempNode.g+1.0;
                    LeftNeighbour.h = Hvalue(LeftNeighbour.row,dest.row,LeftNeighbour.col,dest.col);
                    LeftNeighbour.f = LeftNeighbour.g+LeftNeighbour.h;
                    gValues[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col] = LeftNeighbour.g;

                    nearest[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(LeftNeighbour.f,LeftNeighbour));

                    isInTheOpenList[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(LeftNeighbour.row,LeftNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    LeftNeighbour.g = gValues[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < LeftNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(LeftNeighbour); //remove from open_list
                    //Update new values
                    LeftNeighbour.g = tempGScore;
                    LeftNeighbour.h = Hvalue(LeftNeighbour.row,dest.row,LeftNeighbour.col,dest.col);
                    LeftNeighbour.f = LeftNeighbour.g+LeftNeighbour.h;

                    nearest[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(LeftNeighbour.f,LeftNeighbour)); //Insert it back
                    gValues[LeftNeighbour.row*numberOfColumns+LeftNeighbour.col] = LeftNeighbour.g; //Update gValue
                }

            }
        }
        /* LEFT NEIGHBOUR BLOCK END */

        /* TOP LEFT NEIGHBOUR BLOCK START */
        {
            AstarPoint topLeftNeighbour(tempNode.row-1,tempNode.col-1);

            if(isOk(topLeftNeighbour) && !isOnClosedList(topLeftNeighbour)){

                if(!isInTheOpenList[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col]){ //if not on the open list then insert it

                    topLeftNeighbour.g = tempNode.g+1.0;
                    topLeftNeighbour.h = Hvalue(topLeftNeighbour.row,dest.row,topLeftNeighbour.col,dest.col);
                    topLeftNeighbour.f = topLeftNeighbour.g+topLeftNeighbour.h;
                    gValues[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col] = topLeftNeighbour.g;

                    nearest[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair<int,AstarPoint>(topLeftNeighbour.f,topLeftNeighbour));

                    isInTheOpenList[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col] = true; //now it is on the open list
                    gridcontroller->setGridValue(topLeftNeighbour.row,topLeftNeighbour.col,5);
                    ++(gridcontroller->numberOfVisitedNodes);
                }
                else{
                    topLeftNeighbour.g = gValues[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col];
                }
                double tempGScore = 0;
                tempGScore = tempNode.g + 1.0;
                if(tempGScore < topLeftNeighbour.g){
                    //this is a better path so change the data in the open_list for the neighbour
                    RemoveFromOpenList(topLeftNeighbour); //remove from open_list
                    //Update new values
                    topLeftNeighbour.g = tempGScore;
                    topLeftNeighbour.h = Hvalue(topLeftNeighbour.row,dest.row,topLeftNeighbour.col,dest.col);
                    topLeftNeighbour.f = topLeftNeighbour.g+topLeftNeighbour.h;

                    nearest[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col] = tempNode.row*numberOfColumns+tempNode.col;

                    open_list.insert(pair(topLeftNeighbour.f,topLeftNeighbour)); //Insert it back
                    gValues[topLeftNeighbour.row*numberOfColumns+topLeftNeighbour.col] = topLeftNeighbour.g; //Update gValue
                }

            }
        }
        /* TOP LEFT NEIGHBOUR BLOCK END */

    }

    cout<<"No path found at all."<<endl;
    return 1;

}

bool Astar::Init(const vector<string>& Parameters){
    if(Parameters.size() == 0) return false;

    try{
        heuristic = Parameters.at(0); //Save heuristic name

        numberOfColumns = gridcontroller->numberOfColumns;
        numberOfRows = gridcontroller->numberOfRows;

        closedList = make_unique<bool[]>(numberOfRows*numberOfColumns);
        memset(&closedList[0],0,(numberOfRows*numberOfColumns)*sizeof(bool));

        isInTheOpenList = make_unique<bool[]>(numberOfRows*numberOfColumns);
        memset(&(closedList[0]),0,(numberOfRows*numberOfColumns)*sizeof(bool));

        gValues = make_unique<int[]>(numberOfRows*numberOfColumns);
        int size = numberOfRows*numberOfColumns;
        for(int i = 0;i<size;++i){
            gValues[i] = 0;
        }

        nearest = make_unique<int[]>(numberOfRows*numberOfColumns);
        for(int i = 0;i<size;++i){
            nearest[i] = 0;
        }


    }catch(std::error_code e){
        cout<<e.value()<<endl;
        return false;
    }
    return true;
}

double Astar::EucledianDistance(const int& row1,const int& row2,const int& col1,const int& col2){
    return sqrt(pow((row2-row1),2) + pow((col2 - col1), 2));
}

int Astar::ManhattanDistance(const int& row1,const int& row2,const int& col1,const int& col2)
{
    return (abs(row1-row2))+(abs(col1-col2));
}

double Astar::Hvalue(const int& row1,const int& row2,const int& col1,const int& col2){

    return heuristic == "Manhattan" ? ManhattanDistance(row1,row2,col1,col2) : EucledianDistance(row1,row2,col1,col2);
}

bool Astar::isOk(const AstarPoint& p)
{
    if(p.col<0 || p.col > numberOfColumns-1 || p.row < 0 ||p.row>numberOfRows-1 || (gridcontroller->grid[p.row*numberOfColumns+p.col] == 1))
        return false;
    return true;
}

bool Astar::isOnClosedList(const AstarPoint& p)
{
    return closedList[p.row*numberOfColumns+p.col];
}

void Astar::RemoveFromOpenList(const AstarPoint& p)
{
    for (auto it = open_list.find(p.f); it != open_list.end(); it++) {
        if (it->second.row == p.row && it->second.col == p.col) {
            open_list.erase(it);
            return;
        }
    }
}

AstarPoint Astar::GetPointFromOpenList(const AstarPoint& p)
{
    for (auto it = open_list.begin(); it != open_list.end(); it++) {
        if (it->second.row == p.row && it->second.col == p.col) {

            return it->second;
        }
    }
}

vector<int> Astar::getPath()
{
    vector<int> ThePath;

    int temp = dest.row*numberOfColumns+dest.col;

    while(nearest[temp] != -1){
        ThePath.push_back(temp);
        temp = nearest[temp];
    }

    return ThePath;
}

extern "C" ASTAR_EXPORT IPathfinder* InitPathfinderObject(){

    return new Astar();
}
