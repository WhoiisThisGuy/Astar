#ifndef IObserver_H
#define IObserver_H
#ifndef MYPOINT_H
#define MYPOINT_H

struct Point{

public:
    Point(): x(0),y(0){}
    Point(int x_,int y_):x(x_),y(y_){}
    int x;
    int y;
    bool operator==(const Point& rhs){
        if(this->x == rhs.x && this->y == rhs.y)
            return true;
        return false;
    }
};

#endif // MYPOINT_H


class IObserver
{
public:
    IObserver() {}
    virtual ~IObserver(){}
    virtual void setGridValue(int row, int col, int val) = 0;
    virtual void clearPathColors() = 0;
public:
    int * grid = nullptr;
    Point src;
    Point dst;
    int numberOfRows = 0;
    int numberOfColumns = 0;
    unsigned int numberOfVisitedNodes = 0;
};

#endif // IOBSERVER_H
