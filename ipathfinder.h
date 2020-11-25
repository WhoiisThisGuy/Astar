#ifndef IPATHFINDER_H
#define IPATHFINDER_H
#include "iobserver.h"
#include <string>
#include <vector>

using std::string;
using std::vector;

class IPathfinder {

public:
    virtual ~IPathfinder(){}
    virtual int StartSearch() = 0;
    virtual bool Init(vector<string> Parameters) = 0;
    virtual vector<int> getPath() = 0;
    void Attach(IObserver* grindcontroller_) {gridcontroller = grindcontroller_;}
    void Detach();
protected:
    IObserver* gridcontroller;
};


#endif // H_IPATHFINDALGORITHM_H
