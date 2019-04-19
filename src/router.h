#ifndef __ROUTER_H__
#define __ROUTER_H__

#include <vector>
#include <map>

using namespace std;

struct Edge
{
    Edge() {}
    Edge(int c) { _capacity = c; }
    ~Edge() {}

    void SetCapacity(int c) { _capacity = c; }
    void DecreaseCapacity() { --_capacity; }

    int _capacity;
};

struct Cell
{
    Cell() {}
    ~Cell() {}

    /* for graph search */
    static void SetGlobalRef() { _global_ref++; }
    void Set2GlobalRef() { _ref = _global_ref; }
    bool isGlobalRef() const { return _global_ref == _ref; }

    static int _global_ref;
    int _ref;
};

class Router
{
public:
    Router() {}
    ~Router() {}

    void CreateLayout();

private:

    // _layout[layer][x][y]
    Cell**** _layout; 

    // _edges[layer][id1][id2], returns a edge pointer, id1 > id2
    Edge**** _edges;

    short _width;  // horizontal
    short _height; // virtical
};

#endif /* __ROUTER_H__ */
