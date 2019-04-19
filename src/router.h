#ifndef __ROUTER_H__
#define __ROUTER_H__

#include <vector>
#include <map>
#include <cassert>

using namespace std;

struct Cell;
struct Edge;
class Router;

struct Edge
{
    Edge(int c, Cell* c1, Cell* c2) { _capacity = c; _c1 = c1; _c2 = c2; }
    ~Edge() {}

    void SetCapacity(int c) { _capacity = c; }
    void DecreaseCapacity() { --_capacity; }
    int  GetCapacity() const { return _capacity; }

    /* if the edge between c1 and c2 matches itself */
    bool matched(Cell* c1, Cell* c2) {
        if (c1 == _c1 && c2 == _c2) return true;
        if (c1 == _c2 && c2 == _c1) return true;
        return false;
    }

    int _capacity;
    Cell* _c1;
    Cell* _c2;
};

struct Cell
{
    Cell() { this->init_edge_ptr(); }
    ~Cell() {}

    /* for graph search */
    static void SetGlobalRef() { _global_ref++; }
    void Set2GlobalRef() { _ref = _global_ref; }
    bool isGlobalRef() const { return _global_ref == _ref; }

    static int _global_ref;
    int _ref;

    /* edges */
    void init_edge_ptr() { for (int i = 0; i < 4; ++i) _edges[i] = NULL; }
    void add_edge(Edge* e) {
        for (int i = 0; i < 4; ++i) {
            if (!_edges[i]) {
                _edges[i] = e;
                return;
            }
        }
        assert(0 && "Number of edges in a single cell exceeded!");
    }
    Edge* get_edge(Cell* c2) {
        for (int i = 0; i < 4; ++i) {
            if (!_edges[i]) break;
            if (_edges[i]->matched(this, c2)) return _edges[i];
        }
        return NULL;
    }

    Edge* _edges[4];
};

class Router
{
public:
    Router() {}
    ~Router() {}

    void CreateLayout();

    void create_edge(const int&, const int&, const int&, const int&, const int&, const int&);
    bool check_coordinate(const int& x, const int& y) { return (x >= 0 && x < _width && y >= 0 && y < _height); }

private:

    // _layout[layer][x][y]
    Cell**** _layout; 

    short _width;  // horizontal
    short _height; // virtical
};

#endif /* __ROUTER_H__ */
