#ifndef __ROUTER_H__
#define __ROUTER_H__

#include "routingdb.h"
#include <vector>
#include <map>
#include <cassert>
#include <queue>

using namespace std;

struct Cell;
struct Edge;
struct Coordinate;
struct cmp;
class Router;

typedef pair<Coordinate, int> Info;
typedef priority_queue<Info, vector<Info>, cmp> frontier;

struct Coordinate
{
    Coordinate() {}
    Coordinate(short x, short y, short z) { _x = x; _y = y; _z = z; }
    ~Coordinate() {}

    void  SetCoordinate(short x, short y, short z) { _x = x; _y = y; _z = z; }
    short GetX() const { return _x; }
    short GetY() const { return _y; }
    short GetZ() const { return _z; }

    bool operator == (const Coordinate& a) { return (a._x == _x && a._y == _y && a._z == _z); }

    short _x;
    short _y;
    short _z;
};

struct Edge
{
    Edge(int c, Cell* c1, Cell* c2) { _capacity = c; _c1 = c1; _c2 = c2; }
    ~Edge() {}

    void  SetCapacity(int c) { _capacity = c; _cost = 1.0/_capacity; }
    void  DecreaseCapacity() { --_capacity; _cost = 1.0/_capacity; }
    int   GetCapacity() const { return _capacity; }
    float GetCost() const { return _cost; }

    /* if the edge between c1 and c2 matches itself */
    bool matched(const Cell* c1, const Cell* c2) const {
        if (c1 == _c1 && c2 == _c2) return true;
        if (c1 == _c2 && c2 == _c1) return true;
        return false;
    }

    int   _capacity;
    float _cost;
    Cell* _c1;
    Cell* _c2;
};

struct Cell
{
    Cell() { this->init_edge_ptr(); _parent = NULL; }
    ~Cell() {}

    /* for graph search */
    /*
    static void SetGlobalRef() { _global_ref++; }
    void Set2GlobalRef() { _ref = _global_ref; }
    bool isGlobalRef() const { return _global_ref == _ref; }
    */

    void SetParent(Cell* c) { _parent = c; }
    void ResetParent() { _parent = NULL; }

    short GetX() const { return _coor.GetX(); }
    short GetY() const { return _coor.GetY(); }
    short GetZ() const { return _coor.GetZ(); }

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
    Edge* get_edge(const Cell* c2) const {
        for (int i = 0; i < 4; ++i) {
            if (!_edges[i]) break;
            if (_edges[i]->matched(this, c2)) return _edges[i];
        }
        return NULL;
    }

    // static int _global_ref;
    Edge*      _edges[4];
    Cell*      _parent;
    // int        _ref;
    Coordinate _coor;
};

struct cmp
{
    bool operator() (const Info& lhs, const Info& rhs) { return lhs.second > rhs.second; }
};


class Router
{
public:
    Router() {}
    ~Router() {}

    void RUN();
    void CreateLayout();
    void route();

    void create_edge(const int&, const int&, const int&, const int&, const int&, const int&);
    void route_subnet(SubNet&);
    void dijkstra(Cell*, Cell*);
    void relax(Cell*);
    void relax(Cell*, Cell*);
    bool check_coordinate(const int& x, const int& y) { return (x >= 0 && x < _width && y >= 0 && y < _height); }
    Cell* GetCellByCoordinate(const Coordinate&);
    Cell* GetUpperCell(const Cell*);
    Cell* GetLowerCell(const Cell*);
    Cell* GetRightCell(const Cell*);
    Cell* GetLeftCell (const Cell*);
    Cell* GetAboveCell(const Cell*);
    Cell* GetBelowCell(const Cell*);

private:

    // _layout[layer][x][y]
    Cell**** _layout; 

    short _width;  // horizontal
    short _height; // virtical
};

#endif /* __ROUTER_H__ */
