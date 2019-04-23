#ifndef __ROUTER_H__
#define __ROUTER_H__

#include "routingdb.h"
#include "minHeap.h"
#include <vector>
#include <map>
#include <cassert>
#include <queue>

using namespace std;

struct Cell;
struct Edge;
struct Coordinate;
struct Wire;
struct BBox;
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

    void print() const { cout << "(x: " << _x << ", y: " << _y << ", layer: " << _z << ")"; }

    bool operator == (const Coordinate& a) { return (a._x == _x && a._y == _y && a._z == _z); }

    short _x;
    short _y;
    short _z;
};

struct Wire
{
    Wire() {}
    Wire(Coordinate c1, Coordinate c2) { _start = c1, _end = c2; }
    ~Wire() {}

    Coordinate _start;
    Coordinate _end;
};

struct BBox
{
    BBox(Coordinate ll, Coordinate ur) { _ll = ll; _ur = ur; }
    ~BBox() {}

    Coordinate GetLowerLeft()  const { return _ll; }
    Coordinate GetUpperRight() const { return _ur; }

    short GetLowerLeftX() const { return _ll.GetX(); }
    short GetLowerLeftY() const { return _ll.GetY(); }

    short GetUpperRightX() const { return _ur.GetX(); }
    short GetUpperRightY() const { return _ur.GetY(); }

    int GetSize() const { return (_ur.GetX() - _ll.GetX()) * (_ur.GetY() - _ll.GetY()); }

    Coordinate _ll;
    Coordinate _ur;
};

struct Edge
{
    Edge(int c, Cell* c1, Cell* c2) { this->SetCapacity(c); _c1 = c1; _c2 = c2; }
    ~Edge() {}

    void  SetCapacity(int c) { _capacity = c; _cost = 1.0/_capacity; }
    void  DecreaseCapacity() { if (_capacity > 1) --_capacity; else _capacity /= 2; _cost = 1.0/_capacity; }
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
    Cell() { this->init_edge_ptr(); _parent = NULL; _heap_id = -1; _cur_net_ref = -1; }
    ~Cell() {}

    /* backtracking route */
    /* to avoid marking route repeatively */
    static void SetGlobalNetRef(int netid) { _global_net_ref = netid; }
    void Set2GlobalNetRef() { _cur_net_ref = _global_net_ref; }
    bool isGlobalNetRef() const { return _global_net_ref == _cur_net_ref; }

    void SetHeapID(int id) { _heap_id = id; }
    void ResetHeapID() { _heap_id = -1; }
    int  GetHeapID()   { return _heap_id; }
    bool InHeap()      { return (_heap_id != -1); }

    void  SetParent(Cell* c) { _parent = c; }
    void  ResetParent() { _parent = NULL; }
    Cell* GetParent() const { return _parent; }

    void  SetCoordinate(short x, short y, short z) { _coor.SetCoordinate(x, y, z); }
    Coordinate GetCoordinate() const { return _coor; }
    short GetX() const { return _coor.GetX(); }
    short GetY() const { return _coor.GetY(); }
    short GetZ() const { return _coor.GetZ(); }

    void printCoordinates() const { _coor.print(); }

    void init_edge_ptr() { for (int i = 0; i < 3; ++i) _edges[i] = NULL; }
    void add_edge(Edge* e) {
        for (int i = 0; i < 3; ++i) {
            if (!_edges[i]) {
                _edges[i] = e;
                return;
            }
        }
        assert(0 && "Number of edges in a single cell exceeded!");
    }
    Edge* get_edge(const Cell* c2) const {
        for (int i = 0; i < 3; ++i) {
            if (!_edges[i]) break;
            if (_edges[i]->matched(this, c2)) return _edges[i];
        }
        return NULL;
    }

    static int _global_net_ref;
    Edge*      _edges[3];
    Cell*      _parent;
    int        _heap_id;
    int        _cur_net_ref;
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

    void SetOutputFilename(string out) { _outfile = ofstream(out); }
    void RUN();
    void CreateLayout();
    void route();

    void create_edge(const int&, const int&, const int&, const int&, const int&, const int&);
    void route_subnet(SubNet&);
    void dijkstra(Cell*, Cell*);
    void relax(Cell*, const float&, minHeap<float, Cell*>&);
    void relax(Cell*, Cell*, const float&, minHeap<float, Cell*>&);
    void backtrack(Cell*, Cell*);
    bool check_wire_and_correct(Wire&);
    inline bool check_coordinate(const int& x, const int& y) { return (x >= 0 && x < _width && y >= 0 && y < _height); }

    inline BBox GetBoundingBox(Cell*&, Cell*&);

    inline Cell* GetCellByCoordinate(const Coordinate&);
    inline Cell* GetUpperCell(const Cell*);
    inline Cell* GetLowerCell(const Cell*);
    inline Cell* GetRightCell(const Cell*);
    inline Cell* GetLeftCell (const Cell*);
    inline Cell* GetAboveCell(const Cell*);
    inline Cell* GetBelowCell(const Cell*);

private:

    // _layout[layer][x][y]
    Cell**** _layout; 

    short    _width;  // horizontal
    short    _height; // virtical

    ofstream _outfile;

    vector<Wire> _wires;
};

#endif /* __ROUTER_H__ */
