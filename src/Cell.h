#ifndef __CELL_H__
#define __CELL_H__

#include "Coordinate.h"
#include <cassert>

typedef int CostType;

struct Cell;

struct Edge
{
    Edge(int c, Cell* c1, Cell* c2) { this->SetCapacity(c); _c1 = c1; _c2 = c2; }
    ~Edge() {}

    void  SetCapacity(int c) { _capacity = c; }
    void  DecreaseCapacity() { --_capacity; }
    int   GetCapacity() const { return _capacity; }

    /* if the edge between c1 and c2 matches itself */
    bool matched(const Cell* c1, const Cell* c2) const {
        if (c1 == _c1 && c2 == _c2) return true;
        if (c1 == _c2 && c2 == _c1) return true;
        return false;
    }

    CostType _cost;
    int   _capacity;
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
        // assert(0 && "Number of edges in a single cell exceeded!");
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

#endif /* __CELL_H__ */
