#ifndef __ROUTER_H__
#define __ROUTER_H__

#include "minHeap.h"
#include "Cell.h"
#include "Wire.h"
#include <vector>
#include <map>
#include <cassert>
#include <queue>

#define HORIZONTAL 0
#define VIRTICAL   1

extern RoutingDB db;

using namespace std;

struct BBox
{
    BBox() {}
    BBox(Coordinate ll, Coordinate ur) { _ll = ll; _ur = ur; }
    ~BBox() {}

    Coordinate GetLowerLeft()  const { return _ll; }
    Coordinate GetUpperRight() const { return _ur; }

    short GetLowerLeftX() const { return _ll.GetX(); }
    short GetLowerLeftY() const { return _ll.GetY(); }

    short GetUpperRightX() const { return _ur.GetX(); }
    short GetUpperRightY() const { return _ur.GetY(); }

    int GetSize() const { return (_ur.GetX() - _ll.GetX()) * (_ur.GetY() - _ll.GetY()); }

    bool contain(const Coordinate& c) const {
        return (_ll.GetX() <= c.GetX() && c.GetX() <= _ur.GetX() &&
                _ll.GetY() <= c.GetY() && c.GetY() <= _ur.GetY()); 
    }

    void grow(const Coordinate& c) {
        if (_ll == _ur && !(_ur == c)) { _ll = _ur = c; return; }
        short lx = c.GetX() < _ll.GetX() ? c.GetX() : _ll.GetX();
        short ux = c.GetX() > _ur.GetX() ? c.GetX() : _ur.GetX();
        short ly = c.GetY() < _ll.GetY() ? c.GetY() : _ll.GetY();
        short uy = c.GetY() > _ur.GetY() ? c.GetY() : _ur.GetY();
        _ll.SetX(lx);
        _ll.SetY(ly);
        _ur.SetX(ux);
        _ur.SetY(uy);
    }

    Coordinate _ll;
    Coordinate _ur;
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
    void route_subnet(SubNet&, BBox&);
    void dijkstra(Cell*, Cell*);
    void relax(Cell*, const CostType&, minHeap<CostType, Cell*>&, const BBox&);
    void relax(Cell*, Cell*, const CostType&, minHeap<CostType, Cell*>&);
    void backtrack(Cell*, Cell*);
    void collect_wires(const BBox&);
    inline bool check_coordinate(const int& x, const int& y) { return (x >= 0 && x < _width && y >= 0 && y < _height); }
    inline bool check_coordinate(const int& x, const int& y, const BBox& box) {
        return (x >= box.GetLowerLeftX() && x <= box.GetUpperRightX() && 
                y >= box.GetLowerLeftY() && y <= box.GetUpperRightY());
    }

    BBox GetBoundingBox(Cell*&, Cell*&);

    inline Cell* GetCellByCoordinate(const Coordinate& coor) {
        return _layout[coor.GetZ()][coor.GetX()][coor.GetY()];
    }

    inline Cell* GetUpperCell(const Cell* c, const BBox& box) {
        if (!this->check_coordinate(c->GetX(), c->GetY()+1, box)) return NULL;
        return _layout[c->GetZ()][c->GetX()][c->GetY()+1];
    }

    inline Cell* GetLowerCell(const Cell* c, const BBox& box) {
        if (!this->check_coordinate(c->GetX(), c->GetY()-1, box)) return NULL;
        return _layout[c->GetZ()][c->GetX()][c->GetY()-1];
    }

    inline Cell* GetRightCell(const Cell* c, const BBox& box) {
        if (!this->check_coordinate(c->GetX()+1, c->GetY(), box)) return NULL;
        return _layout[c->GetZ()][c->GetX()+1][c->GetY()];
    }

    inline Cell* GetLeftCell (const Cell* c, const BBox& box) {
        if (!this->check_coordinate(c->GetX()-1, c->GetY(), box)) return NULL;
        return _layout[c->GetZ()][c->GetX()-1][c->GetY()];
    }

    inline Cell* GetAboveCell(const Cell* c, const BBox& box) {
        if (!this->check_coordinate(c->GetX(), c->GetY(), box)) return NULL;
        return _layout[c->GetZ()+1][c->GetX()][c->GetY()];
    }

    inline Cell* GetBelowCell(const Cell* c, const BBox& box) {
        if (!this->check_coordinate(c->GetX(), c->GetY(), box)) return NULL;
        return _layout[c->GetZ()-1][c->GetX()][c->GetY()];
    }

private:

    // _layout[layer][x][y]
    Cell**** _layout; 

    short    _width;  // horizontal
    short    _height; // virtical

    ofstream _outfile;

    vector<Wire> _wires;
};

#endif /* __ROUTER_H__ */
