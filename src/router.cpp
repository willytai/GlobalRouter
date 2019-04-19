#include "router.h"
#include "routingdb.h"
#include "minHeap.h"
#include <cassert>
#include <cstdlib>
#include <limits>

#define HORIZONTAL 0
#define VIRTICAL   1

extern RoutingDB db;

void Router::RUN() {
    this->CreateLayout();
    this->route();
}

void Router::CreateLayout() {
    cout << "\t> Constructing Cells and Edges..." << endl;

    _width          = db.GetHoriGlobalTileNo();
    _height         = db.GetVertiGlobalTileNo();

    short numLayers = db.GetLayerNo();
    int   Hcap      = db.GetLayerHoriCapacity(HORIZONTAL);
    int   Vcap      = db.GetLayerVertiCapacity(VIRTICAL);

    _layout = new Cell***[numLayers];
    for (int i = 0; i < numLayers; ++i) {
        _layout[i] = new Cell**[_width];
        for (int j = 0; j < _width; ++j) {
            _layout[i][j] = new Cell*[_height];
            for (int k = 0; k < _height; ++k) {
                _layout[i][j][k] = new Cell;
                if (this->check_coordinate(j-1, k) && i == HORIZONTAL) this->create_edge(j, k, j-1, k, Hcap, i); 
                if (this->check_coordinate(j, k-1) && i == VIRTICAL)   this->create_edge(j, k, j, k-1, Vcap, i); 
            }
        }
    }

    for (int i = 0; i < db.GetCapacityAdjustNo(); ++i) {
        CapacityAdjust& ca = db.GetCapacityAdjust(i);
        int c1x = ca.GetGx1();
        int c1y = ca.GetGy1();
        int c2x = ca.GetGx2();
        int c2y = ca.GetGy2();
        short layer = ca.GetLayer1()-1;
        Edge* e = _layout[layer][c1x][c1y]->get_edge(_layout[layer][c2x][c2y]);
        assert(e && "Edge not found");
        e->SetCapacity(ca.GetReduceCapacity());
        // cout << "changing edge between " << '(' << c1x << ' ' << c1y << ") (" << c2x << ' ' << c2y << ") on layer " << layer << " to " << e->GetCapacity() << endl;;
    }
}

void Router::create_edge(const int& c1x, const int& c1y, const int& c2x, const int& c2y, const int& cap, const int& layer) {
    // cout << "creating edge between " << '(' << c1x << ' ' << c1y << ") (" << c2x << ' ' << c2y << ')' << " layer: " << layer+1 << endl;
    Cell*& C1 = _layout[layer][c1x][c1y];
    Cell*& C2 = _layout[layer][c2x][c2y];
    Edge* e = new Edge(cap, C1, C2);
    C1->add_edge(e);
    C2->add_edge(e);
}

void Router::route() {
    for (int i = 0; i < db.GetNetNo(); ++i) {
        Net& n = db.GetNetByPosition(i);
        cout << "\t> routing Net " << n.GetName() << endl;
        for (int j = 0; j < n.GetSubNetNo(); ++j) {
            this->route_subnet(n.GetSubNet(j));
        }
    }
}

void Router::route_subnet(SubNet& subnet) {
    short sx = subnet.GetSourcePinGx();
    short sy = subnet.GetSourcePinGy();
    short sz = subnet.GetSourcePinLayer();
    short tx = subnet.GetTargetPinGx();
    short ty = subnet.GetTargetPinGy();
    short tz = subnet.GetTargetPinLayer();
    Coordinate goal(tx, ty, tz);
    Coordinate start(sx, sy, sz);
    this->dijkstra(this->GetCellByCoordinate(start), this->GetCellByCoordinate(goal));
}

void Router::dijkstra(Cell* start, Cell* goal) {
    minHeap<float, Cell*> minQ;
    minQ.insert(0, start);
    for (int layer = 0; layer < 2; ++layer) {
        for (int x = 0; x < _width; ++x) {
            for (int y = 0; y < _height; ++y) {
                if (_layout[layer][x][y] != start) minQ.insert(numeric_limits<float>::max(), _layout[layer][x][y]);
            }
        }
    }
    int numCell = 2*_width*_height;
    for (int i = 0; i < numCell; ++i) {
        Cell* curCell = minQ.ExtractMin();
        minQ.pop();
        this->relax(curCell);
    }
}

void Router::relax(Cell* c) {
    if (c->GetZ() == HORIZONTAL) {
        this->relax(c, this->GetAboveCell(c));
        this->relax(c, this->GetLeftCell(c));
        this->relax(c, this->GetRightCell(c));
    }
    if (c->GetZ() == VIRTICAL) {
        this->relax(c, this->GetBelowCell(c));
        this->relax(c, this->GetUpperCell(c));
        this->relax(c, this->GetLowerCell(c));
    }
}

void Router::relax(Cell* src, Cell* c) {
    if (!c) return;
    
    // find the edge
    Edge* e = src->get_edge(c);
    c->SetParent(src);

    float EdgeCost;
    if (!e) { // via
        EdgeCost = 0;
    }
    else {
        EdgeCost = e->GetCost();
    }

    // decrease key
    // store a heap_id in Cell to make this O(lgn)
    // TODO
}

Cell* Router::GetCellByCoordinate(const Coordinate& coor) {
    return _layout[coor.GetZ()][coor.GetX()][coor.GetY()];
}

Cell* Router::GetUpperCell(const Cell* c) {
    if (!this->check_coordinate(c->GetX(), c->GetY()+1)) return NULL;
    return _layout[c->GetZ()][c->GetX()][c->GetY()+1];
}
Cell* Router::GetLowerCell(const Cell* c) {
    if (!this->check_coordinate(c->GetX(), c->GetY()-1)) return NULL;
    return _layout[c->GetZ()][c->GetX()][c->GetY()-1];
}

Cell* Router::GetRightCell(const Cell* c) {
    if (!this->check_coordinate(c->GetX()+1, c->GetY())) return NULL;
    return _layout[c->GetZ()][c->GetX()+1][c->GetY()];
}

Cell* Router::GetLeftCell (const Cell* c) {
    if (!this->check_coordinate(c->GetX()-1, c->GetY())) return NULL;
    return _layout[c->GetZ()][c->GetX()-1][c->GetY()];
}

Cell* Router::GetAboveCell(const Cell* c) {
    if (!this->check_coordinate(c->GetX(), c->GetY())) return NULL;
    return _layout[c->GetZ()+1][c->GetX()][c->GetY()];
}

Cell* Router::GetBelowCell(const Cell* c) {
    if (!this->check_coordinate(c->GetX(), c->GetY())) return NULL;
    return _layout[c->GetZ()-1][c->GetX()][c->GetY()];
}
