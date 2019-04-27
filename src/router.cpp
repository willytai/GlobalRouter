#include "router.h"
#include "routingdb.h"
#include "minHeap.h"
#include <cassert>
#include <cstdlib>
#include <limits>

extern RoutingDB db;

int Cell::_global_net_ref = -1;

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
                _layout[i][j][k]->SetCoordinate(j, k, i);
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
        // assert(e && "Edge not found");
        e->SetCapacity(ca.GetReduceCapacity());
    }
}

void Router::create_edge(const int& c1x, const int& c1y, const int& c2x, const int& c2y, const int& cap, const int& layer) {
    Cell*& C1 = _layout[layer][c1x][c1y];
    Cell*& C2 = _layout[layer][c2x][c2y];
    Edge* e = new Edge(cap, C1, C2);
    C1->add_edge(e);
    C2->add_edge(e);
}

void Router::route() {
    for (int i = 0; i < db.GetNetNo(); ++i) {
        Net& n = db.GetNetByPosition(i);
        if (i % 100 == 99) cout << "\t> " << i+1 << '/' << db.GetNetNo() << " routed." << endl;
        Cell::SetGlobalNetRef(n.GetUid());
        _wires.clear();
        short sx = n.GetSubNet(0).GetSourcePinGx();
        short sy = n.GetSubNet(0).GetSourcePinGy();
        short sz = n.GetSubNet(0).GetSourcePinLayer()-1;
        Coordinate s(sx, sy, sz);
        BBox P_bbox(s, s);
        for (int j = 0; j < n.GetSubNetNo(); ++j) {
            this->route_subnet(n.GetSubNet(j), P_bbox);
        }
        this->collect_wires(P_bbox);

        // dump to file
        _outfile << n.GetName() << ' ' << n.GetUid() << ' ' <<  _wires.size() << endl;
        for (auto it = _wires.begin(); it != _wires.end(); ++it)
            _outfile << *it << endl;
        _outfile << '!' << endl;
    }
}

void Router::route_subnet(SubNet& subnet, BBox& P_bbox) {
    short sx = subnet.GetSourcePinGx();
    short sy = subnet.GetSourcePinGy();
    short sz = subnet.GetSourcePinLayer()-1;
    short tx = subnet.GetTargetPinGx();
    short ty = subnet.GetTargetPinGy();
    short tz = subnet.GetTargetPinLayer()-1;
    Coordinate goal(tx, ty, tz);
    Coordinate start(sx, sy, sz);
    if (!P_bbox.contain(goal))  P_bbox.grow(goal);
    if (!P_bbox.contain(start)) P_bbox.grow(start);
    this->dijkstra(this->GetCellByCoordinate(start), this->GetCellByCoordinate(goal));
}

void Router::collect_wires(const BBox& P_bbox) {

    // HORIZONTAL layer
    for (int y = P_bbox.GetLowerLeftY(); y <= P_bbox.GetUpperRightY(); ++y) {
        Cell* start = NULL;
        Cell* end   = NULL;
        for (int x = P_bbox.GetLowerLeftX(); x <= P_bbox.GetUpperRightX(); ++x) {
            if (_layout[HORIZONTAL][x][y]->isGlobalNetRef() && !start) {
                start = _layout[HORIZONTAL][x][y];
                continue;
            }
            if (!(_layout[HORIZONTAL][x][y]->isGlobalNetRef()) && start) {
                end = _layout[HORIZONTAL][x-1][y];
            }
            else if (start && x == P_bbox.GetUpperRightX()) {
                end = _layout[HORIZONTAL][x][y];
            }
            else continue;
            if (start != end) {
                _wires.push_back(Wire(start->GetCoordinate(), end->GetCoordinate()));
            }
            start = end = NULL;
        }
    }

    // VIRTICAL layer
    for (int x = P_bbox.GetLowerLeftX(); x <= P_bbox.GetUpperRightX(); ++x) {
        Cell* start = NULL;
        Cell* end   = NULL;
        for (int y = P_bbox.GetLowerLeftY(); y <= P_bbox.GetUpperRightY(); ++y) {
            if (_layout[VIRTICAL][x][y]->isGlobalNetRef() && !start) {
                start = _layout[VIRTICAL][x][y];
                continue;
            }
            if (!(_layout[VIRTICAL][x][y]->isGlobalNetRef()) && start) {
                end = _layout[VIRTICAL][x][y-1];
            }
            else if (start && y == P_bbox.GetUpperRightY()) {
                end = _layout[VIRTICAL][x][y];
            }
            else continue;
            if (start != end) {
                _wires.push_back(Wire(start->GetCoordinate(), end->GetCoordinate()));
            }
            start = end = NULL;
        }
    }

    // VIA
    for (int x = P_bbox.GetLowerLeftX(); x <= P_bbox.GetUpperRightX(); ++x) {
        for (int y = P_bbox.GetLowerLeftY(); y <= P_bbox.GetUpperRightY(); ++y) {
            if (_layout[HORIZONTAL][x][y]->isGlobalNetRef() && _layout[VIRTICAL][x][y]->isGlobalNetRef()) {
                _wires.push_back(Wire(_layout[HORIZONTAL][x][y]->GetCoordinate(), _layout[VIRTICAL][x][y]->GetCoordinate()));
            }
        }
    }
}

void Router::backtrack(Cell* start, Cell* goal) {
    Cell* tmp = goal;
    while (true) {
        tmp->Set2GlobalNetRef();
        Cell* next = tmp->GetParent();
        if (!next) {
            // assert(tmp == start);
            break;
        }

        Edge* e = tmp->get_edge(next);
        if (e) e->DecreaseCapacity();

        tmp = next;
    }
}

BBox Router::GetBoundingBox(Cell*& c1, Cell*& c2) {
    short lx = c1->GetX();
    short ux = c2->GetX();
    if (lx > ux) ::swap(lx, ux);

    short ly = c1->GetY();
    short uy = c2->GetY();
    if (ly > uy) ::swap(ly, uy);

    return BBox(Coordinate(lx, ly, 0), Coordinate(ux, uy, 0));
}
