#include "router.h"
#include "routingdb.h"
#include "minHeap.h"
#include <cassert>
#include <cstdlib>
#include <limits>

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
        cout << "\t> routing Net " << n.GetName() << ' ' << i+1 << '/' << db.GetNetNo() << endl << endl;
        Cell::SetGlobalNetRef(n.GetUid());
        _wires.clear();
        for (int j = 0; j < n.GetSubNetNo(); ++j) {
            this->route_subnet(n.GetSubNet(j));
        }
        this->collect_wires();

        // dump to file
        _outfile << n.GetName() << ' ' << n.GetUid() << ' ' <<  _wires.size() << endl;
        for (auto it = _wires.begin(); it != _wires.end(); ++it)
            _outfile << *it << endl;
        _outfile << '!' << endl;
    }
}

int Cell::_global_net_ref = -1;

void Router::route_subnet(SubNet& subnet) {
    short sx = subnet.GetSourcePinGx();
    short sy = subnet.GetSourcePinGy();
    short sz = subnet.GetSourcePinLayer()-1;
    short tx = subnet.GetTargetPinGx();
    short ty = subnet.GetTargetPinGy();
    short tz = subnet.GetTargetPinLayer()-1;
    Coordinate goal(tx, ty, tz);
    Coordinate start(sx, sy, sz);
    cout << "[starting    point] "; start.print(); cout << endl;
    cout << "[destination point] "; goal.print(); cout << endl;
    this->dijkstra(this->GetCellByCoordinate(start), this->GetCellByCoordinate(goal));
}

void Router::collect_wires() {

    // HORIZONTAL layer
    for (int y = 0; y < _height; ++y) {
        Cell* start = NULL;
        Cell* end   = NULL;
        for (int x = 0; x < _width; ++x) {
            if (_layout[HORIZONTAL][x][y]->isGlobalNetRef() && !start) {
                start = _layout[HORIZONTAL][x][y];
                continue;
            }
            if (!(_layout[HORIZONTAL][x][y]->isGlobalNetRef()) && start) {
                end = _layout[HORIZONTAL][x-1][y];
            }
            else if (start && x == _width-1) {
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
    for (int x = 0; x < _width; ++x) {
        Cell* start = NULL;
        Cell* end   = NULL;
        for (int y = 0; y < _height; ++y) {
            if (_layout[VIRTICAL][x][y]->isGlobalNetRef() && !start) {
                start = _layout[VIRTICAL][x][y];
                continue;
            }
            if (!(_layout[VIRTICAL][x][y]->isGlobalNetRef()) && start) {
                end = _layout[VIRTICAL][x][y-1];
            }
            else if (start && y == _height-1) {
                end = _layout[VIRTICAL][x][y];
            }
            else continue;
            if (start != end) {
                _wires.push_back(Wire(start->GetCoordinate(), end->GetCoordinate()));
            }
            start = end = NULL;
        }
    }
}

void Router::backtrack(Cell* start, Cell* goal) {
    cout << "[Backtracking]" << endl << endl;
    // int length = 0;
    Cell* tmp = goal;
    while (tmp != start) {
        /*
        start->printCoordinates();
        cout << ' ';
        goal->printCoordinates();
        cout << ' ';
        tmp->printCoordinates(); cout << endl;
        */
        // int curLayer = tmp->GetZ();

        Cell* prev = tmp;
        tmp = tmp->GetParent();
        assert(tmp && "destination is not connected to source!");
        Edge* e = tmp->get_edge(prev);
        if (e) {
            e->DecreaseCapacity();
        }
        else { // via, colllect them here
            if (tmp->isGlobalNetRef() && prev->isGlobalNetRef()) {}
            else _wires.push_back(Wire(tmp->GetCoordinate(), prev->GetCoordinate()));
        }

        prev->Set2GlobalNetRef();

        // float cost = e ? e->GetCost() : -1;
        // cout << " edge cost with parent: " << cost << endl;
        // int nextLayer = tmp->GetZ();
        // if (curLayer == nextLayer) ++length;
    }
    tmp->Set2GlobalNetRef(); // start
    // tmp->printCoordinates(); cout << endl;
    // cout << "length: " << length << endl;
}

BBox Router::GetBoundingBox(Cell*& c1, Cell*& c2) {
    // return BBox(Coordinate(0, 0, 0), Coordinate(_width-1, _height-1, 0));
    short lx = c1->GetX();
    short ux = c2->GetX();
    if (lx > ux) ::swap(lx, ux);

    short ly = c1->GetY();
    short uy = c2->GetY();
    if (ly > uy) ::swap(ly, uy);

    return BBox(Coordinate(lx, ly, 0), Coordinate(ux, uy, 0));
}
