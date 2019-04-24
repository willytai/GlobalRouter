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
        cout << "\t> routing Net " << n.GetName() << ' ' << i+1 << '/' << db.GetNetNo() << endl;
        for (int j = 0; j < n.GetSubNetNo(); ++j) {
            this->route_subnet(n.GetSubNet(j));
        }
    }
}

int Cell::_global_net_ref = -1;

void Router::route_subnet(SubNet& subnet) {
    short sx = subnet.GetSourcePinGx();
    short sy = subnet.GetSourcePinGy();
    short sz = subnet.GetSourcePinLayer();
    short tx = subnet.GetTargetPinGx();
    short ty = subnet.GetTargetPinGy();
    short tz = subnet.GetTargetPinLayer();
    Coordinate goal(tx, ty, tz);
    Coordinate start(sx, sy, sz);
    cout << endl;
    cout << "[starting    point] "; start.print(); cout << endl;
    cout << "[destination point] "; goal.print(); cout << endl;
    Cell::SetGlobalNetRef(subnet.GetNetUid());
    this->dijkstra(this->GetCellByCoordinate(start), this->GetCellByCoordinate(goal));
}

void Router::backtrack(Cell* start, Cell* goal) {
    cout << "[Backtracking] ";
    int length = 0;
    Cell* tmp = goal;
    Cell* wire_start = start;
    while (tmp != start) {
        /*
        start->printCoordinates();
        cout << ' ';
        goal->printCoordinates();
        cout << ' ';
        tmp->printCoordinates(); cout << endl;
        */
        int curLayer = tmp->GetZ();

        Cell* prev = tmp;
        tmp = tmp->GetParent();
        assert(tmp && "destination is not connected to source!");
        Edge* e = tmp->get_edge(prev);
        if (e) {
            e->DecreaseCapacity();
        }
        else { // via
            if (!wire_start) wire_start = tmp;
            else {
                Wire newWire(wire_start->GetCoordinate(), prev->GetCoordinate());
                Wire newVia(prev->GetCoordinate(), tmp->GetCoordinate());

                // need to check if these two wire s are legal
                // remove repeated segments, check them by _global_net_ref
                if (this->check_wire_and_correct(newWire)) _wires.push_back(newWire);
                if (this->check_wire_and_correct(newVia))  _wires.push_back(newVia);
            }
        }

        // float cost = e ? e->GetCost() : -1;
        // cout << " edge cost with parent: " << cost << endl;
        int nextLayer = tmp->GetZ();
        if (curLayer == nextLayer) ++length;
    }
    // tmp->printCoordinates(); cout << endl;
    cout << "length: " << length << endl;
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

bool Router::check_wire_and_correct(Wire& wire) {
    // TODO
    return false;
}
