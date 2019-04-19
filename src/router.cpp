#include "router.h"
#include "routingdb.h"
#include <cassert>

#define HORIZONTAL 0
#define VIRTICAL   1

extern RoutingDB db;

void Router::CreateLayout() {
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
