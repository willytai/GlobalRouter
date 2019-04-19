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

    int numCellPerLayer = _width*_height;
    _edges = new Edge***[numLayers];
    for (int i = 0; i < numLayers; ++i) {
        _edges[i] = new Edge**[numCellPerLayer];
        for (int j = 0; j < numCellPerLayer; ++j) {
            if (!j) _edges[i][j] = NULL;
            else _edges[i][j] = new Edge*[j];
            for (int k = 0; k < j; ++k) {
                _edges[i][j][k] = NULL;
            }
        }
    }

    _layout = new Cell***[numLayers];
    for (int i = 0; i < numLayers; ++i) {
        _layout[i] = new Cell**[_width];
        for (int j = 0; j < _width; ++j) {
            _layout[i][j] = new Cell*[_height];
            for (int k = 0; k < _height; ++k) {
                _layout[i][j][k] = new Cell;
                int CellID = j*_width + k;
                int C1     = CellID + 1;
                int C2     = CellID + _width;
                if (k < _width-1) { // x coordinate not exceeded in C1
                    // cout << "layer " << i+1 << ", creating edge for " << C1 << ' ' << CellID << endl;
                    if (i == HORIZONTAL) _edges[i][C1][CellID] = new Edge(Hcap);
                    else                 _edges[i][C1][CellID] = new Edge(Vcap);
                }
                if (j < _height-1) { // y coordinate not exceeded in C2
                    // cout << "layer " << i+1 << ", creating edge for " << C2 << ' ' << CellID << endl;
                    if (i == VIRTICAL)   _edges[i][C2][CellID] = new Edge(Hcap);
                    else                 _edges[i][C2][CellID] = new Edge(Vcap);
                }
            }
        }
    }

    for (int i = 0; i < db.GetCapacityAdjustNo(); ++i) {
        CapacityAdjust& ca = db.GetCapacityAdjust(i);
        int C1 = ca.GetGx1()*_width + ca.GetGy1();
        int C2 = ca.GetGx2()*_width + ca.GetGy2();
        if (C1 < C2) ::swap(C1, C2);
        _edges[ca.GetLayer1()-1][C1][C2]->SetCapacity(ca.GetReduceCapacity());
    }

    for (int C1 = 1; C1 < numCellPerLayer; ++C1) {
        for (int C2 = 0; C2 < C1; ++C2) {
            if (!_edges[0][C1][C2]) continue;
            int x1 = C1 / _width;
            int y1 = C1 % _width;
            int x2 = C2 / _width;
            int y2 = C2 % _width;
            cout << '(' << x1 << ',' << y1 << ") -> " << _edges[1][C1][C2]->_capacity << " -> (" << x2 << ',' << y2 << ')' << endl;
        }
    }
}
