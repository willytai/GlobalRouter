#ifndef __WIRE_H__
#define __WIRE_H__

#include "Coordinate.h"
#include "routingdb.h"
#include <iostream>

extern RoutingDB db;

struct Wire
{
    Wire() {}
    Wire(Coordinate c1, Coordinate c2) { _start = c1, _end = c2; }
    ~Wire() {}

    Coordinate GetStart() const { return _start; }
    Coordinate GetEnd()   const { return _end; }

    void SetStart(short x, short y, short z) {
        _start.SetX(x);
        _start.SetY(y);
        _start.SetZ(z);
    }

    void SetEnd(short x, short y, short z) {
        _end.SetX(x);
        _end.SetY(y);
        _end.SetZ(z);
    }

    friend ostream& operator << (ostream& os, const Wire& w) {
        int TileWidth = db.GetTileWidth();
        int TileHeight = db.GetTileHeight();
        os << "(" << TileWidth  * w.GetStart().GetX() + TileWidth  / 2;
        os << ',' << TileHeight * w.GetStart().GetY() + TileHeight / 2;
        os << ',' << w.GetStart().GetZ()+1 << ")-";
        os << "(" << TileWidth  * w.GetEnd().GetX() + TileWidth  / 2;
        os << ',' << TileHeight * w.GetEnd().GetY() + TileHeight / 2;
        os << ','<< w.GetEnd().GetZ()+1   << ")";
        return os;
    }

    Coordinate _start;
    Coordinate _end;
};

 

#endif /* __WIRE_H__ */
