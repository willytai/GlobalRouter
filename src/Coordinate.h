#ifndef __COORDINATE_H__
#define __COORDINATE_H__

#include <iostream>

struct Coordinate
{
    Coordinate() {}
    Coordinate(short x, short y, short z) { _x = x; _y = y; _z = z; }
    ~Coordinate() {}

    void  SetCoordinate(short x, short y, short z) { _x = x; _y = y; _z = z; }
    short GetX() const { return _x; }
    short GetY() const { return _y; }
    short GetZ() const { return _z; }

    void SetX(short x) { _x = x; }
    void SetY(short y) { _y = y; }
    void SetZ(short z) { _z = z; }

    void print() const { std::cout << "(x: " << _x << ", y: " << _y << ", layer: " << _z << ")"; }

    bool operator == (const Coordinate& a) { return (a._x == _x && a._y == _y && a._z == _z); }

    short _x;
    short _y;
    short _z;
};



#endif /* __COORDINATE_H__ */
