#include "router.h"
#include <limits>

#define HORIZONTAL 0
#define VIRTICAL   1

void Router::dijkstra(Cell* start, Cell* goal) {
    BBox box = this->GetBoundingBox(start, goal);
    minHeap<float, Cell*> minQ;
    for (int layer = 0; layer < 2; ++layer) {
        for (int x = box.GetLowerLeftX(); x <= box.GetUpperRightX(); ++x) {
            for (int y = box.GetLowerLeftY(); y <= box.GetUpperRightY(); ++y) {
                _layout[layer][x][y]->ResetParent();
                if (_layout[layer][x][y] != start) minQ.insert(numeric_limits<float>::max(), _layout[layer][x][y]);
                else minQ.insert(0, _layout[layer][x][y]);
            }
        }
    }

    // start relaxing
    while (true) {
        if (minQ.size() <= 0) {
            cerr << "Routing Failed! ";
            start->printCoordinates(); cout << " -> ";
            goal->printCoordinates(); cout << " !" << endl;
            exit(0);
        }
        pair<float, Cell*> curNode = minQ.ExtractMin();
        minQ.pop();
        if (curNode.second == goal) break;
        this->relax(curNode.second, curNode.first, minQ, box);
    }

    // backtrack from goal
    // TODO Decrease the capacities along the used edges!
    this->backtrack(start, goal);
}

void Router::relax(Cell* c, const float& curCost, minHeap<float, Cell*>& heap, const BBox& bbox) {
    if (c->GetZ() == HORIZONTAL) {
        this->relax(c, this->GetAboveCell(c, bbox), curCost, heap);
        this->relax(c, this->GetLeftCell (c, bbox), curCost, heap);
        this->relax(c, this->GetRightCell(c, bbox), curCost, heap);
    }
    if (c->GetZ() == VIRTICAL) {
        this->relax(c, this->GetBelowCell(c, bbox), curCost, heap);
        this->relax(c, this->GetLowerCell(c, bbox), curCost, heap);
        this->relax(c, this->GetUpperCell(c, bbox), curCost, heap);
    }
}

void Router::relax(Cell* src, Cell* c, const float& curCost, minHeap<float, Cell*>& heap) {
    if (!c) return;
    if (!(c->InHeap())) return; // not inside min heap
    
    // find the edge
    Edge* e = src->get_edge(c);

    float EdgeCost;
    if (!e) { // via
        EdgeCost = 0;
    }
    else {
        EdgeCost = e->GetCost();
    }

    // decrease key
    // store a heap_id in Cell to make this O(lgn)
    if (heap.DecreaseKey(c->GetHeapID(), EdgeCost+curCost)) {
        // assert(!(src->InHeap()));
        c->SetParent(src);
        /*
        cout << "setting ";
        c->printCoordinates();
        cout << "'s parent to ";
        src->printCoordinates();
        cout << " heapid: " << c->GetHeapID() << ", " << src->GetHeapID();
        cout << endl;
        */
        // assert(src->GetParent() != c);
    }
    else {
        /*
        cout << "distance from ";
        c->printCoordinates();
        cout << "to ";
        src->printCoordinates();
        cout << " did not decrease.";
        cout << endl;
        */
    }
}


