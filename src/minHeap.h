#ifndef __MINHEAP_H__
#define __MINHEAP_H__

#include <vector>
#include <iostream>

using namespace std;

template <class Key, class Data>
class minHeap
{
    typedef pair<Key, Data> Node;
public:
    minHeap() { _data.clear(); _data.resize(1); } // _data[0] is reserved (not used)
    ~minHeap() {}

    size_t size() const { return _data.size()-1; }

    Node ExtractMin() const { return _data[1]; }

    bool DecreaseKey(int id, Key key) {
        if (key > _data[id].first) return false;
        _data[id].first = key;
        this->fix_up(id);
        return true;
    }

    void pop() {
        if (!this->size()) return;
        _data[1].second->ResetHeapID();
        _data[1] = _data.back();
        _data.pop_back();
        if (this->size()) this->fix_down(1);
    }

    void insert(Key key, Data d) {
        _data.push_back(Node(key, d));
        d->SetHeapID(this->size());
        this->fix_up(this->size());
    }

    void fix_down(int id) {
        if (id > this->size()/2) return;
        int left_id  = 2*id;
        int right_id = 2*id + 1;
        int target_id;
        if (right_id <= this->size() && left_id <= this->size())
            target_id = (_data[left_id].first < _data[right_id].first ? left_id : right_id);
        else if (right_id > this->size() && left_id <= this->size())
            target_id = left_id;
        else if (right_id <= this->size() && left_id > this->size())
            target_id = right_id;
        else exit(0);
        if (_data[id].first > _data[target_id].first) {
            _data[id].second->SetHeapID(target_id);
            _data[target_id].second->SetHeapID(id);
            ::swap(_data[id], _data[target_id]);
            return fix_down(target_id);
        }
    }

    void fix_up(int id) {
        if (id < 1) return;
        int parent_id = id / 2;
        if (_data[parent_id].first > _data[id].first) {
            _data[parent_id].second->SetHeapID(id);
            _data[id].second->SetHeapID(parent_id);
            ::swap(_data[parent_id], _data[id]);
            return fix_up(parent_id);
        }
    }

    void print() const {
        for (int i = 1; i < _data.size(); ++i) {
            cout << "(key: " << _data[i].first << ", data: ";
            _data[i].second->printCoordinates();
            cout << ", HeapID: " << _data[i].second->GetHeapID() << ")" << endl;
        }
        cout << endl;
    }

    class iterator
    {
    public:
        iterator(typename std::vector<Data>::iterator t) { _it = t; }
        iterator(const iterator& i) { _it = i._it; }
        ~iterator() {};

        const Data& operator * () const { return *_it; }
        iterator& operator ++ () {  ++_it; return *this; }
        iterator& operator ++ (int) { iterator ret = *this; _it++; return ret; }
        iterator& operator -- () {  --_it; return *this; }
        iterator& operator -- (int) { iterator ret = *this; _it--; return ret; }
        bool operator != (const iterator& i) const { return _it != i._it; }
        bool operator == (const iterator& i) const { return _it == i._it; }
        iterator& operator = (const iterator& i) { _it = i._it; }
    private:
        typename std::vector<Data>::iterator _it;
    };

    iterator begin() { return ++(_data.begin()); }
    iterator end() { return _data.end(); }

private:
    vector<Node> _data;
};

#endif /* __MINHEAP_H__ */
