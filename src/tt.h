/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2008 Tord Romstad (Glaurung author)
  Copyright (C) 2008-2015 Marco Costalba, Joona Kiiski, Tord Romstad
  Copyright (C) 2015-2018 Marco Costalba, Joona Kiiski, Gary Linscott, Tord Romstad

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TT_H_INCLUDED
#define TT_H_INCLUDED

#include "misc.h"
#include "types.h"

/// TTEntry struct is the 10 bytes transposition table entry, defined as below:
///
/// key        16 bit
/// move       16 bit
/// value      16 bit
/// eval value 16 bit
/// generation  6 bit
/// bound type  2 bit
/// depth       8 bit


/// A TranspositionTable consists of a power of 2 number of clusters and each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty entry
/// contains information of exactly one position. The size of a cluster should
/// divide the size of a cache line size, to ensure that clusters never cross
/// cache lines. This ensures best cache performance, as the cacheline is
/// prefetched, as soon as possible.

class TranspositionTable;
extern TranspositionTable TT;

class TranspositionTable {

  static const int CacheLineSize = 64;
  static const int ClusterSize = 3;

  typedef uint16_t Key16;

  struct Cluster;

public:

  struct Data {
    Move  move()  const { return Move(uint16_t(data >> 32)); }
    Value value() const { return Value(int64_t(data) >> 48); }
    Value eval()  const { return Value(int32_t(data) >> 16); }
    Depth depth() const { return Depth(int8_t(data) * int(ONE_PLY)); }
    Bound bound() const { return Bound(data & 0x300); }
    uint16_t generation() const { return data & 0xFC00; }
    void set_generation(uint16_t gen) { data = (data & 0xFFFFFFFFFFFF03FF) | gen; }
    void set_move(Move m) { data = (data & 0xFFFF0000FFFFFFFF) | uint64_t(m) << 32; }
    void set(Move m, Value v, Value ev, Depth d, uint16_t g, Bound b) {
        data = uint64_t(m) << 32 | pack(v, ev, d, g, b);
    }
    void set(Value v, Value ev, Depth d, uint16_t g, Bound b) {
        data = (data & 0xFFFF00000000) | pack(v, ev, d, g, b);
    }
    void empty() {
        set(MOVE_NONE, VALUE_NONE, VALUE_NONE, DEPTH_NONE, 0, BOUND_NONE);
    }
    int importance(uint16_t gen) const {
    	return depth() - (((0x103FF + gen - int(data)) & 0xFC00) >> 7);
    }

  private:
    static uint64_t pack(Value v, Value ev, Depth d, uint16_t g, Bound b) {
        return uint64_t(v) << 48 | uint32_t(ev << 16 | g | b | uint8_t(d));
    }

    uint64_t data;
    friend Cluster;
  };

private:
  struct Cluster {

    int probe(Key16 key16, Data& ttData, uint16_t g);

    void save(int i, Key16 k, Value v, Bound b, Depth d, Move m, Value ev, uint16_t g) {

      assert(d / ONE_PLY * ONE_PLY == d);
      Data rdata = data[i];
      Key16 rkey = key16[i];

      if (k != rkey)
        rdata.set(m, v, ev, d, g, b);
      else if (d / ONE_PLY > rdata.depth() - 4
            /* || g != (genBound8 & 0xFC) // Matching non-zero keys are already refreshed by probe() */
               || b == BOUND_EXACT)
        m ? rdata.set(m, v, ev, d, g, b) : rdata.set(v, ev, d, g, b);
      else if (m)
        rdata.set_move(m);
      else return;

      key16[i] = k;
      data[i] = rdata;
    }
  private:
    Key16 key16[ClusterSize];
  public:
    Data data[ClusterSize];
  };

  static_assert(CacheLineSize % sizeof(Cluster) == 0, "Cluster size incorrect");

public:
  struct Manager {
    Manager(Key key) {
      key16 = key >> 48;
      clusterPtr = TT.first_entry(key);
    }
    Data probe() {
       Data data;
       index = clusterPtr->probe(key16, data, TT.generation());
       return data;
    }
    void save(Value v, Bound b, Depth d, Move m, Value ev) {
      clusterPtr->save(index, key16, v, b, d, m, ev, TT.generation());
    }
  private:
    Key16 key16;
    Cluster *clusterPtr;
    int index;
  };

 ~TranspositionTable() { free(mem); }
  void new_search() { generation8 += 1024; } // Lower 2 bits are used by Bound
  int hashfull() const;
  void resize(size_t mbSize);
  void clear();

  Cluster* first_entry(const Key key) const {
    return &table[(uint32_t(key) * uint64_t(clusterCount)) >> 32];
  }

private:
  uint16_t generation() const { return generation8; }
  /*int probe(Key key, Data& ttData, bool& found) const {
    return first_entry(key)->probe(key, ttData, found, generation());
  }
  void save(int i, Key k, Value v, Bound b, Depth d, Move m, Value ev) {
    first_entry(k)->save(i, k, v, b, d, m, ev, generation());
  }*/

  // The 32 lowest order bits of the key are used to get the index of the cluster

  size_t clusterCount;
  Cluster* table;
  void* mem;
  uint16_t generation8; // Size must be not bigger than TTEntry::genBound8
};

#endif // #ifndef TT_H_INCLUDED
