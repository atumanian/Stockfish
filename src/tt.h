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

struct TTEntry {

  struct Data {
    Move  move()  const { return Move(uint16_t(data >> 32)); }
    Value value() const { return Value(int64_t(data) >> 48); }
    Value eval()  const { return Value(int32_t(data) >> 16); }
    Depth depth() const { return Depth(int8_t(data) * int(ONE_PLY)); }
    uint16_t generation() const { return data & 0xFC00; }
    explicit operator int() { return data; }
    Bound bound() const { return Bound(data & 0x300); }
    Key operator^(Key keyXorData) const { return data ^ keyXorData; }
    void operator=(uint64_t d) { data = d; }
    void setGeneration(uint16_t gen) { data = (data & 0xFFFFFFFFFFFF03FF) | gen; }
    void setMove(Move m) { data = (data & 0xFFFF0000FFFFFFFF) | uint64_t(m) << 32; }
    void set(Move m, Value v, Value ev, Depth d, uint16_t g, Bound b) {
        data = uint64_t(v << 16 | m) << 32 | uint8_t(d) | b | g | uint32_t(ev << 16);
    }
    void set(Value v, Value ev, Depth d, uint16_t g, Bound b) {
        data = (data & 0xFFFF00000000) | uint64_t(v) << 48 | uint8_t(d) | b | g | uint32_t(ev << 16);
    }

  private:
    uint64_t data;
  };

  void save(Key k, Value v, Bound b, Depth d, Move m, Value ev, uint16_t g) {

    assert(d / ONE_PLY * ONE_PLY == d);
    Data rdata;
    Key key;
    read(key, rdata);

    if (k != key)
      rdata.set(m, v, ev, d, g, b);
    else if (d / ONE_PLY > rdata.depth() - 4
         /* || g != (genBound8 & 0xFC) // Matching non-zero keys are already refreshed by probe() */
            || b == BOUND_EXACT)
      m ? rdata.set(m, v, ev, d, g, b) : rdata.set(v, ev, d, g, b);
    else if (m)
      rdata.setMove(m);
    else return;

    write(k, rdata);
  }

private:
  friend class TranspositionTable;

  void read(Key& key, Data& rdata) const {
      rdata = data;
      key = rdata ^ keyXorData;
  }
  void write(Key key, Data rdata) {
      data = rdata;
      keyXorData = rdata ^ key;
  }

  uint64_t keyXorData;
  Data data;
};


/// A TranspositionTable consists of a power of 2 number of clusters and each
/// cluster consists of ClusterSize number of TTEntry. Each non-empty entry
/// contains information of exactly one position. The size of a cluster should
/// divide the size of a cache line size, to ensure that clusters never cross
/// cache lines. This ensures best cache performance, as the cacheline is
/// prefetched, as soon as possible.

class TranspositionTable {

  static const int CacheLineSize = 64;
  static const int ClusterSize = 4;

  struct Cluster {
    TTEntry entry[ClusterSize];
  };

  static_assert(CacheLineSize % sizeof(Cluster) == 0, "Cluster size incorrect");

public:
 ~TranspositionTable() { free(mem); }
  void new_search() { generation8 += 1024; } // Lower 2 bits are used by Bound
  uint16_t generation() const { return generation8; }
  TTEntry* probe(const Key key, TTEntry::Data& ttData, bool& found) const;
  int hashfull() const;
  void resize(size_t mbSize);
  void clear();

  // The 32 lowest order bits of the key are used to get the index of the cluster
  TTEntry* first_entry(const Key key) const {
    return &table[(uint32_t(key) * uint64_t(clusterCount)) >> 32].entry[0];
  }

private:
  size_t clusterCount;
  Key clusterIndexMask;
  Cluster* table;
  void* mem;
  uint16_t generation8; // Size must be not bigger than TTEntry::genBound8
};

extern TranspositionTable TT;

#endif // #ifndef TT_H_INCLUDED
