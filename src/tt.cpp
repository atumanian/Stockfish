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

#include <cstring>   // For std::memset
#include <iostream>

#include "bitboard.h"
#include "tt.h"

TranspositionTable TT; // Our global transposition table


/// TranspositionTable::resize() sets the size of the transposition table,
/// measured in megabytes. Transposition table consists of a power of 2 number
/// of clusters and each cluster consists of ClusterSize number of TTEntry.

void TranspositionTable::resize(size_t mbSize) {

  size_t newClusterCount = mbSize * 1024 * 1024 / sizeof(Cluster);

  if (newClusterCount == clusterCount)
      return;

  clusterCount = newClusterCount;
  clusterIndexMask = newClusterCount - 1;

  free(mem);
  mem = malloc(clusterCount * sizeof(Cluster) + CacheLineSize - 1);

  if (!mem)
  {
      std::cerr << "Failed to allocate " << mbSize
                << "MB for transposition table." << std::endl;
      exit(EXIT_FAILURE);
  }

  table = (Cluster*)((uintptr_t(mem) + CacheLineSize - 1) & ~(CacheLineSize - 1));
  clear();
}


/// TranspositionTable::clear() overwrites the entire transposition table
/// with zeros. It is called whenever the table is resized, or when the
/// user asks the program to clear the table (from the UCI interface).

void TranspositionTable::clear() {

  std::memset(table, 0, clusterCount * sizeof(Cluster));
}


/// TranspositionTable::probe() looks up the current position in the transposition
/// table. It returns true and a pointer to the TTEntry if the position is found.
/// Otherwise, it returns false and a pointer to an empty or least valuable TTEntry
/// to be replaced later. The replace value of an entry is calculated as its depth
/// minus 8 times its relative age. TTEntry t1 is considered more valuable than
/// TTEntry t2 if its replace value is greater than that of t2.

TTEntry* TranspositionTable::probe(const Key k, TTEntry::Data& ttData) const {
  TTEntry* const tte = first_entry(k);

  for (int i = 0; i < ClusterSize; ++i) {
      if (!tte[i].key)
        return ttData.empty(), &tte[i];
      TTEntry::Data rdata = tte[i].data;
      Key key = tte[i].key;

      if (key == k) {
        if (rdata.generation() != generation8) {
             rdata.set_generation(generation8);
             tte[i].key = key;
             tte[i].data = rdata;
        }
        return ttData = rdata, &tte[i];
      }
  }

  // Find an entry to be replaced according to the replacement strategy
  TTEntry* replace = tte;
  // Due to our packed storage format for generation and its cyclic
  // nature we add 259 (256 is the modulus plus 3 to keep the lowest
  // two bound bits from affecting the result) to calculate the entry
  // age correctly even after generation8 overflows into the next cycle.
  int entryValue = tte[0].data.importance(generation8);
  for (int i = 1; i < ClusterSize; ++i) {
      int newValue = tte[i].data.importance(generation8);
      if (entryValue > newValue) {
          entryValue = newValue;
          replace = &tte[i];
      }
  }
  return ttData.empty(), replace;
}


/// TranspositionTable::hashfull() returns an approximation of the hashtable
/// occupation during a search. The hash is x permill full, as per UCI protocol.

int TranspositionTable::hashfull() const {

  int cnt = 0;
  for (int i = 0; i < 1000 / ClusterSize; i++)
  {
      const TTEntry* tte = &table[i].entry[0];
      for (int j = 0; j < ClusterSize; j++)
          if (tte[j].data.generation() == generation8)
              cnt++;
  }
  return cnt;
}
