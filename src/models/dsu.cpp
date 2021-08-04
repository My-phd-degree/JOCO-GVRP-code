#include "models/dsu.hpp"

#include <stdio.h>
#include <stdlib.h>

using namespace models;
using namespace std;

DSU::DSU (unsigned int n_) : n(n_) {
  unsigned int m = n_ * sizeof(unsigned int);
  pred = (unsigned int*) malloc (m);
  rank = (unsigned int*) malloc (m);
  for (unsigned int i = 0; i < n; pred[i] = i, rank[i++] = 0);
}

void DSU::join (unsigned int i, unsigned int j) {
  if (i != j && i < n && j < n) {
    unsigned int a = findSet(i);
    unsigned int b = findSet(j);
    if (rank[a] < rank[b]) 
      swap(a, b);
    pred[b] = a;
    if (rank[a] == rank[b])
      rank[a]++;
  }
}

void DSU::swap (unsigned int &a, unsigned int &b) {
  a ^= b;
  b ^= a;
  a ^= b;
}

unsigned int DSU::findSet (unsigned int i) {
  if (pred[i] == i)
    return i;
  return pred[i] = pred[findSet(pred[i])];
}

void DSU::clean () {
  for (unsigned int i = 0; i < n; pred[i] = i, rank[i++] = 0);
}
