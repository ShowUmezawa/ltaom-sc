#pragma once
#include <vector>
// TEASER e415c0d graph.cc used unqualified vector via an older PMC header.
// Current pinned PMC no longer leaks this using-declaration.
using std::vector;
