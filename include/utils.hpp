#ifndef SRC_GRID_LOCALISATION_INCLUDE_UTILS
#define SRC_GRID_LOCALISATION_INCLUDE_UTILS

namespace Utils
{
int map_to_index(const int &row, const int &col, const int &map_width)
{
    return row*map_width + col;
}
}

#endif /* SRC_GRID_LOCALISATION_INCLUDE_UTILS */
