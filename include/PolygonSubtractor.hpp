#pragma once
#include "clipper/clipper.hpp"

//polygonA - polygonB
ClipperLib::Paths PolygonSubtract(const ClipperLib::Path& polygonA, const ClipperLib::Path& polygonB);
