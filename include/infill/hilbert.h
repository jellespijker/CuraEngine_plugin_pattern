// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.
// Hilbert curve is loosely based on the Hilbert Curve from Smart Avionics branch of CuraEngine:
// https://github.com/smartavionics/CuraEngine/blob/mb-master/src/infill/HilbertInfill.cpp

#ifndef CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_HILBERT_H
#define CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_HILBERT_H

#include "infill/geometry.h"
#include "infill/pattern.h"

#include <polyclipping/clipper.hpp>
#include <spdlog/spdlog.h>
#include <tl/expected.hpp>

#include <algorithm>
#include <cmath>
#include <string_view>
#include <tuple>
#include <utility>

namespace pattern
{
struct Hilbert : public InfillPattern
{
    Hilbert(const ClipperLib::cInt line_distance) noexcept
        : line_distance_{ line_distance } {};

    [[nodiscard]] std::tuple<ClipperLib::Paths, ClipperLib::Paths> operator()(const ClipperLib::Paths& outlines) noexcept final
    {
        const auto aabb = geometry::getBoundingBox(outlines);
        if (! aabb.has_value())
        {
            spdlog::warn(aabb.error());
            return { {}, {} };
        }
        aabb_ = aabb.value();
        bounding_box_ = ClipperLib::Paths{ { aabb_.first, { aabb_.second.X, aabb_.first.Y }, aabb_.second, { aabb_.first.X, aabb_.second.Y } } };

        mesh_max_size_ = std::max(aabb_.second.X - aabb_.first.X, aabb_.second.Y - aabb_.first.Y);
        infill_origin_ = ClipperLib::IntPoint{ aabb_.first.X + (aabb_.second.X - aabb_.first.X) / 2, aabb_.first.Y + (aabb_.second.Y - aabb_.first.Y) / 2 };
        last_ = infill_origin_;

        ClipperLib::Paths result;
        try
        {
            const auto depth = std::ceil(std::log2(static_cast<double>(mesh_max_size_ / line_distance_)));
            const auto size = static_cast<ClipperLib::cInt>(std::exp2(depth) * line_distance_);
            result = generateCoordinates(outlines, size, depth);
        }
        catch (const std::exception& e)
        {
            spdlog::warn("{}", e.what());
            return { {}, {} };
        }

        return { result, {} };
    };

private:
    std::pair<ClipperLib::IntPoint, ClipperLib::IntPoint> aabb_;
    ClipperLib::Paths bounding_box_;
    ClipperLib::IntPoint infill_origin_;
    ClipperLib::IntPoint last_;
    ClipperLib::cInt mesh_max_size_{ 1000 };
    ClipperLib::cInt line_distance_{ 800 };

    ClipperLib::Paths generateCoordinates(const ClipperLib::Paths& outlines, const ClipperLib::cInt size, const int depth)
    {
        ClipperLib::Paths result;
        bool is_first_point = true;

        std::function<void(ClipperLib::cInt, ClipperLib::cInt, ClipperLib::cInt, ClipperLib::cInt, ClipperLib::cInt, ClipperLib::cInt, int)> hilbert
            = [&, this](ClipperLib::cInt x0, ClipperLib::cInt y0, ClipperLib::cInt xi, ClipperLib::cInt xj, ClipperLib::cInt yi, ClipperLib::cInt yj, int n)
        {
            if (n <= 0)
            {
                const auto x = x0 + (xi + yi) / 2;
                const auto y = y0 + (xj + yj) / 2;
                const ClipperLib::IntPoint current{ x, y };
                if (! geometry::inside(bounding_box_, current))
                {
                    last_ = current;
                    return;
                }

                if (! is_first_point)
                {
                    result.emplace_back(ClipperLib::Path{ last_, current });
                }

                last_ = current;
                is_first_point = false;
            }
            else
            {
                hilbert(x0, y0, yi / 2, yj / 2, xi / 2, xj / 2, n - 1);
                hilbert(x0 + xi / 2, y0 + xj / 2, xi / 2, xj / 2, yi / 2, yj / 2, n - 1);
                hilbert(x0 + xi / 2 + yi / 2, y0 + xj / 2 + yj / 2, xi / 2, xj / 2, yi / 2, yj / 2, n - 1);
                hilbert(x0 + xi / 2 + yi, y0 + xj / 2 + yj, -yi / 2, -yj / 2, -xi / 2, -xj / 2, n - 1);
            }
        };

        hilbert(infill_origin_.X - size / 2, infill_origin_.Y - size / 2, size, 0, 0, size, depth);
        return geometry::clip(result, false, outlines);
    };
};

} // namespace pattern

#endif // CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_HILBERT_H
