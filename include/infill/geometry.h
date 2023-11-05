#ifndef CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_GEOMETRY_H
#define CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_GEOMETRY_H

#include <polyclipping/clipper.hpp>
#include <range/v3/empty.hpp>
#include <range/v3/front.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/join.hpp>
#include <tl/expected.hpp>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>
#include <string_view>
#include <type_traits>

template<typename T>
concept Numeric = std::is_integral_v<T> || std::is_floating_point_v<T>;

namespace ClipperLib
{

[[nodiscard]] ClipperLib::IntPoint operator-(const ClipperLib::IntPoint& p0) noexcept
{
    return { -p0.X, -p0.Y };
}

[[nodiscard]] ClipperLib::IntPoint operator+(const ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1) noexcept
{
    return { p0.X + p1.X, p0.Y + p1.Y };
}

[[nodiscard]] ClipperLib::IntPoint operator-(const ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1) noexcept
{
    return { p0.X - p1.X, p0.Y - p1.Y };
}

[[nodiscard]] ClipperLib::IntPoint operator*(const ClipperLib::IntPoint& p0, const Numeric auto i) noexcept
{
    return { static_cast<ClipperLib::cInt>(p0.X * i), static_cast<ClipperLib::cInt>(p0.Y * i) };
}

[[nodiscard]] ClipperLib::IntPoint operator*(const Numeric auto i, const ClipperLib::IntPoint& p0) noexcept
{
    return p0 * i;
}

[[nodiscard]] tl::expected<ClipperLib::IntPoint, std::string_view> operator/(const ClipperLib::IntPoint& p0, const Numeric auto i) noexcept
{
    if (i == 0)
    {
        return tl::make_unexpected("Can't divide by zero!");
    }
    return ClipperLib::IntPoint{ static_cast<ClipperLib::cInt>(p0.X / i), static_cast<ClipperLib::cInt>(p0.Y / i) };
}

[[nodiscard]] tl::expected<ClipperLib::IntPoint, std::string_view> operator/(const ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1) noexcept
{
    if (p1.X == 0 || p1.Y == 0)
    {
        return tl::make_unexpected("Can't divide by zero!");
    }
    return ClipperLib::IntPoint{ static_cast<ClipperLib::cInt>(p0.X / p1.X), static_cast<ClipperLib::cInt>(p0.Y / p1.Y) };
}

[[nodiscard]] ClipperLib::IntPoint& operator+=(ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1) noexcept
{
    p0.X += p1.X;
    p0.Y += p1.Y;
    return p0;
}

[[nodiscard]] ClipperLib::IntPoint& operator-=(ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1) noexcept
{
    p0.X -= p1.X;
    p0.Y -= p1.Y;
    return p0;
}
} // namespace ClipperLib

namespace pattern::geometry
{

[[nodiscard]] ClipperLib::IntPoint rotate(const ClipperLib::IntPoint& p0, const std::floating_point auto& rad_angle) noexcept
{
    const double cos_component = std::cos(rad_angle);
    const double sin_component = std::sin(rad_angle);
    return { static_cast<decltype(ClipperLib::IntPoint::X)>(cos_component * p0.X - sin_component * p0.Y),
             static_cast<decltype(ClipperLib::IntPoint::Y)>(sin_component * p0.X + cos_component * p0.Y) };
}

[[nodiscard]] tl::expected<std::pair<ClipperLib::IntPoint, ClipperLib::IntPoint>, std::string> getBoundingBox(const ClipperLib::Paths& paths) noexcept
{
    if (ranges::empty(paths) || ranges::empty(ranges::front(paths)))
    {
        return tl::make_unexpected("Paths cannot be empty");
    }

    ClipperLib::IntPoint p_min{ std::numeric_limits<ClipperLib::cInt>::max(), std::numeric_limits<ClipperLib::cInt>::max() };
    ClipperLib::IntPoint p_max{ std::numeric_limits<ClipperLib::cInt>::min(), std::numeric_limits<ClipperLib::cInt>::min() };
    for (const auto& point : paths | ranges::view::join)
    {
        p_min.X = std::min(point.X, p_min.X);
        p_min.Y = std::min(point.Y, p_min.Y);
        p_max.X = std::max(point.X, p_max.X);
        p_max.Y = std::max(point.Y, p_max.Y);
    }

    return std::make_pair(p_min, p_max);
}

[[nodiscard]] bool inside(const ClipperLib::Path& path, ClipperLib::IntPoint point, const bool border_result = false) noexcept
{
    const auto res = ClipperLib::PointInPolygon(point, path);
    return (res == -1) ? border_result : (res == 1);
}

[[nodiscard]] bool inside(const ClipperLib::Paths& paths, ClipperLib::IntPoint point, const bool border_result = false) noexcept
{
    for (const auto& path : paths)
    {
        if (inside(path, point, border_result))
        {
            return true;
        }
    }
    return false;
}


ClipperLib::Paths clip(const auto& polys, const bool& is_poly_closed, const ClipperLib::Paths& outer_contours)
{
    ClipperLib::Clipper clipper;
    clipper.AddPaths(outer_contours, ClipperLib::PolyType::ptClip, true);

    ClipperLib::Paths grid_poly;
    for (auto& poly : polys)
    {
        grid_poly.push_back(poly);
    }
    clipper.AddPaths(grid_poly, ClipperLib::PolyType::ptSubject, is_poly_closed);

    ClipperLib::Paths ret;
    if (! is_poly_closed)
    {
        ClipperLib::PolyTree result;
        clipper.Execute(ClipperLib::ClipType::ctIntersection, result);
        ClipperLib::OpenPathsFromPolyTree(result, ret);
    }
    else
    {
        clipper.Execute(ClipperLib::ClipType::ctIntersection, ret);
    }
    return ret;
}

} // namespace pattern::geometry

#endif // CURAENGINE_PLUGIN_PATTERN_INCLUDE_INFILL_GEOMETRY_H
